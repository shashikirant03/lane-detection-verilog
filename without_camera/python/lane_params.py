"""
lane_params.py
==============
Single source of truth for every parameter shared between the Python tools
and the Verilog RTL.  If you change anything here you MUST change the same
parameter in rtl/lane_detect_core.v (module parameters).

Paper: H. Yun, D. Park, "Low-Power Lane Detection Unit With Sliding-Based
Parallel Segment Detection Accelerator for FPGA", IEEE Access, vol. 12, 2024.
"""
import math
import os

# ----------------------------------------------------------------------------
# Image / parallelism
# ----------------------------------------------------------------------------
W = 320            # image width  (pixels)
H = 240            # image height (pixels)
T = 4              # number of parallel Canny engines ("threads" in the paper)
S = H // T         # rows owned by each engine (image segment height)
HALO = 5           # overlap rows fetched from neighbouring segments (2+1+1+1)
BORDER = 5         # pixels at the image border forced to "no edge"
LROWS = S + 2 * HALO
BANK_DEPTH = S * W # words per input bank / edge-memory depth

# ----------------------------------------------------------------------------
# Canny
# ----------------------------------------------------------------------------
TH_HIGH = 160      # hysteresis high threshold (on sqrt(Gx^2+Gy^2), range 0..1442)
TH_LOW = 80        # hysteresis low  threshold
GAUSS_RECIP = 3841 # round(2^20 / 273)  -> blur = (sum * 3841) >> 20
GAUSS_K = [[1, 4, 7, 4, 1],
           [4, 16, 26, 16, 4],
           [7, 26, 41, 26, 7],
           [4, 16, 26, 16, 4],
           [1, 4, 7, 4, 1]]
TAN_22_5_Q8 = 106  # tan(22.5 deg) * 256
TAN_67_5_Q8 = 618  # tan(67.5 deg) * 256

# ----------------------------------------------------------------------------
# Hough (CORDIC-style incremental rotation, tan(dphi) = 2^-5  -> 1.79 deg)
# ----------------------------------------------------------------------------
ROI_Y = H // 2           # only the bottom half votes (paper optimisation #1)
CORDIC_SHIFT = 5
DPHI = math.atan(2.0 ** -CORDIC_SHIFT)       # 0.031240 rad = 1.7899 deg
NTH = 101                # theta steps 0 .. 100*1.79 = 179 deg
F = 10                   # fractional bits of the rotator registers
RHO_MAX = int(math.ceil(math.hypot(W / 2, H / 2)))   # 200
NRHO = 2 * RHO_MAX + 1   # 401 rho bins, 1 pixel each, origin = image centre
RHO_OFF = RHO_MAX
K_EXCL_LO = 45           # 80.5 deg  } near-horizontal lines are ignored
K_EXCL_HI = 55           # 98.4 deg  } (paper optimisation #2: 80 < th < 100)
K_LEFT_MIN = 10          # left lane : theta in [17.9, 75.2] deg
K_LEFT_MAX = 42          #   (near-vertical: poles/cars, near-horizontal: horizon/guard rails)
K_RIGHT_MIN = 58         # right lane: theta in [103.8, 161.1] deg
K_RIGHT_MAX = 90
LANE_FRAC_NUM = 3        # candidate lane needs votes*4 >= peak*3 (75 % of the region peak)
YB = H // 2              # reference point (0, YB) = bottom centre = the car position
# ---- drawing -----------------------------------------------------------------
EXT_TOP = int(H * 0.3)   # lanes may be extended above ROI_Y up to this row ...
MAX_GAP = 12             # ... following edge evidence; stop after 12 rows without evidence
HIT_TOL_Q14 = 24576      # edge pixel within 1.5 px of the line = evidence
GAP_Q14 = 12 * 16384     # a lane is never drawn closer than 12 px to the other lane
ACC_BITS = 9             # accumulator cell width (saturating)
MIN_VOTES = 20           # a lane is "found" only with at least this many votes
LINE_THICK_Q14 = 16384   # draw pixels whose distance to the line <= 1.0 px

OUT_BITS = 2             # output pixel code: bit1 = lane, bit0 = edge


def _rnd(v):
    """round-half-up (deterministic, same everywhere)"""
    return int(math.floor(v + 0.5))


def ginv_q15(k):
    """1/G^k in Q15 where G = sqrt(1+2^-10) is the gain of one micro-rotation"""
    g = (1.0 + 2.0 ** (-2 * CORDIC_SHIFT)) ** (k / 2.0)
    return _rnd(32768.0 / g)


def cos_q14(k):
    return _rnd(math.cos(k * DPHI) * 16384.0)


def sin_q14(k):
    return _rnd(math.sin(k * DPHI) * 16384.0)


GINV = [ginv_q15(k) for k in range(NTH)]
COS = [cos_q14(k) for k in range(NTH)]
SIN = [sin_q14(k) for k in range(NTH)]


def theta_deg(k):
    return math.degrees(k * DPHI)


# ----------------------------------------------------------------------------
# Paths
# ----------------------------------------------------------------------------
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SIM_DIR = os.path.join(ROOT, "sim")
QUARTUS_DIR = os.path.join(ROOT, "quartus")
WORK_DIR = os.path.join(ROOT, "work")


# ----------------------------------------------------------------------------
# Memory-file helpers (readmemh .hex, Quartus .mif, Intel .hex)
# ----------------------------------------------------------------------------
def write_readmemh(path, words, width):
    nd = (width + 3) // 4
    with open(path, "w") as f:
        for w in words:
            f.write("%0*X\n" % (nd, int(w)))


def write_mif(path, words, width):
    nd = (width + 3) // 4
    with open(path, "w") as f:
        f.write("-- generated by python/img_to_mem.py\n")
        f.write("WIDTH=%d;\nDEPTH=%d;\n" % (width, len(words)))
        f.write("ADDRESS_RADIX=UNS;\nDATA_RADIX=HEX;\n\nCONTENT BEGIN\n")
        for a, w in enumerate(words):
            f.write("\t%d : %0*X;\n" % (a, nd, int(w)))
        f.write("END;\n")


def _parse_num(tok, radix):
    tok = tok.strip()
    base = {"HEX": 16, "UNS": 10, "DEC": 10, "BIN": 2, "OCT": 8}.get(radix, 16)
    if radix == "DEC" and tok.startswith("-"):
        return int(tok, 10)
    return int(tok, base)


def read_mif(path):
    """Parse a Quartus .mif (also the files exported by the In-System Memory
    Content Editor).  Handles 'a : d;', '[a..b] : d;' and 'a : d0 d1 d2;'."""
    txt = open(path).read()
    # strip comments
    import re
    txt = re.sub(r"--[^\n]*", "", txt)
    txt = re.sub(r"%[^%]*%", "", txt)
    hdr = {}
    for key in ("WIDTH", "DEPTH", "ADDRESS_RADIX", "DATA_RADIX"):
        m = re.search(key + r"\s*=\s*([A-Za-z0-9]+)\s*;", txt, re.I)
        if m:
            hdr[key] = m.group(1).upper()
    depth = int(hdr["DEPTH"])
    arad = hdr.get("ADDRESS_RADIX", "HEX")
    drad = hdr.get("DATA_RADIX", "HEX")
    body = re.search(r"CONTENT\s+BEGIN(.*?)END\s*;", txt, re.S | re.I).group(1)
    mem = [0] * depth
    for stmt in body.split(";"):
        stmt = stmt.strip()
        if not stmt or ":" not in stmt:
            continue
        a, d = stmt.split(":", 1)
        a = a.strip()
        data = d.split()
        if a.startswith("["):
            lo, hi = a[1:-1].split("..")
            lo, hi = _parse_num(lo, arad), _parse_num(hi, arad)
            vals = [_parse_num(x, drad) for x in data]
            for i, addr in enumerate(range(lo, hi + 1)):
                mem[addr] = vals[i % len(vals)]
        else:
            addr = _parse_num(a, arad)
            for i, x in enumerate(data):
                mem[addr + i] = _parse_num(x, drad)
    return mem


def read_readmemh(path, depth=None):
    words, addr, mem = [], 0, {}
    for line in open(path):
        line = line.split("//")[0]
        for tok in line.split():
            if tok.startswith("@"):
                addr = int(tok[1:], 16)
                continue
            if set(tok) <= set("xXzZ"):
                val = 0
            else:
                val = int(tok, 16)
            mem[addr] = val
            addr += 1
    n = depth if depth is not None else (max(mem) + 1 if mem else 0)
    return [mem.get(i, 0) for i in range(n)]


def read_intel_hex(path, width):
    """Intel HEX as exported by Quartus (word addressed, one word per record)."""
    bpw = (width + 7) // 8
    mem, base = {}, 0
    for line in open(path):
        line = line.strip()
        if not line.startswith(":"):
            continue
        n = int(line[1:3], 16)
        addr = int(line[3:7], 16)
        typ = int(line[7:9], 16)
        data = bytes.fromhex(line[9:9 + 2 * n])
        if typ == 0:
            for i in range(0, n, bpw):
                mem[base + addr + i // bpw] = int.from_bytes(data[i:i + bpw], "big")
        elif typ == 2:
            base = int.from_bytes(data, "big") * 16
        elif typ == 4:
            base = int.from_bytes(data, "big") << 16
        elif typ == 1:
            break
    n = max(mem) + 1 if mem else 0
    return [mem.get(i, 0) for i in range(n)]


def read_mem_auto(path, width):
    """Read .mif, Intel .hex (lines starting with ':') or $readmemh text."""
    if path.lower().endswith(".mif"):
        return read_mif(path)
    with open(path) as f:
        first = ""
        for line in f:
            if line.strip():
                first = line.strip()
                break
    if first.startswith(":"):
        return read_intel_hex(path, width)
    return read_readmemh(path)
