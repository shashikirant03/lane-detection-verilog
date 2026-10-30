"""
mem_to_img.py   --  STEP 3 of the flow (output memory file -> image)

Reads the output memory produced by
   * the ModelSim testbench   : sim/output_image.hex   ($readmemh text)
   * the FPGA (DE2-115)       : file exported from Quartus In-System Memory
                                Content Editor, instance "OUT" (.mif or .hex)
and converts it to images.  Each output word is 2 bits:
   bit0 = Canny edge pixel,  bit1 = pixel lies on a detected lane line

Optionally reads the 16-word result memory (sim/results.hex or the "RES"
instance exported from the FPGA) and prints the lane parameters / cycles.

Usage examples:
  python mem_to_img.py                  (no arguments: uses fpga_out/OUT.mif + RES.mif
                                         if they exist, otherwise sim/output_image.hex)
  python mem_to_img.py fpga             FPGA result   (fpga_out/OUT.mif, RES.mif)
  python mem_to_img.py sim              simulation result (sim/output_image.hex, results.hex)
  python mem_to_img.py ../sim/output_image.hex --results ../sim/results.hex
  python mem_to_img.py ../fpga_out/OUT.mif --results ../fpga_out/RES.mif --prefix ../fpga_out/fpga
"""
import argparse
import os
import numpy as np
from PIL import Image
import lane_params as P


def safe_save(img, path):
    """Save a PIL image.  If the old file is still open in the Windows Photos
    viewer (Windows then refuses to overwrite it: OSError Errno 22 / 13),
    save under a new name  <name>_1.png, _2.png ... instead."""
    base, ext = os.path.splitext(path)
    cand, n = path, 0
    while True:
        try:
            img.save(cand)
            if cand != path:
                print("note: %s is open in another program (close the picture "
                      "viewer), saved as %s" % (os.path.basename(path), os.path.basename(cand)))
            return cand
        except OSError:
            n += 1
            if n > 50:
                raise
            cand = "%s_%d%s" % (base, n, ext)


def decode_results(words):
    words = list(words) + [0] * (16 - len(words))
    txt = []
    if words[0] != 0x4C414E45:
        txt.append("  (warning: result magic word is %08X, expected 4C414E45)" % words[0])
    txt.append("  image %d x %d" % (words[1] >> 16, words[1] & 0xFFFF))
    for i, name in ((2, "left "), (4, "right")):
        w = words[i]
        found, k, b = (w >> 31) & 1, (w >> 16) & 0x7F, w & 0xFFFF
        txt.append("  %s lane: found=%d  k=%3d (theta=%6.2f deg)  rho=%4d px  votes=%d"
                   % (name, found, k, P.theta_deg(k), b - P.RHO_OFF, words[i + 1]))
    txt.append("  cycles: canny=%d  hough(clear+vote+max)=%d  render=%d  total=%d"
               % (words[6], words[7], words[8], words[9]))
    txt.append("  total time @50 MHz = %.3f ms ; edge pixels voted in ROI = %d"
               % (words[9] * 20e-9 * 1e3, words[10]))
    return "\n".join(txt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("outmem", nargs="?", default=None,
                    help="output memory file (.hex / .mif), or 'fpga' / 'sim'. "
                         "Default: fpga_out/OUT.mif if it exists, else sim/output_image.hex")
    ap.add_argument("--results", help="16-word result memory file (.hex / .mif)")
    ap.add_argument("--orig", default=os.path.join(P.WORK_DIR, "input_resized.png"),
                    help="original (resized) image for the overlay picture")
    ap.add_argument("--prefix", default=None, help="output file prefix")
    ap.add_argument("--scale", type=int, default=2, help="up-scaling of saved images")
    ap.add_argument("--no-open", action="store_true", help="do not open the pictures")
    args = ap.parse_args()

    # ---- choose the input automatically when it is not given ----
    fpga_out = os.path.join(P.ROOT, "fpga_out", "OUT.mif")
    fpga_res = os.path.join(P.ROOT, "fpga_out", "RES.mif")
    sim_out = os.path.join(P.SIM_DIR, "output_image.hex")
    sim_res = os.path.join(P.SIM_DIR, "results.hex")
    choice = args.outmem
    if choice is None:
        choice = "fpga" if os.path.exists(fpga_out) else "sim"
    if choice.lower() == "fpga":
        args.outmem = fpga_out
        args.results = args.results or fpga_res
        args.prefix = args.prefix or os.path.join(P.ROOT, "fpga_out", "fpga")
    elif choice.lower() == "sim":
        args.outmem = sim_out
        args.results = args.results or sim_res
    if not os.path.exists(args.outmem):
        raise SystemExit("output file not found: %s\n"
                         "  FPGA: export OUT and RES with the In-System Memory Content Editor "
                         "into fpga_out\\OUT.mif and fpga_out\\RES.mif\n"
                         "  simulation: run the ModelSim simulation first" % args.outmem)
    if args.results and not os.path.exists(args.results):
        print("note: result file %s not found, lane values are not printed" % args.results)
        args.results = None
    print("reading", args.outmem)
    prefix = args.prefix or os.path.splitext(args.outmem)[0]

    mem = P.read_mem_auto(args.outmem, P.OUT_BITS)
    if len(mem) < P.W * P.H:
        raise SystemExit("output memory has %d words, expected %d" % (len(mem), P.W * P.H))
    code = np.array(mem[:P.W * P.H], dtype=np.int64).reshape(P.H, P.W) & 3
    edge, lane = (code & 1) > 0, (code & 2) > 0

    # 1) edges (white) + lanes (red) on black -- needs only the output file
    a = np.zeros((P.H, P.W, 3), np.uint8)
    a[edge] = (255, 255, 255)
    a[lane] = (255, 0, 0)
    im = Image.fromarray(a)
    im = im.resize((P.W * args.scale, P.H * args.scale), Image.NEAREST)
    p1 = safe_save(im, prefix + "_edges_lanes.png")

    # 2) edge map only
    p2 = safe_save(Image.fromarray((edge * 255).astype(np.uint8)).resize(
        (P.W * args.scale, P.H * args.scale), Image.NEAREST), prefix + "_edges.png")

    # 3) lanes drawn on the original colour image (like Fig. 6 of the paper)
    made = [p1, p2]
    if args.orig and os.path.exists(args.orig):
        o = np.asarray(Image.open(args.orig).convert("RGB").resize((P.W, P.H))).copy()
        o[lane] = (255, 0, 0)
        p3 = safe_save(Image.fromarray(o).resize((P.W * args.scale, P.H * args.scale),
                                                 Image.NEAREST), prefix + "_overlay.png")
        made.append(p3)

    print("edge pixels: %d   lane pixels: %d" % (int(edge.sum()), int(lane.sum())))
    if args.results:
        print(decode_results(P.read_mem_auto(args.results, 32)))
    for m in made:
        print("wrote", m)
    if not args.no_open and hasattr(os, "startfile"):      # Windows: show the pictures
        for m in made[::-1]:
            try:
                os.startfile(m)
            except OSError:
                pass


if __name__ == "__main__":
    main()
