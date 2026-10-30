"""
img_to_mem.py   --  STEP 1 of the flow (image -> memory files)

Resizes any image (jpg/png/bmp...) to W x H, then writes:

  sim/img_banks.hex          all T banks concatenated, 24-bit RGB, $readmemh
                             format -> used by the ModelSim testbench
  quartus/img_bank<j>.mif    one Quartus MIF per bank (j = 0..T-1)
                             -> initial content of the on-chip ROMs
  work/input_resized.png     the W x H image actually processed
  work/input_rgb.hex         whole image, 24-bit per line (for reference)

Bank j holds image rows [j*S, (j+1)*S) : this is the "segment" of the
paper that the j-th parallel Canny engine owns.

Usage:
  python img_to_mem.py path/to/road.jpg
  python img_to_mem.py path/to/road.jpg --crop-top 0.3
"""
import argparse
import os
import numpy as np
from PIL import Image
import lane_params as P


def load_resized(path, crop_top=0.0, crop_bottom=0.0):
    """Open any image, optionally cut away a fraction of the top (sky) and/or
    the bottom (car bonnet / dashboard), then resize to W x H."""
    img = Image.open(path).convert("RGB")
    if crop_top > 0 or crop_bottom > 0:
        w, h = img.size
        img = img.crop((0, int(h * crop_top), w, h - int(h * crop_bottom)))
    if img.size != (P.W, P.H):
        img = img.resize((P.W, P.H), Image.LANCZOS)
    return np.asarray(img).astype(np.int64)


def rgb_words(a):
    return (a[..., 0] << 16) | (a[..., 1] << 8) | a[..., 2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image")
    ap.add_argument("--sim-dir", default=P.SIM_DIR)
    ap.add_argument("--quartus-dir", default=P.QUARTUS_DIR)
    ap.add_argument("--work-dir", default=P.WORK_DIR)
    ap.add_argument("--crop-top", type=float, default=0.0,
                    help="fraction of the image height to cut from the top (e.g. 0.2)")
    ap.add_argument("--crop-bottom", type=float, default=0.0,
                    help="fraction to cut from the bottom, e.g. 0.15 to remove the car bonnet")
    args = ap.parse_args()
    for d in (args.sim_dir, args.quartus_dir, args.work_dir):
        os.makedirs(d, exist_ok=True)

    if not os.path.exists(args.image):
        folder = os.path.dirname(args.image) or "."
        print("ERROR: image not found: %s" % args.image)
        if os.path.isdir(folder):
            print("Images in %s :" % folder)
            for f in sorted(os.listdir(folder)):
                if f.lower().endswith((".jpg", ".jpeg", ".png", ".bmp")):
                    print("   ", f)
        raise SystemExit(1)
    a = load_resized(args.image, args.crop_top, args.crop_bottom)
    Image.fromarray(a.astype(np.uint8)).save(os.path.join(args.work_dir, "input_resized.png"))
    words = rgb_words(a)                                  # H x W
    P.write_readmemh(os.path.join(args.work_dir, "input_rgb.hex"), words.flatten(), 24)

    all_banks = []
    for j in range(P.T):
        bank = words[j * P.S:(j + 1) * P.S, :].flatten()  # S*W words
        all_banks.extend(bank.tolist())
        P.write_mif(os.path.join(args.quartus_dir, "img_bank%d.mif" % j), bank, 24)
    P.write_readmemh(os.path.join(args.sim_dir, "img_banks.hex"), all_banks, 24)

    print("image %s -> %dx%d, %d banks of %d words" % (args.image, P.W, P.H, P.T, P.BANK_DEPTH))
    print("  sim     :", os.path.join(args.sim_dir, "img_banks.hex"))
    print("  quartus :", ", ".join("img_bank%d.mif" % j for j in range(P.T)))


if __name__ == "__main__":
    main()
