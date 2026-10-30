# Lane Detection on the DE2-115 FPGA (Verilog)

Lane detection in Verilog on the Terasic DE2-115 board (Intel Cyclone IV E EP4CE115F29C7), with the result shown on a VGA monitor. No soft processor and no external memory are used.

The design follows the algorithms of:

> H. Yun and D. Park, "Low-Power Lane Detection Unit With Sliding-Based Parallel Segment Detection Accelerator for FPGA," *IEEE Access*, vol. 12, 2024.

The repository contains two versions:

| Folder | Input | Output |
|---|---|---|
| [`with_camera/`](with_camera) | Live video from an OV7670 camera | Camera picture with the detected lanes drawn in red, in real time |
| [`without_camera/`](without_camera) | A road photo stored in on-chip memory (converted with Python) | Canny edges (white) and detected lanes (red) |

Both versions use the same processing chain:

```
RGB image -> gray (eq. 8: Y = (5R + 9G + 2B) / 16)
          -> Canny: Gaussian 5x5 -> Sobel + square root -> non-maximum suppression -> hysteresis
          -> Hough transform with a CORDIC rotator (101 angles, 1.79 deg steps), bottom half only
          -> strongest line in the left and right lane regions
          -> lanes drawn on a 640x480 @ 60 Hz VGA picture
```

---

## 1. `with_camera/` - OV7670 camera version

### Files

| File | Function |
|---|---|
| `lane_detection_top.v` | Top level: pins, PLL, resets, switches, frame memories |
| `pll_25m.v` | 50 MHz -> 25 MHz (VGA) and 25 MHz (camera XCLK) |
| `ov7670_config.v`, `sccb_master.v` | Camera register set-up over SCCB |
| `ov7670_capture.v` | Camera bytes -> RGB565 pixels |
| `async_fifo.v` | Camera clock -> 25 MHz clock (Gray-code pointers) |
| `gray_scale.v` | 640x480 -> 320x240 and gray value (eq. 8) |
| `lane_detection.v` | Canny -> edge list -> Hough; frame control |
| `canny_edge_detection.v`, `sliding_window.v`, `gaussian_smoothing.v`, `sobel_operator.v`, `square_root.v`, `non_max_suppression.v`, `hysteresis.v` | Canny edge detection |
| `hough_transform.v`, `cordic_rotator.v`, `local_maxima.v` | Hough transform and peak search (Algorithm 1) |
| `vga_display.v` | VGA timing, 2x enlargement, lane drawing, display modes |
| `dual_port_ram.v` | RAM template used for every memory (M9K) |
| `lane_detection.tcl` | Creates the Quartus project with all files, pins and timing constraints |

### Build and run (Quartus Prime 18.1 Lite)

1. Open a command prompt in `with_camera/` and run `quartus_sh -t lane_detection.tcl`.
   Or, in Quartus, open *View -> Utility Windows -> Tcl Console*, `cd` to the folder and run `source lane_detection.tcl`.
2. Open `lane_detection.qpf` and run *Processing -> Start Compilation*.
3. Program `lane_detection.sof` with *Tools -> Programmer* (USB-Blaster, JTAG mode).

### Connections

- **Camera:** GPIO header JP5. The header pin of every signal is listed in section 5 of `lane_detection.tcl`.
- **Camera power:** JP5 pin 29 (3.3 V) and pin 12 (GND). Do not use pin 11 (5 V).
- **Monitor:** the VGA connector of the board.

### Controls

| Control | Function |
|---|---|
| KEY0 / KEY1 | Reset / send the camera settings again |
| SW1..SW0 | 00 camera + lanes, 01 gray, 10 Canny edges, 11 edges + lanes |
| SW2 | Freeze picture and lanes |
| SW3 / SW4 | Mirror / flip the camera picture |
| SW9 | Peak search: 0 = highest cell per lane region, 1 = Algorithm 1 of the paper |
| SW17..SW10 | Hysteresis threshold (all down = 30) |
| LEDG0 / LEDG1 | Left / right lane found |

### Settings in the code

| What | File | Parameter |
|---|---|---|
| Line colour | `vga_display.v` | stage 4: `vga_r/g/b <= 255, 0, 0` |
| Line thickness | `vga_display.v` | `HALF_WIDTH` (Q12, 6144 = 3 px) |
| Where the lines start | `vga_display.v` | `roi0 = (vc >= 2*ROI_Y)`, `ROI_Y = 120` |
| Gap between the two lines | `vga_display.v` | `MIN_SEP` (Q12, 57344 = 14 px) |
| Rows used for detection | `lane_detection.v` | `ROI_Y = 120` |
| Lane angles, minimum votes | `hough_transform.v` | `LEFT_T0..RIGHT_T1`, `MIN_VOTES = 24` |

---

## 2. `without_camera/` - stored-image version

The road image is stored in four on-chip ROM banks (`quartus/img_bank0..3.mif`). The image is split into four horizontal segments that four Canny engines process in parallel, as in the sliding-based parallel segment detection of the paper. The result is shown on the VGA monitor, and can also be read back through JTAG and saved as a picture.

### Files

| Path | Function |
|---|---|
| `rtl/de2_115/de2_115_lane_top.v` | Top level: keys, LEDs, VGA, image ROM banks, result memories |
| `rtl/de2_115/de2_115_support.v` | ROM / RAM wrappers and VGA output for the DE2-115 |
| `rtl/lane_detect_core.v` | Lane detection core: 4 parallel Canny engines, Hough, drawing |
| `rtl/canny_stages.v`, `rtl/window_gen.v` | Canny stages and sliding windows (line memories) |
| `rtl/hough_unit.v`, `rtl/trig_lut.v` | CORDIC Hough transform and lane selection |
| `rtl/render_unit.v` | Draws the detected lanes in hardware |
| `rtl/sdp_ram.v` | Simple dual-port RAM template |
| `quartus/` | Quartus project (`lane_detect.qpf`, `.qsf` with all pins, `.sdc`) and the current image (`img_bank0..3.mif`) |
| `python/img_to_mem.py` | Converts any road photo to the four `.mif` files |
| `python/mem_to_img.py` | Converts the FPGA result memories back to pictures |
| `python/lane_params.py` | Shared parameters and file helpers for the two scripts |

### Build and run

1. Open `quartus/lane_detect.qpf` in Quartus and run *Processing -> Start Compilation*.
2. Connect a VGA monitor and program `quartus/output_files/lane_detect.sof`.
3. The detection runs once after programming. The monitor shows the edges in white and the lanes in red.
   - KEY0 = reset, KEY1 = run the detection again.
   - LEDG0 = done, LEDG1 / LEDG2 = left / right lane found.

### Use a different road image

Requires Python 3 with `numpy` and `pillow` (`pip install numpy pillow`).

1. In `without_camera/python/` run `python img_to_mem.py path/to/road.jpg`.
   - The image is resized to 320x240 and written to `quartus/img_bank0..3.mif`.
   - Add `--crop-top 0.3` to cut away the sky (top 30 %).
2. Compile again, or load the new `.mif` files without compiling:
   - Open *Tools -> In-System Memory Content Editor*.
   - For each memory IB0..IB3: *Import Data from File* -> the matching `img_bankN.mif` -> *Write Data to In-System Memory*.
   - Press KEY1.

### Save the result as a picture (optional)

1. In the In-System Memory Content Editor, read the memories **OUT** and **RES** and export them as `without_camera/fpga_out/OUT.mif` and `RES.mif`.
2. In `without_camera/python/` run `python mem_to_img.py fpga`.

---

## 3. Resources (Quartus 18.1, EP4CE115F29C7)

| | `with_camera` | `without_camera` |
|---|---|---|
| Logic elements | 3,665 (3 %) | 10,239 (9 %) |
| Memory bits (on-chip M9K) | 2,077,635 (52 %) | 2,702,279 (68 %) |
| Embedded 9-bit multipliers | 12 | 36 |
| Clock | 25 MHz (PLL) + camera clock | 50 MHz |

Timing is met in both versions.

## 4. Limitations

- Only straight lines are detected, so a curved lane is shown as its best straight fit.
- The region of interest is fixed (bottom half of the picture).
- `with_camera`: if no road is in view, the strongest slanted straight edge on each side is still drawn when it has at least `MIN_VOTES` votes.
- `without_camera`: the image size is fixed at 320x240, and a new image needs the Python conversion step.
