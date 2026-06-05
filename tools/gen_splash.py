#!/usr/bin/env python3
"""Generate a panel-native 4-bpp .bin splash for the Waveshare 7.3" Spectra 6
(ED2208-GCA) on the ESP32-S3 PhotoPainter -- 800 x 480 LANDSCAPE.

Reads a square PNG logo (alpha composited onto white), Floyd-Steinberg dithers
the 800x480 canvas to the firmware's 6-colour palette
(0=black, 1=white, 2=yellow, 3=red, 5=blue, 6=green), and packs to the
panel's 4-bpp scanline-order format (high nibble = even col, low = odd col),
producing exactly 192,000 bytes.

The firmware embeds the output via CMake's EMBED_FILES and streams it
straight to the panel with the existing epd_display() path.

Default layout (landscape):
  - logo:  300x300 px at left half, vertically centered
  - QR:    300x300 px at right half, vertically centered
  - labels: stacked under the QR if --label given

Usage:
    gen_splash.py --logo path/to/logo.png --out assets/splash_logo.bin
    gen_splash.py --logo path/to/logo.png --out assets/splash_portal.bin \\
                  --qr-data 'WIFI:T:WPA;S:Tesserae-Setup;P:tesserae;;' \\
                  --label 'Tesserae-Setup' --label 'pwd: tesserae'
"""
import argparse
import os
import sys
from PIL import Image, ImageDraw, ImageFont
import numpy as np

try:
    import qrcode  # only required when --qr-data is used
except ImportError:
    qrcode = None

# Order matters: tried in turn until one loads. Helvetica.ttc ships with macOS
# and renders cleanly at the sizes we use; SFNS is the system UI font.
_FONT_CANDIDATES = [
    "/System/Library/Fonts/Helvetica.ttc",
    "/System/Library/Fonts/HelveticaNeue.ttc",
    "/System/Library/Fonts/SFNSDisplay.ttf",
    "/Library/Fonts/Arial.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
]


def load_font(size: int) -> ImageFont.FreeTypeFont:
    for path in _FONT_CANDIDATES:
        if os.path.exists(path):
            return ImageFont.truetype(path, size=size)
    # Last-resort bitmap font; small but readable.
    return ImageFont.load_default()

PANEL_W, PANEL_H = 800, 480

# (nibble, RGB) -- matches firmware app_config.h palette
PALETTE = [
    (0x0, (  0,   0,   0)),   # black
    (0x1, (255, 255, 255)),   # white
    (0x2, (255, 255,   0)),   # yellow
    (0x3, (255,   0,   0)),   # red
    (0x5, (  0,   0, 255)),   # blue
    (0x6, (  0, 255,   0)),   # green
]
PALETTE_RGB = np.array([rgb for _, rgb in PALETTE], dtype=np.float32)
PALETTE_NIBBLE = np.array([n for n, _ in PALETTE], dtype=np.uint8)


def composite_logo(canvas: Image.Image, logo_path: str, logo_size: int,
                   logo_x: int, logo_y: int) -> None:
    """Alpha-composite a square PNG onto `canvas` at (logo_x, logo_y),
    resized to logo_size x logo_size."""
    logo = Image.open(logo_path).convert("RGBA")
    logo = logo.resize((logo_size, logo_size), Image.LANCZOS)
    bg = Image.new("RGB", logo.size, (255, 255, 255))
    bg.paste(logo, mask=logo.split()[3])
    canvas.paste(bg, (logo_x, logo_y))


def overlay_labels(canvas: Image.Image, labels, x_center: int, y_top: int,
                   font_px: int = 28, line_gap_px: int = 10,
                   colour=(0, 0, 0)) -> int:
    """Centered black text lines stacked vertically at horizontal center
    `x_center`. Returns y of the bottom of the last line."""
    font = load_font(font_px)
    draw = ImageDraw.Draw(canvas)
    y = y_top
    for line in labels:
        bbox = draw.textbbox((0, 0), line, font=font)
        text_w = bbox[2] - bbox[0]
        x = x_center - text_w // 2
        draw.text((x, y), line, fill=colour, font=font)
        y += (bbox[3] - bbox[1]) + line_gap_px
    return y


def overlay_qr(canvas: Image.Image, data: str, target_px: int,
               x_left: int, y_top: int, quiet_zone: int = 4) -> int:
    """Render a QR code for `data` at top-left (x_left, y_top), scaled so
    every module aligns to an integer pixel grid (clean dither output).
    Returns the bottom Y of the rendered QR."""
    if qrcode is None:
        sys.exit("--qr-data requires the `qrcode` Python package "
                 "(pip install qrcode)")
    qr = qrcode.QRCode(
        version=None,
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=1,
        border=quiet_zone,
    )
    qr.add_data(data)
    qr.make(fit=True)
    matrix = qr.get_matrix()    # list[list[bool]], includes quiet zone
    m_size = len(matrix)        # square; modules along one side incl. border
    module_px = target_px // m_size
    if module_px < 2:
        sys.exit(f"QR target_px={target_px} too small for {m_size}-module code "
                 f"(need at least {m_size * 2})")
    bmp_px = module_px * m_size

    bmp = Image.new("L", (bmp_px, bmp_px), 255)
    pixels = bmp.load()
    for my in range(m_size):
        for mx in range(m_size):
            if matrix[my][mx]:
                for dy in range(module_px):
                    for dx in range(module_px):
                        pixels[mx * module_px + dx, my * module_px + dy] = 0

    # Center the rendered QR inside the target_px frame -- the rendered size
    # is module_px*m_size which is module-rounded-down from target_px, so the
    # QR is slightly smaller than the requested frame. Centering it makes the
    # QR's actual center coincide with the target frame's center, so labels
    # placed at the frame center line up under the QR (not 12 px off).
    offset = (target_px - bmp_px) // 2
    canvas.paste(bmp.convert("RGB"), (x_left + offset, y_top + offset))
    print(f"  qr: {m_size}x{m_size} modules ({m_size - 2 * quiet_zone} data + "
          f"{quiet_zone}-module border), {module_px}px/module, "
          f"final {bmp_px}x{bmp_px} centered in {target_px}x{target_px} "
          f"at ({x_left + offset},{y_top + offset})")
    return y_top + target_px


def dither_to_nibbles(rgb: np.ndarray) -> np.ndarray:
    """Floyd-Steinberg dither rgb (H,W,3 float32) to the 6-colour palette.
    Returns an H x W uint8 array of palette nibble values."""
    h, w, _ = rgb.shape
    out = np.zeros((h, w), dtype=np.uint8)
    arr = rgb.copy()

    for y in range(h):
        for x in range(w):
            old = arr[y, x]
            dists = np.sum((PALETTE_RGB - old) ** 2, axis=1)
            idx = int(np.argmin(dists))
            new = PALETTE_RGB[idx]
            out[y, x] = PALETTE_NIBBLE[idx]
            err = old - new
            if x + 1 < w:
                arr[y, x + 1] += err * (7 / 16)
            if y + 1 < h:
                if x > 0:
                    arr[y + 1, x - 1] += err * (3 / 16)
                arr[y + 1, x] += err * (5 / 16)
                if x + 1 < w:
                    arr[y + 1, x + 1] += err * (1 / 16)
    return out


def pack_4bpp(nibbles: np.ndarray) -> bytes:
    """Pack H x W nibble array to panel-native 4-bpp scanline bytes:
    high nibble = even column, low = odd column."""
    hi = nibbles[:, 0::2].astype(np.uint8)
    lo = nibbles[:, 1::2].astype(np.uint8)
    packed = (hi << 4) | lo
    return packed.tobytes()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--logo", required=True, help="source PNG (square logo)")
    p.add_argument("--out",  required=True, help="output .bin (panel-native 4bpp)")
    p.add_argument("--logo-size", type=int, default=320,
                   help="logo edge in panel pixels (default 320)")
    p.add_argument("--logo-x", type=int, default=-1,
                   help="logo left X in panel pixels "
                        "(default: centered when no QR; left half centered with QR)")
    p.add_argument("--logo-y", type=int, default=-1,
                   help="logo top Y in panel pixels (default: vertically centered)")
    p.add_argument("--qr-data",   help="bake a QR code for this string into the splash "
                                       "(e.g. 'WIFI:T:WPA;S:Tesserae-Setup;P:tesserae;;')")
    p.add_argument("--qr-size",   type=int, default=320,
                   help="QR code target edge in panel pixels (default 320)")
    p.add_argument("--qr-x", type=int, default=-1,
                   help="QR left X (default: right half centered)")
    p.add_argument("--qr-y", type=int, default=-1,
                   help="QR top Y (default: vertically centered)")
    p.add_argument("--label",     action="append", default=[],
                   help="text line to render centered below the QR "
                        "(may be repeated to stack lines)")
    p.add_argument("--label-y",   type=int, default=-1,
                   help="top Y for the first label (default: 20px under QR)")
    p.add_argument("--label-px",  type=int, default=28,
                   help="label font size in panel pixels (default 28)")
    args = p.parse_args()

    if args.logo_size > PANEL_H:
        sys.exit(f"logo size {args.logo_size} > panel height {PANEL_H}")

    have_qr = bool(args.qr_data)

    # Default placement: logo centered when alone, left-half centered when QR.
    if args.logo_x < 0:
        args.logo_x = (PANEL_W // 2 - args.logo_size) // 2 if have_qr \
                      else (PANEL_W - args.logo_size) // 2
    if args.logo_y < 0:
        args.logo_y = (PANEL_H - args.logo_size) // 2

    if args.logo_x + args.logo_size > PANEL_W:
        sys.exit(f"logo at x={args.logo_x} size {args.logo_size} exceeds panel width {PANEL_W}")
    if args.logo_y + args.logo_size > PANEL_H:
        sys.exit(f"logo at y={args.logo_y} size {args.logo_size} exceeds panel height {PANEL_H}")

    canvas = Image.new("RGB", (PANEL_W, PANEL_H), (255, 255, 255))
    print(f"compositing {args.logo} at ({args.logo_x},{args.logo_y}) "
          f"size {args.logo_size}x{args.logo_size} on {PANEL_W}x{PANEL_H} white...")
    composite_logo(canvas, args.logo, args.logo_size, args.logo_x, args.logo_y)

    if have_qr:
        if args.qr_x < 0:
            args.qr_x = PANEL_W // 2 + (PANEL_W // 2 - args.qr_size) // 2
        if args.qr_y < 0:
            args.qr_y = (PANEL_H - args.qr_size) // 2
        print(f"baking QR for {args.qr_data!r}...")
        qr_bottom = overlay_qr(canvas, args.qr_data, args.qr_size,
                               args.qr_x, args.qr_y)
        if args.label:
            label_y = args.label_y if args.label_y >= 0 else qr_bottom + 12
            label_cx = args.qr_x + args.qr_size // 2
            print(f"baking {len(args.label)} label line(s) at y={label_y}...")
            overlay_labels(canvas, args.label, label_cx, label_y,
                           font_px=args.label_px)

    print("Floyd-Steinberg dithering to the 6-colour palette...")
    nibbles = dither_to_nibbles(np.array(canvas, dtype=np.float32))

    packed = pack_4bpp(nibbles)
    expected = PANEL_W * PANEL_H // 2
    assert len(packed) == expected, f"got {len(packed)} bytes, expected {expected}"

    with open(args.out, "wb") as f:
        f.write(packed)
    print(f"wrote {args.out} ({len(packed)} bytes)")


if __name__ == "__main__":
    main()
