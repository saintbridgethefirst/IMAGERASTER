#!/usr/bin/env python3
"""Turn any picture into a scan sheet for the robot AND the Digital Twin.

    python3 tools/make_sheet.py my_picture.png --width 150
    python3 tools/make_sheet.py logo.jpg --width 200 --bw          # black and white
    python3 tools/make_sheet.py drawing.png --height 120 --name bird
    python3 tools/make_sheet.py --use test_sheet                    # switch the twin back

Writes, in DigitalTwin/sheets/:
    <name>.pdf          print this at 100 % ("Actual size"); the page size is chosen for you
    <name>.svg          vector source (open in Inkscape)
    <name>_surface.png  the same sheet at 1 pixel per mm (what the twin scans)
    <name>_template.png the same sheet at 4 px/mm with its placement built in: load it in the
                        dashboard's "Compare with your template" to compare it with a robot scan
    <name>.json         where the sheet sits in the world, and the scan settings for it
and makes it the twin's active sheet (DigitalTwin/surface.png + surface.json),
unless you pass --no-install.

Layout (the robot's frame): the wheel-axle centre starts on the printed cross,
facing the arrow (+x); rows step to the robot's LEFT (+y, up the page). The
picture's bottom-left corner sits at x = 45 mm, y = 0 mm, just ahead of where the
middle sensor (DN3, 41 mm ahead of the axle) starts. The sheet also prints the
dashboard settings that cover the picture: Rows, Row length and Row pitch.

Needs Pillow and rsvg-convert (librsvg).
"""
import argparse
import base64
import io
import json
import math
import os
import shutil
import subprocess

from PIL import Image, ImageOps
from PIL.PngImagePlugin import PngInfo

HERE = os.path.dirname(os.path.abspath(__file__))
TWIN = os.path.normpath(os.path.join(HERE, '..', 'DigitalTwin'))

DN3_AHEAD = 41.0       # middle sensor, mm ahead of the axle
IMG_X0 = 45.0          # picture's left edge (world x, mm)
BAND = 9.0             # DN2/DN4 are 9 mm either side of DN3
PAGES = [('A4 landscape', 297, 210), ('A4 portrait', 210, 297),
         ('A3 landscape', 420, 297), ('A3 portrait', 297, 420)]


def write_template_png(svg_path, png_path, place):
    """The sheet at 4 px/mm, with its position in the robot's frame stored in the PNG
    (tEXt 'rasterbot-template'), so the dashboard can line it up with a scan."""
    subprocess.run(['rsvg-convert', '-f', 'png', '-d', '101.6', '-p', '101.6', '-b', 'white',
                    '-o', png_path, svg_path], check=True)
    im = Image.open(png_path).convert('L')
    meta = PngInfo()
    meta.add_text('rasterbot-template', json.dumps(place))
    im.save(png_path, pnginfo=meta, dpi=(101.6, 101.6), optimize=True)


def settings_for(w_mm, h_mm, pitch):
    rows = max(1, math.ceil((h_mm - BAND) / pitch) + 1)
    row_len = math.ceil(w_mm + 2 * (IMG_X0 - DN3_AHEAD))
    return rows, row_len


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('image', nargs='?')
    ap.add_argument('--use', metavar='NAME', help='make an existing sheet in DigitalTwin/sheets the active one')
    ap.add_argument('--width', type=float, help='printed picture width in mm (default 150)')
    ap.add_argument('--height', type=float, help='printed picture height in mm (instead of --width)')
    ap.add_argument('--pitch', type=float, default=7.0, help='row pitch to plan for, mm (default 7)')
    ap.add_argument('--bw', action='store_true', help='black and white (threshold) instead of grey')
    ap.add_argument('--threshold', type=int, default=128, help='0-255 threshold for --bw (default 128)')
    ap.add_argument('--invert', action='store_true', help='swap black and white')
    ap.add_argument('--name', help='sheet name (default: the image file name)')
    ap.add_argument('--no-install', action='store_true', help="don't make it the twin's active sheet")
    a = ap.parse_args()
    if a.use:
        src = os.path.join(TWIN, 'sheets', a.use)
        shutil.copyfile(src + '_surface.png', os.path.join(TWIN, 'surface.png'))
        shutil.copyfile(src + '.json', os.path.join(TWIN, 'surface.json'))
        print(f'{a.use} is now the twin\'s sheet (restart the sketch).')
        return
    if not a.image:
        ap.error('give an image file, or --use NAME')

    name = a.name or os.path.splitext(os.path.basename(a.image))[0]
    name = ''.join(c if c.isalnum() or c in '-_' else '_' for c in name)

    # ---- the picture ---------------------------------------------------------
    im = Image.open(a.image)
    if im.mode in ('RGBA', 'LA', 'P'):
        im = im.convert('RGBA')
        bg = Image.new('RGBA', im.size, (255, 255, 255, 255))
        im = Image.alpha_composite(bg, im)
    im = im.convert('L')
    if a.invert:
        im = ImageOps.invert(im)
    aspect = im.height / im.width
    if a.height:
        h_mm = a.height
        w_mm = h_mm / aspect
    else:
        w_mm = a.width or 150.0
        h_mm = w_mm * aspect
    px_per_mm = 10
    im = im.resize((max(1, round(w_mm * px_per_mm)), max(1, round(h_mm * px_per_mm))), Image.LANCZOS)
    if a.bw:
        im = im.point(lambda v: 255 if v >= a.threshold else 0)
    buf = io.BytesIO()
    im.save(buf, 'PNG')
    data = base64.b64encode(buf.getvalue()).decode()

    rows, row_len = settings_for(w_mm, h_mm, a.pitch)
    top_rows = (rows - 1) * a.pitch

    # ---- page --------------------------------------------------------------------
    # Content box in world mm: start marks on the left; ruler and title below,
    # out of the sensors' reach (DN1/DN5 are 27 mm either side of the rows).
    cx0, cx1 = -58.0, IMG_X0 + w_mm + 15
    cy0, cy1 = -60.0, max(h_mm, top_rows + 27) + 8
    need_w, need_h = cx1 - cx0, cy1 - cy0
    page = next(((n, pw, ph) for n, pw, ph in PAGES if need_w <= pw - 10 and need_h <= ph - 10), None)
    if page is None:
        raise SystemExit(f'Picture too big for A3 ({w_mm:.0f} x {h_mm:.0f} mm). Use a smaller --width.')
    page_name, PW, PH = page
    left = cx0 - (PW - need_w) / 2          # world x of the page's left edge
    bottom = cy0 - (PH - need_h) / 2        # world y of the page's bottom edge

    def P(x, y):
        return x - left, (bottom + PH) - y

    def line(x0, y0, x1, y1, stroke='#666', sw=0.3, dash=None):
        a_, b_ = P(x0, y0)
        c_, d_ = P(x1, y1)
        d = f' stroke-dasharray="{dash}"' if dash else ''
        return f'<line x1="{a_:.3f}" y1="{b_:.3f}" x2="{c_:.3f}" y2="{d_:.3f}" stroke="{stroke}" stroke-width="{sw}"{d}/>'

    def text(x, y, s, size=3.0, anchor='start', fill='#555', weight='normal'):
        a_, b_ = P(x, y)
        return (f'<text x="{a_:.3f}" y="{b_:.3f}" font-family="DejaVu Sans, Arial, sans-serif" font-size="{size}" '
                f'font-weight="{weight}" text-anchor="{anchor}" fill="{fill}">{s}</text>')

    grey = '#9aa3ad'
    el = [f'<rect x="0" y="0" width="{PW}" height="{PH}" fill="#fff"/>']
    ix, iy = P(IMG_X0, h_mm)
    el.append(f'<image x="{ix:.3f}" y="{iy:.3f}" width="{w_mm:.3f}" height="{h_mm:.3f}" '
              f'preserveAspectRatio="none" href="data:image/png;base64,{data}"/>')
    # Start: axle cross, wheel ticks, arrow, label. All at x < 0, where the
    # sensors only pass during run-ins, so none of it ends up in the image.
    el.append(line(-6, 0, 6, 0, '#555', 0.4))
    el.append(line(0, -6, 0, 6, '#555', 0.4))
    for side in (1, -1):                       # wheel centres, 41 mm either side
        el.append(line(-9, side * 41.25, -1, side * 41.25, grey, 0.8))
    for x0, y0, x1, y1 in [(-46, -8, -14, -8), (-14, -8, -18, -5.5), (-14, -8, -18, -10.5)]:
        el.append(line(x0, y0, x1, y1, '#777', 0.6))
    el.append(text(-56, -14, 'START: wheel-axle centre on +', 2.6, 'start', '#555', 'bold'))
    el.append(text(-56, -17.5, 'facing the arrow; grey ticks', 2.4))
    el.append(text(-56, -20.8, 'under the wheels. Next row is', 2.4))
    el.append(text(-56, -24.1, "on the robot's LEFT (up).", 2.4))
    # Ruler from the start axle (wheel-radius distance check), below the scan.
    ry = -36.0
    rlen = cx1 - 3
    el.append(line(0, ry, rlen, ry, '#444', 0.3))
    for mm in range(0, int(rlen) + 1):
        hgt = 5 if mm % 10 == 0 else (3.5 if mm % 5 == 0 else 2)
        el.append(line(mm, ry, mm, ry - hgt, '#444', 0.3 if mm % 10 == 0 else 0.15))
        if mm % 50 == 0:
            el.append(text(mm, ry - 8.0, str(mm), 2.5, 'middle', '#444'))
    # Title with the dashboard settings that cover the picture.
    el.append(text(-56, -51, f'{name}', 3.6, 'start', '#333', 'bold'))
    el.append(text(-56, -56, f'Dashboard: Rows {rows}, Row length {row_len} mm, Row pitch {a.pitch:g} mm, '
                   f'Next row on the left.  Print at 100 % on {page_name.split()[0]}.', 2.8))

    svg_body = '\n  '.join(el)
    svg = (f'<?xml version="1.0" encoding="UTF-8"?>\n<svg xmlns="http://www.w3.org/2000/svg" '
           f'xmlns:xlink="http://www.w3.org/1999/xlink" width="{PW}mm" height="{PH}mm" '
           f'viewBox="0 0 {PW} {PH}">\n  {svg_body}\n</svg>\n')

    out = os.path.join(TWIN, 'sheets')
    os.makedirs(out, exist_ok=True)
    svg_path = os.path.join(out, name + '.svg')
    pdf_path = os.path.join(out, name + '.pdf')
    png_path = os.path.join(out, name + '_surface.png')
    json_path = os.path.join(out, name + '.json')
    with open(svg_path, 'w') as f:
        f.write(svg)
    subprocess.run(['rsvg-convert', '-f', 'pdf', '-o', pdf_path, svg_path], check=True)
    subprocess.run(['rsvg-convert', '-f', 'png', '-d', '25.4', '-p', '25.4', '-b', 'white',
                    '-o', png_path, svg_path], check=True)
    info = {
        'sheet': name,
        'page': page_name,
        'offset_x_mm': round(left + PW / 2, 3),
        'offset_y_mm': round(bottom + PH / 2, 3),
        'picture_mm': [round(w_mm, 1), round(h_mm, 1)],
        'settings': {'pattern': 0, 'rows': rows, 'row_len': row_len, 'pitch': a.pitch, 'side': 1},
    }
    with open(json_path, 'w') as f:
        json.dump(info, f, indent=2)
    write_template_png(svg_path, os.path.join(out, name + '_template.png'),
                       {'sheet': name, 'x0_mm': round(left, 3), 'y0_mm': round(bottom, 3), 'w_mm': PW, 'h_mm': PH})
    if not a.no_install:
        shutil.copyfile(png_path, os.path.join(TWIN, 'surface.png'))
        shutil.copyfile(json_path, os.path.join(TWIN, 'surface.json'))

    print(f'{name}: picture {w_mm:.0f} x {h_mm:.0f} mm on {page_name}')
    print(f'  print:     {pdf_path}  (100 %, "Actual size")')
    print(f'  dashboard: Rows {rows}, Row length {row_len} mm, Row pitch {a.pitch:g} mm, next row on the left')
    print('  twin:      ' + ('installed as surface.png + surface.json' if not a.no_install else png_path))


if __name__ == '__main__':
    main()
