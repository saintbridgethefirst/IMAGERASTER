#!/usr/bin/env python3
"""Make the ImageRaster test sheet: one design, three files.

    DigitalTwin/test_sheet.svg     vector source (open/edit in Inkscape)
    DigitalTwin/test_sheet_A3.pdf  print this at 100 % ("Actual size") on A3
    DigitalTwin/surface.png        the same sheet at 1 pixel per mm for the Digital Twin

World frame (the robot's): wheel-axle centre at (0, 0) at the start, facing +x,
rows stepping to +y (the robot's left, "Next row is on: the left"). The default
scan (17 rows x 160 mm at 7 mm pitch) covers x 41..201 mm, y -9..121 mm with the
middle sensors. Everything that should appear in the scan sits inside that box;
marks outside it (start outline, ruler, labels) are where the sensors don't look
while scanning a row.

Needs rsvg-convert (librsvg). Run from the repository root:
    python3 tools/make_test_sheet.py
"""
import json
import os
import shutil
import subprocess

PAGE_W, PAGE_H = 420.0, 297.0          # A3 landscape, mm
WORLD_LEFT, WORLD_BOTTOM = -125.0, -90.0  # world position of the page's bottom-left corner

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'DigitalTwin')


def P(x, y):
    """World mm -> SVG page mm (y down)."""
    return x - WORLD_LEFT, (WORLD_BOTTOM + PAGE_H) - y


def rect(x0, y0, x1, y1, fill='#000'):
    """Rectangle given by world corners."""
    ax, ay = P(min(x0, x1), max(y0, y1))
    return f'<rect x="{ax:.3f}" y="{ay:.3f}" width="{abs(x1 - x0):.3f}" height="{abs(y1 - y0):.3f}" fill="{fill}"/>'


def poly(points, fill='#000'):
    pts = ' '.join('%.3f,%.3f' % P(x, y) for x, y in points)
    return f'<polygon points="{pts}" fill="{fill}"/>'


def circle(x, y, r, fill='#000', stroke=None, sw=0.3, dash=None):
    cx, cy = P(x, y)
    s = f' stroke="{stroke}" stroke-width="{sw}"' if stroke else ''
    d = f' stroke-dasharray="{dash}"' if dash else ''
    return f'<circle cx="{cx:.3f}" cy="{cy:.3f}" r="{r:.3f}" fill="{fill}"{s}{d}/>'


def line(x0, y0, x1, y1, stroke='#888', sw=0.3):
    a, b = P(x0, y0)
    c, d = P(x1, y1)
    return f'<line x1="{a:.3f}" y1="{b:.3f}" x2="{c:.3f}" y2="{d:.3f}" stroke="{stroke}" stroke-width="{sw}"/>'


def text(x, y, s, size=3.2, anchor='start', fill='#555', weight='normal'):
    a, b = P(x, y)
    return (f'<text x="{a:.3f}" y="{b:.3f}" font-family="DejaVu Sans, Arial, sans-serif" '
            f'font-size="{size}" font-weight="{weight}" text-anchor="{anchor}" fill="{fill}">{s}</text>')


def build():
    import math
    el = []
    el.append(f'<rect x="0" y="0" width="{PAGE_W}" height="{PAGE_H}" fill="#fff"/>')

    # ---- Inside the scan area -------------------------------------------------
    # Four shapes, about 30 mm (top half).
    cy = 88.0
    el.append(rect(50, cy - 15, 80, cy + 15))                                   # square
    el.append(circle(103, cy, 15))                                              # circle
    el.append(poly([(126, cy - 13), (156, cy - 13), (141, cy + 13)]))           # triangle
    el.append(poly([(179 + 15 * math.cos(math.pi / 6 + k * math.pi / 3),
                     cy + 15 * math.sin(math.pi / 6 + k * math.pi / 3)) for k in range(6)]))  # hexagon

    # Grey steps 0, 25, 50, 75, 100 % black (sensor response / linearity).
    for k, level in enumerate([0, 25, 50, 75, 100]):
        g = round(255 * (1 - level / 100))
        x = 50 + k * 14
        el.append(rect(x, -2, x + 12, 12, fill='#%02x%02x%02x' % (g, g, g)))
    el.append(rect(50, -2, 118, 12, fill='none').replace('fill="none"', 'fill="none" stroke="#000" stroke-width="0.15"'))

    # Bars ACROSS the rows (perpendicular to travel): along-row resolution.
    # Two bars of width w with a gap w, for w = 1, 2, 3, 4, 6 mm.
    x = 50.0
    for w in [1, 2, 3, 4, 6]:
        el.append(rect(x, 20, x + w, 52))
        el.append(rect(x + 2 * w, 20, x + 3 * w, 52))
        x += 3 * w + 4

    # Bars ALONG the rows (parallel to travel): across-row resolution, which
    # depends on row pitch and pose error. w = 2, 3, 4, 6 mm.
    y = -3.0
    for w in [2, 3, 4, 6]:
        el.append(rect(128, y, 195, y + w))
        el.append(rect(128, y + 2 * w, 195, y + 3 * w))
        y += 3 * w + 4

    # ---- Outside the scan area ---------------------------------------------------
    grey = '#9aa3ad'
    # Corner ticks of the scanned area (the sensors never look here mid-row).
    for xx in (41, 201):
        el.append(line(xx, -34, xx, -40, grey, 0.4))
        el.append(line(xx, 146, xx, 152, grey, 0.4))
    el.append(text(121, -44, 'scan area: 160 mm rows, 17 rows at 7 mm (default settings)', 3.0, 'middle', grey))

    # Start position: robot outline, axle cross, wheels, heading arrow.
    el.append(circle(0, 0, 48.5, 'none', grey, 0.4, '2,2'))
    el.append(line(-6, 0, 6, 0, '#666', 0.4))
    el.append(line(0, -6, 0, 6, '#666', 0.4))
    for side in (1, -1):
        el.append(rect(-16, side * 41.25 - 3, 16, side * 41.25 + 3, fill='none')
                  .replace('fill="none"', f'fill="none" stroke="{grey}" stroke-width="0.4"'))
    el.append(poly([(-118, -58), (-96, -58), (-96, -55), (-88, -60), (-96, -65), (-96, -62), (-118, -62)], fill=grey))
    el.append(text(-84, -59, 'START: wheel-axle centre on the cross, facing right (arrow)', 3.2, 'start', '#555', 'bold'))
    el.append(text(-84, -64, "Next row is on the robot's LEFT, i.e. up this page.", 3.0, 'start', '#555'))

    # Labels for the test patches (below the scan area).
    el.append(text(84, -50, 'grey 0-100 %   |   bars across rows: 1 2 3 4 6 mm', 2.8, 'middle', grey))
    el.append(text(161.5, -50, 'bars along rows: 2 3 4 6 mm', 2.8, 'middle', grey))

    # Ruler (for the wheel-radius distance check): 0..280 mm from the start axle.
    ry = -78.0
    el.append(line(0, ry, 280, ry, '#444', 0.3))
    for mm in range(0, 281):
        h = 6 if mm % 10 == 0 else (4 if mm % 5 == 0 else 2.2)
        el.append(line(mm, ry, mm, ry + h, '#444', 0.18 if mm % 10 else 0.3))
        if mm % 50 == 0:
            el.append(text(mm, ry + 7.2, str(mm), 2.8, 'middle', '#444'))
    el.append(text(0, ry - 4.5, 'mm from the start axle position', 2.8, 'start', '#444'))

    # Title.
    el.append(text(-118, 200, 'ImageRaster test sheet', 5, 'start', '#333', 'bold'))
    el.append(text(-118, 194, 'A3 landscape. Print at 100 % (Actual size). Check: the ruler\'s 0-200 marks are 200 mm apart.', 3, 'start', '#555'))
    el.append(text(-118, 189, 'Digital Twin: surface.png is this sheet at 1 px/mm, centred at world (85, 58.5) mm.', 3, 'start', '#555'))

    body = '\n  '.join(el)
    return (f'<?xml version="1.0" encoding="UTF-8"?>\n'
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{PAGE_W}mm" height="{PAGE_H}mm" '
            f'viewBox="0 0 {PAGE_W} {PAGE_H}">\n  {body}\n</svg>\n')


def main():
    os.makedirs(OUT, exist_ok=True)
    svg = os.path.join(OUT, 'test_sheet.svg')
    with open(svg, 'w') as f:
        f.write(build())
    pdf = os.path.join(OUT, 'test_sheet_A3.pdf')
    png = os.path.join(OUT, 'surface.png')
    subprocess.run(['rsvg-convert', '-f', 'pdf', '-o', pdf, svg], check=True)
    # 25.4 dpi = 1 pixel per mm, exactly the Digital Twin's scale.
    subprocess.run(['rsvg-convert', '-f', 'png', '-d', '25.4', '-p', '25.4',
                    '-b', 'white', '-o', png, svg], check=True)
    # Where the sheet sits and the settings that cover it (read by the twin).
    info = {'sheet': 'test_sheet', 'page': 'A3 landscape',
            'offset_x_mm': WORLD_LEFT + PAGE_W / 2, 'offset_y_mm': WORLD_BOTTOM + PAGE_H / 2,
            'settings': {'pattern': 0, 'rows': 17, 'row_len': 160, 'pitch': 7.0, 'side': 1}}
    with open(os.path.join(OUT, 'surface.json'), 'w') as f:
        json.dump(info, f, indent=2)
    # Keep a copy with the other sheets, so make_sheet.py --use test_sheet switches back.
    sheets = os.path.join(OUT, 'sheets')
    os.makedirs(sheets, exist_ok=True)
    shutil.copyfile(png, os.path.join(sheets, 'test_sheet_surface.png'))
    shutil.copyfile(os.path.join(OUT, 'surface.json'), os.path.join(sheets, 'test_sheet.json'))
    # For the dashboard's "Compare with your template".
    import sys
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from make_sheet import write_template_png
    write_template_png(svg, os.path.join(sheets, 'test_sheet_template.png'),
                       {'sheet': 'test_sheet', 'x0_mm': WORLD_LEFT, 'y0_mm': WORLD_BOTTOM, 'w_mm': PAGE_W, 'h_mm': PAGE_H})
    print('wrote', svg, pdf, png, 'and surface.json')


if __name__ == '__main__':
    main()
