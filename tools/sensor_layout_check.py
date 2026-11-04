# Run from this folder: python3 sensor_layout_check.py  (needs numpy and pillow)
"""Check: does a wrong sensor-layout model explain the smeared scan image?

World frame: start axle at (0,0), robot faces +x, rows step +y (robot's left).
Shapes laid out like the user's sheet (triangle/hexagon low, square/circle high).
"""
import math
import numpy as np
from PIL import Image, ImageDraw

rng = np.random.default_rng(3)

# ---- target sheet, 1 px = 0.5 mm, origin offset so negative coords fit
S = 2.0  # px per mm
X0, Y0 = -60.0, -80.0
W, H = int(380 * S), int(360 * S)
sheet = Image.new("L", (W, H), 0)
d = ImageDraw.Draw(sheet)

def P(x, y):  # world mm -> sheet px (y up)
    return ((x - X0) * S, H - (y - Y0) * S)

# triangle (lower-left), hexagon (lower-right), square (upper-left), circle (upper-right)
d.polygon([P(75, 40), P(122, 40), P(98.5, 84)], fill=255)
hx, hy, hr = 158, 63, 24
d.polygon([P(hx + hr * math.cos(math.radians(30 + 60 * k)), hy + hr * math.sin(math.radians(30 + 60 * k))) for k in range(6)], fill=255)
d.rectangle([P(76, 147), P(121, 104)], fill=255)
cx, cy, cr = 159, 125, 21
x0, y0 = P(cx - cr, cy + cr); x1, y1 = P(cx + cr, cy - cr)
d.ellipse([x0, y0, x1, y1], fill=255)
T = np.asarray(sheet, dtype=np.float32) / 255.0

def dark(x, y, r=2.0):
    """Average darkness over a small disc (the sensor's spot)."""
    acc, n = 0.0, 0
    for dx in (-r, 0, r):
        for dy in (-r, 0, r):
            i = int((x + dx - X0) * S); j = int(H - (y + dy - Y0) * S)
            if 0 <= i < W and 0 <= j < H:
                acc += T[j, i]
            n += 1
    return acc / n

# True 3Pi+ layout measured from the underside photo: (ahead, left) mm
TRUE = [(28, 27), (40, 9), (41, 0), (40, -9), (28, -27)]
GAIN = [1300, 1450, 1500, 1420, 1250]
OFF = [640, 600, 580, 610, 660]

def scan(rows, row_len, pitch, lead=40.0):
    samples = []
    for k in range(rows):
        y = k * pitch
        if k % 2 == 0:
            xs = np.arange(0, row_len + 1e-6, 1.1); th = 0.0
        else:
            start = row_len + 2 * lead
            xs = start - np.arange(0, row_len + 1e-6, 1.1); th = math.pi
        for x in xs:
            c, s = math.cos(th), math.sin(th)
            dn = []
            for i, (f, l) in enumerate(TRUE):
                X = x + f * c - l * s; Y = y + f * s + l * c
                dn.append(OFF[i] + GAIN[i] * dark(X, Y) + rng.normal(0, 25))
            samples.append((x, y, th, dn))
    return samples

def rebuild(samples, model, cell=1.0, foot=10.0):
    dn = np.array([s[3] for s in samples])
    lo = np.percentile(dn, 2, axis=0); hi = np.percentile(dn, 98, axis=0)
    span = hi - lo
    good = span >= 0.35 * span.max()
    span = np.where(good, span, np.median(span[good]))
    pts = []
    for (x, y, th, r) in samples:
        c, s = math.cos(th), math.sin(th)
        for i, (f, l) in enumerate(model):
            pts.append((x + f * c - l * s, y + f * s + l * c, np.clip((r[i] - lo[i]) / span[i], 0, 1)))
    pts = np.array(pts)
    gx0, gy0, gx1, gy1 = -20, -70, 280, 290
    Wc, Hc = int((gx1 - gx0) / cell), int((gy1 - gy0) / cell)
    acc = np.zeros((Hc, Wc)); wsum = np.zeros((Hc, Wc))
    sig = foot / 2.355; rad = int(math.ceil(foot / cell))
    for X, Y, v in pts:
        fi = (X - gx0) / cell - 0.5; fj = (Y - gy0) / cell - 0.5
        ci, cj = int(round(fi)), int(round(fj))
        for j in range(max(0, cj - rad), min(Hc, cj + rad + 1)):
            for i in range(max(0, ci - rad), min(Wc, ci + rad + 1)):
                d2 = ((i - fi) ** 2 + (j - fj) ** 2) * cell * cell
                if d2 > foot * foot: continue
                w = math.exp(-d2 / (2 * sig * sig))
                acc[j, i] += w * v; wsum[j, i] += w
    img = np.full((Hc, Wc, 3), 40, np.uint8)
    ok = wsum >= 0.2
    g = (255 * (1 - acc[ok] / wsum[ok])).astype(np.uint8)
    img[ok] = np.stack([g, g, g], -1)
    return Image.fromarray(img[::-1])

old = scan(6, 180, 40)
straight_23 = [(40, 46), (40, 23), (40, 0), (40, -23), (40, -46)]
a = rebuild(old, straight_23, cell=2, foot=10)
b = rebuild(old, TRUE, cell=2, foot=10)
new = scan(10, 160, 14)
c = rebuild(new, TRUE, cell=2, foot=7)
truth = sheet.crop((int((-20 - X0) * S), int(H - (290 - Y0) * S), int((280 - X0) * S), int(H - (-70 - Y0) * S))).resize(a.size)
truth = Image.eval(truth, lambda v: 255 - v).convert("RGB")
out = Image.new("RGB", (a.width * 4 + 30, a.height + 34), (255, 255, 255))
labels = ["Sheet", "Your scan, bar model 23 mm", "Your scan, 3Pi+ layout", "10 rows x 14 mm, 3Pi+ layout"]
dd = ImageDraw.Draw(out)
for k, im in enumerate([truth, a, b, c]):
    out.paste(im, (k * (a.width + 10), 34))
    dd.text((k * (a.width + 10) + 4, 10), labels[k], fill=(0, 0, 0))
out = out.resize((out.width * 2, out.height * 2), Image.NEAREST)
out.save("geom_check.png")
print("samples old", len(old), "new", len(new))
