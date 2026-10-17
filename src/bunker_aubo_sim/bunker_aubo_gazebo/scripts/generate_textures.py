#!/usr/bin/env python3
import os
import sys

import numpy as np
from PIL import Image, ImageDraw

SIZE = 256


def palette(rng, n):
    return [tuple(int(c) for c in rng.integers(20, 235, 3)) for _ in range(n)]


def blocks(rng):
    cells = rng.choice([8, 12, 16])
    grid = rng.random((cells, cells)) > 0.5
    col = palette(rng, 2)
    img = np.zeros((cells, cells, 3), np.uint8)
    img[grid] = col[0]
    img[~grid] = (240, 240, 240)
    return Image.fromarray(img).resize((SIZE, SIZE), Image.NEAREST)


def shapes(rng):
    img = Image.new('RGB', (SIZE, SIZE), tuple(int(v) for v in rng.integers(180, 250, 3)))
    d = ImageDraw.Draw(img)
    for c in palette(rng, 14):
        x0, y0 = rng.integers(0, SIZE - 40, 2)
        w, h = rng.integers(20, 110, 2)
        kind = rng.integers(0, 3)
        box = [int(x0), int(y0), int(x0 + w), int(y0 + h)]
        if kind == 0:
            d.rectangle(box, fill=c, outline=(0, 0, 0), width=2)
        elif kind == 1:
            d.ellipse(box, fill=c, outline=(0, 0, 0), width=2)
        else:
            d.polygon([(box[0], box[3]), ((box[0] + box[2]) // 2, box[1]), (box[2], box[3])],
                      fill=c, outline=(0, 0, 0))
    return img


def stripes(rng):
    img = Image.new('RGB', (SIZE, SIZE), (245, 245, 245))
    d = ImageDraw.Draw(img)
    cols = palette(rng, 5)
    step = int(rng.integers(14, 30))
    for i, x in enumerate(range(-SIZE, SIZE * 2, step)):
        d.polygon([(x, 0), (x + step // 2, 0), (x + step // 2 + SIZE, SIZE), (x + SIZE, SIZE)],
                  fill=cols[i % len(cols)])
    for c in palette(rng, 4):
        x, y = rng.integers(10, SIZE - 60, 2)
        d.rectangle([int(x), int(y), int(x) + 45, int(y) + 45], fill=c, outline=(0, 0, 0), width=3)
    return img


def text_lines(rng):
    img = Image.new('RGB', (SIZE, SIZE), (250, 250, 245))
    d = ImageDraw.Draw(img)
    title = palette(rng, 1)[0]
    d.rectangle([0, 0, SIZE, 50], fill=title)
    y = 65
    while y < SIZE - 12:
        x = 12
        while x < SIZE - 30:
            w = int(rng.integers(12, 45))
            d.rectangle([x, y, min(x + w, SIZE - 12), y + 8], fill=(30, 30, 30))
            x += w + int(rng.integers(6, 12))
        y += 18
    d.ellipse([SIZE - 60, 5, SIZE - 15, 45], fill=(255, 255, 255))
    return img


def checker(rng):
    n = int(rng.choice([6, 8, 10]))
    a, b = palette(rng, 2)
    img = np.zeros((n, n, 3), np.uint8)
    for i in range(n):
        for j in range(n):
            img[i, j] = a if (i + j) % 2 else b
    im = Image.fromarray(img).resize((SIZE, SIZE), Image.NEAREST)
    d = ImageDraw.Draw(im)
    x, y = rng.integers(40, SIZE - 80, 2)
    d.ellipse([int(x), int(y), int(x) + 60, int(y) + 60], fill=(250, 250, 250), outline=(0, 0, 0), width=3)
    return im


def rug(rng):
    img = Image.new('RGB', (SIZE, SIZE), tuple(int(v) for v in rng.integers(60, 140, 3)))
    d = ImageDraw.Draw(img)
    for c in palette(rng, 30):
        x, y = rng.integers(0, SIZE, 2)
        r = int(rng.integers(4, 18))
        d.ellipse([int(x) - r, int(y) - r, int(x) + r, int(y) + r], fill=c)
    d.rectangle([4, 4, SIZE - 5, SIZE - 5], outline=(230, 230, 230), width=6)
    return img


GENERATORS = [blocks, shapes, stripes, text_lines, checker]


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'textures'
    os.makedirs(out, exist_ok=True)
    rng = np.random.default_rng(20260923)
    for i in range(1, 21):
        gen = GENERATORS[(i - 1) % len(GENERATORS)]
        gen(rng).save(os.path.join(out, f'poster_{i:02d}.png'), optimize=True)
    for i in range(1, 5):
        rug(rng).save(os.path.join(out, f'rug_{i:02d}.png'), optimize=True)
    print(f'wrote textures to {out}')


if __name__ == '__main__':
    main()
