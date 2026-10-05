#!/usr/bin/env python3
"""
SuperRT data generator (replacement for the Windows-only C# testbed's
"PAL Regen" + "Write data" buttons).

Generates, from the scene command buffer:
  MainPal.bin       512 bytes  SNES CGRAM palette (256 x BGR555)
  PaletteMap.bin    32768 bytes  RGB555 -> palette index (same as the testbed)
  Placeholder.bin   16000 bytes  SNES 8bpp tiles of Placeholder.png  (200x80)
  Placeholder2.bin  32000 bytes  SNES 8bpp tiles of Placeholder2.png (200x160)
  SRTMcuData.bin    MCU descriptor + palette k-d tree (for the sd2snes MCU renderer)

The palette is built exactly like the testbed's PaletteGenerator (median-cut
style k-d tree split on the axis with the largest error), from 144 renders of
the scene (camera grid x,z in -10..10 step 4, four yaw angles), rendered with
the bit-accurate C port of the chip (batch_render).

usage: srt_datagen.py <batch_render binary> <CommandBuffer.bin> <Data dir> <out dir>
"""
import math
import os
import struct
import subprocess
import sys
import tempfile

import numpy as np
from PIL import Image

F = 1 << 14
W, H = 200, 160


# ---------------------------------------------------------------------------
# Camera set-up, following Form1.Trace() of the testbed
# ---------------------------------------------------------------------------

def fixed(v):
    return int(v * F)  # C# (int) cast truncates toward zero


def cs_fixedmul(x, y):
    # C# FixedMaths.FixedMul (sign-magnitude 40 bit truncation)
    p = x * y
    p = (p & 0x7FFFFFFFFF) if p >= 0 else -((-p) & 0x7FFFFFFFFF)
    return p >> 14


def norm_f(x, y, z):
    l = math.sqrt(x * x + y * y + z * z)
    return x / l, y / l, z / l


def testbed_params(camx, camy, camz, yaw):
    half = math.tan(45.0 * 3.14 / 180.0) * 0.5
    tl = [fixed(c) for c in norm_f(-half, -half, 1.0)]
    tr = [fixed(c) for c in norm_f(half, -half, 1.0)]
    bl = [fixed(c) for c in norm_f(-half, half, 1.0)]
    s = fixed(math.sin(yaw))
    c = fixed(math.cos(yaw))

    def rot(v):
        x, y, z = v
        return [cs_fixedmul(x, c) + cs_fixedmul(z, s), y, cs_fixedmul(z, c) - cs_fixedmul(x, s)]

    tl, tr, bl = rot(tl), rot(tr), rot(bl)

    def fdiv(a, b):  # C# (a << 14) / (b << 14) with truncation toward zero
        n, d = a << 14, b << 14
        q = abs(n) // abs(d)
        return q if (n >= 0) == (d >= 0) else -q

    xs = [fdiv(tr[i] - tl[i], W) for i in range(3)]
    ys = [fdiv(bl[i] - tl[i], H) for i in range(3)]
    light = [fixed(c) for c in norm_f(0.5, -1.0, -0.5)]

    def s16(v):
        v &= 0xFFFF
        return v - 0x10000 if v & 0x8000 else v

    return [fixed(camx), fixed(camy), fixed(camz)] + [s16(v) for v in tl] + \
        [s16(v) for v in xs] + [s16(v) for v in ys] + light


# ---------------------------------------------------------------------------
# PaletteGenerator.cs port
# ---------------------------------------------------------------------------

IDX = np.arange(0x8000)


def rgb555_split(c):
    r = (c & 0x1F) << 3
    g = ((c >> 5) & 0x1F) << 3
    b = ((c >> 10) & 0x1F) << 3
    r = np.where((r >> 3) & 1, r | 7, r)
    g = np.where((g >> 3) & 1, g | 7, g)
    b = np.where((b >> 3) & 1, b | 7, b)
    return r, g, b


R8, G8, B8 = rgb555_split(IDX)
CH = [R8, G8, B8]


def to555(r, g, b):
    return (r >> 3) | ((g >> 3) << 5) | ((b >> 3) << 10)


class Leaf:
    def __init__(self, counts):
        self.counts = counts
        self.parent = None
        self.update()

    def update(self):
        c = self.counts.astype(np.float64)
        tot = c.sum()
        self.npix = int(tot)
        lr = int(round((R8 * c).sum() / tot))
        lg = int(round((G8 * c).sum() / tot))
        lb = int(round((B8 * c).sum() / tot))
        self.colour = to555(lr, lg, lb)
        self.err = [(np.abs(R8 - lr) * c).sum(), (np.abs(G8 - lg) * c).sum(), (np.abs(B8 - lb) * c).sum()]

    def total_err(self):
        return sum(self.err)


class Split:
    def __init__(self, axis, point, left, right):
        self.axis, self.point, self.left, self.right = axis, point, left, right
        self.parent = None


def split_leaf(leaf):
    used = leaf.counts > 0
    if used.sum() < 2:
        return leaf
    mins = [int(ch[used].min()) for ch in CH]
    maxs = [int(ch[used].max()) for ch in CH]
    if mins == maxs:
        return leaf
    er, eg, eb = leaf.err
    if eg > er and eg > eb:
        axis = 1
    elif eb > er:
        axis = 2
    else:
        axis = 0
    point = (maxs[axis] + mins[axis]) // 2
    is_left = CH[axis] <= point
    lc = np.where(is_left, leaf.counts, 0)
    rc = np.where(is_left, 0, leaf.counts)
    if not lc.any() or not rc.any():
        return leaf
    node = Split(axis, point, Leaf(lc), Leaf(rc))
    node.parent = leaf.parent
    node.left.parent = node
    node.right.parent = node
    return node


def leaves_of(node, out):
    if isinstance(node, Leaf):
        out.append(node)
    else:
        leaves_of(node.left, out)
        leaves_of(node.right, out)


def generate_palette(counts, fixed_colours):
    entries = list(fixed_colours)
    root = Leaf(counts)
    leaves = [root]
    for _ in range((256 - len(entries)) - 1):
        leaves.sort(key=lambda l: -l.total_err())
        tgt = leaves[0]
        new = split_leaf(tgt)
        if new is tgt:
            break
        leaves.remove(tgt)
        leaves_of(new, leaves)
        if tgt.parent is not None:
            if tgt.parent.left is tgt:
                tgt.parent.left = new
            else:
                tgt.parent.right = new
        else:
            root = new
    leaves.sort(key=lambda l: -l.colour)
    for l in leaves:
        l.index = len(entries)
        entries.append(l.colour)
    # palette map
    pmap = np.zeros(0x8000, dtype=np.uint8)

    def fill(node, mask):
        if isinstance(node, Leaf):
            pmap[mask] = node.index
            return
        left = CH[node.axis] <= node.point
        fill(node.left, mask & left)
        fill(node.right, mask & ~left)

    fill(root, np.ones(0x8000, dtype=bool))
    return entries, pmap, root


def snes_palette(entries):
    out = bytearray(512)
    for i in range(256):
        if i < len(entries):
            r, g, b = (int(v) for v in rgb555_split(np.array(entries[i])))
        else:
            r = g = b = 0
        out[i * 2] = ((r >> 3) | (((g >> 3) & 7) << 5)) & 0xFF
        out[i * 2 + 1] = ((g >> 6) | ((b >> 3) << 2)) & 0xFF
    return bytes(out)


# ---------------------------------------------------------------------------
# Tree export for the MCU
# ---------------------------------------------------------------------------

def export_tree(root):
    """4 byte nodes: flags|axis, split, left, right.
    flags bit 6: left child is a leaf (palette index), bit 7: right child is a leaf."""
    nodes = []

    def walk(node):
        idx = len(nodes)
        nodes.append(None)
        lflag = isinstance(node.left, Leaf)
        rflag = isinstance(node.right, Leaf)
        left = node.left.index if lflag else walk(node.left)
        right = node.right.index if rflag else walk(node.right)
        nodes[idx] = bytes([node.axis | (0x40 if lflag else 0) | (0x80 if rflag else 0), node.point, left, right])
        return idx

    if isinstance(root, Leaf):
        return b'', root.index
    walk(root)
    assert len(nodes) <= 255
    return b''.join(nodes), 0


def tree_lookup(tree, root_leaf, c):
    if not tree:
        return root_leaf
    r, g, b = (int(v) for v in rgb555_split(np.array(c)))
    ch = (r, g, b)
    n = 0
    while True:
        f, p, l, rr = tree[n * 4:n * 4 + 4]
        if ch[f & 3] <= p:
            if f & 0x40:
                return l
            n = l
        else:
            if f & 0x80:
                return rr
            n = rr


# ---------------------------------------------------------------------------
# SNES tile conversion (Form1.ConvertToSNESFormat)
# ---------------------------------------------------------------------------

def to_snes_tiles(indices, width, height):
    tiles_x = width // 8
    out = bytearray(width * height)
    for ty in range(height // 8):
        for tx in range(tiles_x):
            tile = ty * tiles_x + tx
            for y in range(8):
                px = indices[(ty * 8 + y) * width + tx * 8:(ty * 8 + y) * width + tx * 8 + 8]
                for plane in range(8):
                    v = 0
                    for x in range(8):
                        v |= ((int(px[x]) >> plane) & 1) << (7 - x)
                    out[tile * 64 + (plane >> 1) * 16 + y * 2 + (plane & 1)] = v
    return bytes(out)


def image_to_tiles(path, pmap):
    im = Image.open(path).convert('RGB')
    w, h = im.size
    a = np.asarray(im).astype(np.int32)
    c555 = (a[:, :, 0] >> 3) | ((a[:, :, 1] >> 3) << 5) | ((a[:, :, 2] >> 3) << 10)
    return to_snes_tiles(pmap[c555.reshape(-1)], w, h)


# ---------------------------------------------------------------------------

MCU_MAGIC = b'SRTMCU01'


def main():
    if len(sys.argv) != 5:
        print(__doc__)
        sys.exit(1)
    renderer, cmdbuf, datadir, outdir = sys.argv[1:]
    os.makedirs(outdir, exist_ok=True)

    views = []
    for x in range(-10, 11, 4):
        for z in range(-10, 11, 4):
            for yaw in (0, 90, 180, 270):
                views.append(testbed_params(float(x), 0.0, float(z), fixed(yaw * math.pi / 180.0) / F))
    with tempfile.TemporaryDirectory() as td:
        pl = os.path.join(td, 'views.txt')
        fb = os.path.join(td, 'frames.bin')
        with open(pl, 'w') as f:
            for v in views:
                f.write(' '.join(str(i) for i in v) + '\n')
        subprocess.check_call([renderer, cmdbuf, pl, fb])
        frames = np.fromfile(fb, dtype='<u2')
    print(f'rendered {len(views)} views')
    counts = np.bincount(frames & 0x7FFF, minlength=0x8000).astype(np.int64)

    black, white = to555(0, 0, 0), to555(255, 255, 255)
    entries, pmap, root = generate_palette(counts, [black, white])
    print(f'palette: {len(entries)} entries')

    tree, root_leaf = export_tree(root)
    # verify the tree reproduces the flat map
    for c in range(0, 0x8000, 7):
        assert tree_lookup(tree, root_leaf, c) == pmap[c]

    open(os.path.join(outdir, 'MainPal.bin'), 'wb').write(snes_palette(entries))
    open(os.path.join(outdir, 'PaletteMap.bin'), 'wb').write(pmap.tobytes())
    open(os.path.join(outdir, 'Placeholder.bin'), 'wb').write(image_to_tiles(os.path.join(datadir, 'Placeholder.png'), pmap))
    open(os.path.join(outdir, 'Placeholder2.bin'), 'wb').write(image_to_tiles(os.path.join(datadir, 'Placeholder2.png'), pmap))

    # MCU descriptor: header (64 bytes) + tree. Lives at ROM file offset 0x8000.
    hdr = bytearray(64)
    hdr[0:8] = MCU_MAGIC
    struct.pack_into('<HBB', hdr, 8, len(tree) // 4, root_leaf, 0)
    struct.pack_into('<I', hdr, 12, 64)            # tree offset, relative to descriptor
    struct.pack_into('<I', hdr, 16, 0x10000)       # file offset of the 32000 byte start-up image
    open(os.path.join(outdir, 'SRTMcuData.bin'), 'wb').write(bytes(hdr) + tree)
    print(f'tree: {len(tree) // 4} nodes')

    # preview of one view through the palette
    f0 = frames[:W * H]
    pal = np.array([[int(v) for v in rgb555_split(np.array(e))] for e in entries + [0] * (256 - len(entries))], dtype=np.uint8)
    Image.fromarray(pal[pmap[f0 & 0x7FFF]].reshape(H, W, 3)).resize((W * 3, H * 3), Image.NEAREST).save(os.path.join(outdir, 'preview_view0.png'))


if __name__ == '__main__':
    main()
