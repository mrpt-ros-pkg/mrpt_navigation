#!/usr/bin/env python3
# Plots the traces saved by test_navigation_battery.py (MRPT_NAV_TEST_LOG):
# for each goal, map obstacles, planned and replanned reference paths, ground
# truth and localized trajectories, and footprints along the ground truth.
#
# Usage: plot_nav_traces.py <map.yaml> <out_dir> <trace.json>...
# e.g. map: mrpt_tutorials/maps/demo_world2.yaml

import json
import math
import os
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402


def load_map(yaml_file):
    cfg = {}
    for line in open(yaml_file):
        if ':' in line:
            k, v = line.split(':', 1)
            cfg[k.strip()] = v.strip()
    img = np.array(Image.open(os.path.join(os.path.dirname(yaml_file), cfg['image'])).convert('L'))
    res = float(cfg['resolution'])
    ox, oy = [float(x) for x in cfg['origin'].strip('[]').split(',')[:2]]
    occ = (255 - img) / 255.0 > float(cfg['occupied_thresh'])
    h, w = img.shape
    return occ, [ox, ox + w * res, oy, oy + h * res]


def footprint(shape, x, y, phi):
    c, s = math.cos(phi), math.sin(phi)
    pts = [(x + c * px - s * py, y + s * px + c * py) for px, py in shape]
    pts.append(pts[0])
    return zip(*pts)


def arrow(ax, x, y, phi, color, label=None):
    ax.annotate('', xy=(x + 1.2 * math.cos(phi), y + 1.2 * math.sin(phi)), xytext=(x, y),
                arrowprops=dict(arrowstyle='->', color=color, lw=2))
    ax.plot(x, y, 'o', color=color, ms=6, label=label)


def plot_goal(ax, occ, extent, tr, shape, title):
    gt = np.array(tr['gt'])
    loc = np.array(tr['loc']) if tr['loc'] else None
    pts = [np.array(p) for _, p in tr['ref_paths'] if p]
    allx = [gt[:, 1]] + [p[:, 0] for p in pts] + [[tr['goal'][0]]]
    ally = [gt[:, 2]] + [p[:, 1] for p in pts] + [[tr['goal'][1]]]
    x0, x1 = min(map(np.min, allx)) - 3, max(map(np.max, allx)) + 3
    y0, y1 = min(map(np.min, ally)) - 3, max(map(np.max, ally)) + 3
    ax.imshow(occ, extent=extent, cmap='Greys', vmin=0, vmax=1.4, interpolation='nearest')
    colors = plt.cm.autumn(np.linspace(0, 0.8, max(1, len(pts))))
    for k, p in enumerate(pts):
        ax.plot(p[:, 0], p[:, 1], '--' if k else '-', color='tab:blue' if k == 0 else colors[k],
                lw=2, label='planned path' if k == 0 else f'replan #{k}')
    if shape:
        for k in range(0, len(gt), 15):
            fx, fy = footprint(shape, gt[k, 1], gt[k, 2], gt[k, 3])
            ax.plot(list(fx), list(fy), color='tab:green', lw=0.6, alpha=0.6)
    ax.plot(gt[:, 1], gt[:, 2], color='tab:green', lw=1.8, label='executed (ground truth)')
    if loc is not None and len(loc):
        ax.plot(loc[:, 1], loc[:, 2], ':', color='tab:purple', lw=1.5, label='localization')
    arrow(ax, gt[0, 1], gt[0, 2], gt[0, 3], 'k', 'start')
    g = tr['goal']
    arrow(ax, g[0], g[1], math.radians(g[2]), 'tab:red', 'goal')
    r = tr['result']
    ax.set_title(f"{title}\n{r['final_status']}: pos err {r['pos_err']:.2f} m, "
                 f"yaw err {r['yaw_err']:.0f} deg, max dev {r['max_dev']:.2f} m, "
                 f"max loc err {r.get('max_loc_err', 0):.2f} m, "
                 f"collisions {r.get('collisions', '?')}", fontsize=9)
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_aspect('equal')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=7, loc='best')


def main():
    occ, extent = load_map(sys.argv[1])
    out = sys.argv[2]
    os.makedirs(out, exist_ok=True)
    for fn in sys.argv[3:]:
        d = json.load(open(fn))
        name = os.path.splitext(os.path.basename(fn))[0]
        n = len(d['goals'])
        fig, axs = plt.subplots(2, 3, figsize=(18, 11))
        for i, tr in enumerate(d['goals']):
            plot_goal(axs.flat[i], occ, extent, tr, d['robot_shape'], f'{name} goal #{i}')
        for ax in axs.flat[n:]:
            ax.axis('off')
        fig.tight_layout()
        fig.savefig(os.path.join(out, f'{name}.png'), dpi=110)
        plt.close(fig)
        print('saved', os.path.join(out, f'{name}.png'))


if __name__ == '__main__':
    main()
