#!/usr/bin/env python3
"""Render frozen sampled trajectories against the unchanged raster guard."""
import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--offline', type=Path, required=True)
    args = p.parse_args()
    audit = json.loads((args.offline / 'audit.json').read_text())
    grid = np.load(args.offline / 'guard_grid.npz')
    width, height = int(grid['width']), int(grid['height'])
    mask = np.zeros((height, width))
    mask.flat[grid['free']] = 1
    res = float(grid['resolution'])
    ox, oy = grid['origin']
    entries = audit['entries'][:5]
    fig, axes = plt.subplots(2, 3, figsize=(12, 7), layout='constrained')
    for ax, event in zip(axes.flat, entries):
        e = event['entry']
        rows = event['preceding_1s'] + [e]
        pos = np.asarray([r['position'] for r in rows])
        ax.imshow(mask, cmap='Greys_r', vmin=0, vmax=1, origin='upper',
                  extent=[ox, ox+width*res, oy, oy+height*res], alpha=.3)
        ax.plot(pos[:, 0], pos[:, 1], '-o', ms=2, color='black', label='sampled actual root')
        extra = [r for r in rows if r['safe'] and r['heading_risk'] and not r['actual_risk']]
        if extra:
            r = extra[-1]
            for key, color, label in [('actual_end', '#1976D2', 'instantaneous prediction'),
                                      ('heading_end', '#E65100', 'heading prediction')]:
                ax.plot([r['position'][0], r[key][0]], [r['position'][1], r[key][1]],
                        '--', color=color, lw=2, label=label)
        ax.scatter(*e['position'], marker='x', color='red', s=60, label='first unsafe sample')
        ax.set_xlim(e['position'][0]-.8, e['position'][0]+.8)
        ax.set_ylim(e['position'][1]-.8, e['position'][1]+.8)
        ax.set_aspect('equal')
        ax.set_title(f"{e['person'].split('/')[3]} at t={e['t']:.3f} s")
        ax.set_xlabel('x (m)')
        ax.set_ylabel('y (m)')
    axes.flat[-1].axis('off')
    handles, labels = axes.flat[0].get_legend_handles_labels()
    axes.flat[-1].legend(handles, labels, loc='center')
    fig.suptitle('Retained baseline: sampled crossing histories, original 0.20 m guard\n'
                 'Predictions use unchanged 0.70 s response and 4 m/s² acceleration; no counterfactual safety claim')
    fig.savefig(args.offline / 'first_five_entries.png', dpi=160)
    plt.close(fig)


if __name__ == '__main__':
    main()
