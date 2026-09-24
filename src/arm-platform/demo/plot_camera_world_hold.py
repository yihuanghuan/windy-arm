#!/usr/bin/env python3
"""Plot actual recorded HOLD samples; never interpolate across disable/re-enable."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('csv')
    parser.add_argument('--output', default='')
    args = parser.parse_args()
    with open(args.csv) as stream:
        rows = list(csv.DictReader(stream))
    if not rows or not any(r['state'] == 'HOLD' for r in rows):
        parser.error('no HOLD samples in this log; no performance plot can be made')
    t = np.array([float(r['time']) for r in rows])
    t -= t[0]
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    for ax, names, unit in zip(axes, [('ep_x','ep_y','ep_z'),
                                     ('rpy_error_roll','rpy_error_pitch','rpy_error_yaw')],
                               ['position error (m)', 'orientation error (deg)']):
        for name in names:
            y = np.array([float(r[name]) if r['state'] == 'HOLD' else np.nan for r in rows])
            if 'deg' in unit:
                y = np.rad2deg(y)
            ax.plot(t, y, label=name)
        ax.set_ylabel(unit)
        ax.grid(True)
        ax.legend()
    axes[-1].set_xlabel('time (s)')
    fig.suptitle(Path(args.csv).name + '\nRPY of R_des R_cur^T (controller uses SO(3) log)')
    fig.tight_layout()
    output = args.output or str(Path(args.csv).with_suffix('.png'))
    fig.savefig(output, dpi=160)
    print(output)


if __name__ == '__main__':
    main()
