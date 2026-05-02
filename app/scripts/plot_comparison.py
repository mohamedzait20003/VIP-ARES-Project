#!/usr/bin/env python3
"""
NMPC vs NRHDG Performance Comparison
======================================
Usage:
  python3 plot_comparison.py [nmpc_csv] [nrhdg_csv] [output_png]

Defaults:
  nmpc_csv   = ~/ares_metrics/nmpc_metrics.csv
  nrhdg_csv  = ~/ares_metrics/nrhdg_metrics.csv
  output_png = comparison.png

CSV columns (produced by MetricsLoggerNode):
  t, ex, ey, ez, pos_err, vel_err, ang_err, solve_ms, thrust, torque, ok
"""

import os
import sys

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec


def load(path: str) -> pd.DataFrame:
    path = os.path.expanduser(path)
    if not os.path.exists(path):
        raise FileNotFoundError(f'Metrics file not found: {path}')
    df = pd.read_csv(path)
    df['t'] -= df['t'].iloc[0]          # normalise time to start at 0
    return df


def print_summary(name: str, df: pd.DataFrame) -> None:
    ok      = df['ok'] == 1.0
    rmse    = float(np.sqrt((df.loc[ok, 'pos_err'] ** 2).mean()))
    max_err = float(df.loc[ok, 'pos_err'].max())
    mean_ms = float(df.loc[ok, 'solve_ms'].mean())
    max_ms  = float(df.loc[ok, 'solve_ms'].max())
    p95_ms  = float(df.loc[ok, 'solve_ms'].quantile(0.95))
    sr      = ok.mean() * 100.0
    print(f'  {name}:')
    print(f'    Position RMSE      : {rmse:.4f} m')
    print(f'    Max position error : {max_err:.4f} m')
    print(f'    Solve time (mean)  : {mean_ms:.2f} ms')
    print(f'    Solve time (p95)   : {p95_ms:.2f} ms')
    print(f'    Solve time (max)   : {max_ms:.2f} ms')
    print(f'    Solver success rate: {sr:.1f}%')


def plot(nmpc: pd.DataFrame, nrhdg: pd.DataFrame, out: str) -> None:
    fig = plt.figure(figsize=(16, 11))
    fig.suptitle('NMPC vs NRHDG — Simulation Performance Comparison', fontsize=14, y=0.98)
    gs = gridspec.GridSpec(3, 2, hspace=0.45, wspace=0.32)

    blue   = '#2563EB'
    orange = '#EA580C'

    # --- 1. Position error norm ---
    ax = fig.add_subplot(gs[0, 0])
    ax.plot(nmpc['t'],  nmpc['pos_err'],  color=blue,   label='NMPC',  lw=1.2)
    ax.plot(nrhdg['t'], nrhdg['pos_err'], color=orange, label='NRHDG', lw=1.2)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (m)')
    ax.set_title('3-D Position Tracking Error')
    ax.legend()
    ax.grid(True, alpha=0.35)

    # --- 2. Per-axis position error ---
    ax = fig.add_subplot(gs[0, 1])
    for col, ls in zip(['ex', 'ey', 'ez'], ['-', '--', ':']):
        ax.plot(nmpc['t'],  nmpc[col].abs(),  color=blue,   ls=ls, lw=1.0, label=f'NMPC  {col}')
        ax.plot(nrhdg['t'], nrhdg[col].abs(), color=orange, ls=ls, lw=1.0, label=f'NRHDG {col}')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('|Error| (m)')
    ax.set_title('Per-Axis Position Error')
    ax.legend(fontsize=7, ncol=2)
    ax.grid(True, alpha=0.35)

    # --- 3. Solve time over time ---
    ax = fig.add_subplot(gs[1, 0])
    ax.plot(nmpc['t'],  nmpc['solve_ms'],  color=blue,   label='NMPC',  lw=1.0, alpha=0.8)
    ax.plot(nrhdg['t'], nrhdg['solve_ms'], color=orange, label='NRHDG', lw=1.0, alpha=0.8)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Solve Time (ms)')
    ax.set_title('Solver Computation Time per Step')
    ax.legend()
    ax.grid(True, alpha=0.35)

    # --- 4. Solve time histogram ---
    ax = fig.add_subplot(gs[1, 1])
    bins = np.linspace(
        0,
        max(nmpc['solve_ms'].max(), nrhdg['solve_ms'].max()) * 1.05,
        40,
    )
    ax.hist(nmpc['solve_ms'],  bins=bins, color=blue,   alpha=0.6, label='NMPC',  density=True)
    ax.hist(nrhdg['solve_ms'], bins=bins, color=orange, alpha=0.6, label='NRHDG', density=True)
    ax.axvline(nmpc['solve_ms'].mean(),  color=blue,   ls='--', lw=1.5,
               label=f'NMPC mean {nmpc["solve_ms"].mean():.1f} ms')
    ax.axvline(nrhdg['solve_ms'].mean(), color=orange, ls='--', lw=1.5,
               label=f'NRHDG mean {nrhdg["solve_ms"].mean():.1f} ms')
    ax.set_xlabel('Solve Time (ms)')
    ax.set_ylabel('Density')
    ax.set_title('Solve Time Distribution')
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.35)

    # --- 5. Thrust ---
    ax = fig.add_subplot(gs[2, 0])
    ax.plot(nmpc['t'],  nmpc['thrust'],  color=blue,   label='NMPC',  lw=1.0)
    ax.plot(nrhdg['t'], nrhdg['thrust'], color=orange, label='NRHDG', lw=1.0)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Normalised Thrust')
    ax.set_title('Thrust Command')
    ax.set_ylim(0, 1.05)
    ax.legend()
    ax.grid(True, alpha=0.35)

    # --- 6. Torque norm ---
    ax = fig.add_subplot(gs[2, 1])
    ax.plot(nmpc['t'],  nmpc['torque'],  color=blue,   label='NMPC',  lw=1.0)
    ax.plot(nrhdg['t'], nrhdg['torque'], color=orange, label='NRHDG', lw=1.0)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('||τ|| (N·m)')
    ax.set_title('Torque Command Norm')
    ax.legend()
    ax.grid(True, alpha=0.35)

    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'Saved → {out}')
    plt.show()


def main() -> None:
    nmpc_path  = sys.argv[1] if len(sys.argv) > 1 else '~/ares_metrics/nmpc_metrics.csv'
    nrhdg_path = sys.argv[2] if len(sys.argv) > 2 else '~/ares_metrics/nrhdg_metrics.csv'
    out_path   = sys.argv[3] if len(sys.argv) > 3 else 'comparison.png'

    nmpc  = load(nmpc_path)
    nrhdg = load(nrhdg_path)

    print('\n=== Summary Statistics ===')
    print_summary('NMPC',  nmpc)
    print_summary('NRHDG', nrhdg)
    print()

    plot(nmpc, nrhdg, out_path)


if __name__ == '__main__':
    main()
