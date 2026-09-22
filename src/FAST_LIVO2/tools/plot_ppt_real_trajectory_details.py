#!/usr/bin/env python3
"""Redraw archived experiments as equal-scale PPT trajectory details, without AI."""

import argparse
import json
import math
from pathlib import Path
import tempfile

import numpy as np
import plot_rtk_slam_ppt_ablation as original
import matplotlib.pyplot as plt


ARCHIVE = Path('/home/gulu/TiaoZhanBei/eval_20260830/results/rtk_slam_ppt_real_20260905/evidence')
OUTPUT = Path(__file__).resolve().parents[1] / 'artifacts/ppt_real_20260905'
BLUE, RED = '#1577B8', '#D75452'


def read_first_emission(path):
    data = np.loadtxt(str(path), comments='#', ndmin=2)
    if data.shape[1] != 8 or not np.isfinite(data).all():
        raise ValueError('Invalid TUM: {}'.format(path))
    if np.any(np.diff(data[:, 0]) < 0):
        raise ValueError('Nonmonotonic TUM: {}'.format(path))
    # Keep the first emitted estimate at a repeated time, before later revisions.
    return data[np.r_[True, np.diff(data[:, 0]) > 0]]


def rotation(q):
    return original.quaternion_rotate(np.tile(q, (3, 1)), np.eye(3)).T


def initial_frame(raw, online, lever):
    matches = np.flatnonzero(raw[:, 0] == online[0, 0])
    if len(matches) != 1:
        raise ValueError('Initial prior must match an exact raw timestamp')
    i = matches[0]
    r = rotation(online[0, 4:8]) @ rotation(raw[i, 4:8]).T
    base = raw[:, 1:4] + original.quaternion_rotate(raw[:, 4:8], lever)
    t = online[0, 1:4] - r @ base[i]
    result = raw.copy()
    result[:, 1:4] = base @ r.T + t
    return result, r, t


def split_at_gaps(t, xy, max_gap=1.5):
    """Insert a pen lift, preserving every measured vertex on both sides."""
    return np.insert(xy, np.flatnonzero(np.diff(t) > max_gap) + 1, np.nan, axis=0)


def separation(a, b):
    """Same-time separation for audit only; never interpolate plotted vertices."""
    k = original.nearest_indices(b[:, 0], a[:, 0])
    if np.array_equal(a[:, 0], b[k, 0]):
        matched = b[k, 1:4]
        valid = np.ones(len(a), dtype=bool)
    else:
        right = np.searchsorted(b[:, 0], a[:, 0])
        left = np.clip(right - 1, 0, len(b) - 1)
        right = np.clip(right, 0, len(b) - 1)
        valid = ((a[:, 0] >= b[0, 0]) & (a[:, 0] <= b[-1, 0]) &
                 (b[right, 0] - b[left, 0] <= 1.5))
        valid |= a[:, 0] == b[k, 0]
        matched = np.column_stack([np.interp(a[:, 0], b[:, 0], b[:, i]) for i in (1, 2, 3)])
    if not valid.any():
        raise ValueError('No common timestamps for separation')
    delta = a[valid, 1:4] - matched[valid]
    return {'samples': int(valid.sum()),
            'max_xy_separation_m': float(np.linalg.norm(delta[:, :2], axis=1).max()),
            'max_3d_separation_m': float(np.linalg.norm(delta, axis=1).max())}


def draw(output, name, curves, labels, window, t0, records, export_data=True):
    lo, hi = window
    selected = [a[(a[:, 0] - t0 >= lo) & (a[:, 0] - t0 <= hi)] for a in curves]
    if any(len(a) < 2 for a in selected):
        raise ValueError('Insufficient vertices for ' + name)
    if any(c in label for label in labels for c in '()（）'):
        raise ValueError('Parentheses forbidden in figure labels')
    xy = np.vstack([a[:, 1:3] for a in selected])
    center = (xy.max(axis=0) + xy.min(axis=0)) / 2
    fig = plt.figure(figsize=(8.0, 3.45), facecolor='white')
    ax = fig.add_axes([0.035, 0.055, 0.93, 0.74], facecolor='white')
    for a, label, color in zip(selected, labels, (RED, BLUE)):
        points = split_at_gaps(a[:, 0], a[:, 1:3] - center)
        ax.plot(points[:, 0], points[:, 1], color=color, linewidth=2.3,
                linestyle='-', solid_capstyle='round', solid_joinstyle='round', label=label)
    ax.set_aspect('equal', adjustable='datalim')
    ax.margins(0.075)
    ax.set_axis_off()
    fig.legend(loc='upper center', bbox_to_anchor=(0.5, 0.985), ncol=2,
               frameon=False, fontsize=15, handlelength=2.0,
               columnspacing=1.6, handletextpad=0.55)
    # ponytail: fixed canvas matches the small PPT result boxes; edit figsize
    # for a different slide layout instead of adding a layout engine.
    for suffix in ('png', 'svg'):
        fig.savefig(str(output / (name + '.' + suffix)), dpi=300, facecolor='white')
    plt.close(fig)
    for i, a in enumerate(selected if export_data else []):
        np.savetxt(str(output / 'evidence' / '{}_{}.csv'.format(name, i + 1)),
                   np.column_stack((a[:, 0], a[:, 0] - t0, a[:, 1:4])),
                   delimiter=',', fmt='%.12f',
                   header='timestamp,elapsed_s,x_m,y_m,z_m', comments='')
    record = {'name': name, 'labels': labels, 'window_s': list(window),
              'timestamp_origin': t0, 'vertex_counts': [len(a) for a in selected],
              'shared_plot_translation_xy_m': center.tolist(),
              'combined_extent_xy_m': np.ptp(xy, axis=0).tolist(),
              'separation_is_not_error': separation(selected[1], selected[0])}
    records.append(record)
    return record


def self_check():
    q = np.array([0., 0., math.sin(math.pi / 4), math.cos(math.pi / 4)])
    raw = np.array([[1., 1., 2., 3., 0., 0., 0., 1.],
                    [2., 4., 5., 6., 0., 0., 0., 1.]])
    lever = np.array([.1, .2, .3])
    expected_r, expected_t = rotation(q), np.array([5., 6., 7.])
    online = raw[:1].copy()
    online[0, 1:4] = expected_r @ (raw[0, 1:4] + lever) + expected_t
    online[0, 4:8] = q
    result, r, t = initial_frame(raw, online, lever)
    assert np.allclose(r, expected_r) and np.allclose(t, expected_t)
    assert np.allclose(result[:, 1:4], (raw[:, 1:4] + lever) @ r.T + t)
    assert np.allclose(np.linalg.norm(np.diff(result[:, 1:4], axis=0), axis=1),
                       np.linalg.norm(np.diff(raw[:, 1:4], axis=0), axis=1))
    points = np.array([[1., 2.], [3., 4.], [5., 6.]])
    split = split_at_gaps(np.array([0., 1., 4.]), points)
    assert np.isnan(split[2]).all() and np.array_equal(split[[0, 1, 3]], points)
    short = raw.copy()
    short[1, 0] = 5.
    mid = raw.copy()
    mid[:, 0] = [2., 3.]
    try:
        separation(mid, short)
    except ValueError:
        pass
    else:
        raise AssertionError('Interpolation across a time gap must be rejected')
    mid[1, 0] = 5.
    assert separation(mid, short)['samples'] == 1
    with tempfile.TemporaryDirectory(prefix='ppt_trajectory_check_') as directory:
        path = Path(directory) / 'repeated.tum'
        revised = raw[:1].copy()
        revised[0, 1] += 9.
        np.savetxt(str(path), np.vstack([raw[:1], revised, raw[1:]]))
        assert np.allclose(read_first_emission(path), raw)
    print('PASS initial rigid frame, lever arm, gap pen lift, association boundary')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--archive', type=Path, default=ARCHIVE)
    parser.add_argument('--output-dir', type=Path, default=OUTPUT)
    parser.add_argument('--self-check', action='store_true')
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    archive, output = args.archive.resolve(), args.output_dir.resolve()
    (output / 'evidence').mkdir(parents=True, exist_ok=True)
    original.configure_chinese_font()
    plt.rcParams['path.simplify'] = False
    plt.rcParams['svg.fonttype'] = 'path'
    metrics = json.loads((archive / 'metrics.json').read_text())
    dynamic, fixed = archive / 'runs/dynamic', archive / 'runs/fixed'
    config = original.audit_ablation_configs(dynamic, fixed)
    paired = original.audit_same_native_inputs(dynamic, fixed)
    t0 = metrics['window']['start_unix_s']
    paths = {'raw': dynamic / 'native/livo_raw_online.tum',
             'online': dynamic / 'native/rtk_optimized_online.tum',
             'final': dynamic / 'native/rtk_optimized_final.tum',
             'fixed': fixed / 'native/rtk_optimized_online.tum'}
    data = {key: original.read_tum(path) for key, path in paths.items()}
    if not all(np.array_equal(data['online'][:, 0], data[key][:, 0]) for key in ('fixed', 'final')):
        raise ValueError('Paired RTK timelines differ')
    lever = original.scalar_vector(config['dynamic'], 'result_pose_lever_arm_body_m')
    raw_global, r, t = initial_frame(data['raw'], data['online'], lever)
    records = []
    rows = [
        ('01_gnss_fusion', [raw_global, data['online']], ['LIVO 前端', '融合 GNSS'], (280., 315.)),
        ('02_adaptive_weight', [data['fixed'], data['online']], ['GNSS 固定权重', 'GNSS 自适应权重'], (285., 315.)),
        ('03_fixed_lag', [data['online'], data['final']], ['在线输出', '20 s 固定滞后最终结果'], (310., 314.)),
    ]
    for name, curves, labels, window in rows:
        draw(output, name, curves, labels, window, t0, records)
        draw(output / 'evidence', name + '_context', curves, labels,
             (float(data['online'][0, 0] - t0), 360.), t0, [], export_data=False)

    uwb_base = Path('/home/gulu/data/2026-08-15-151219')
    uwb_fused = Path('/home/gulu/data/2026-08-15-151546')
    uwb_raw_hash = original.sha256(uwb_base / 'livo_raw_online.tum')
    if uwb_raw_hash != original.sha256(uwb_fused / 'livo_raw_online.tum'):
        raise ValueError('UWB pair does not have identical frontend inputs')
    paths.update(uwb_baseline=uwb_base / 'rtk_optimized_online.tum',
                 uwb_fused=uwb_fused / 'rtk_optimized_online.tum')
    uwb = [read_first_emission(paths[k]) for k in ('uwb_baseline', 'uwb_fused')]
    if not np.allclose(uwb[0][0], uwb[1][0], rtol=0, atol=1e-10):
        raise ValueError('UWB pair initial frame mismatch')
    draw(output, '04_uwb_0709b_optional', uwb, ['未融合 UWB', '融合 UWB'],
         (375., 389.), uwb[0][0, 0], records)

    gnss_804 = Path('/home/gulu/data/804')
    paths.update(raw_804=gnss_804 / 'livo_raw_online.tum',
                 fused_804=gnss_804 / 'rtk_optimized_online.tum')
    raw_804 = read_first_emission(paths['raw_804'])
    fused_804 = read_first_emission(paths['fused_804'])
    # This archived version outputs the body origin. Its README/log, rather
    # than the current RTK-SLAM lever-arm configuration, defines that contract.
    mapped_804, r804, t804 = initial_frame(raw_804, fused_804, np.zeros(3))
    draw(output, '05_gnss_804_optional', [mapped_804, fused_804],
         ['LIVO 在线轨迹', 'GNSS 融合在线轨迹'], (1080., 1105.), raw_804[0, 0], records)

    sources = [{'role': key, 'path': str(path), 'sha256': original.sha256(path)}
               for key, path in paths.items()]
    for key, path in [('original_metrics', archive / 'metrics.json'),
                      ('dynamic_config', config['dynamic_path']),
                      ('fixed_config', config['fixed_path']),
                      ('804_frame_contract', gnss_804 / 'README.md'),
                      ('804_initial_alignment_log', gnss_804 / 'rtk_backend.log'),
                      ('uwb_initial_alignment_log', uwb_fused / 'rtk_backend.log'),
                      ('uwb_recorded_measurements', uwb_fused / 'uwb_backend_measurements.csv'),
                      ('plot_script', Path(__file__).resolve()),
                      ('plot_helpers', Path(original.__file__).resolve())]:
        sources.append({'role': key, 'path': str(path), 'sha256': original.sha256(path)})
    report = {'archive': str(archive), 'sources': sources,
              'protocol': {'scope': 'archived experiments redrawn; no new replay',
                           'coordinates': 'source ENU or local map XY; shared translation only',
                           'aspect': 'equal metric scale on both axes',
                           'drawn_vertices': 'original samples; no smoothing, interpolation or residual exaggeration',
                           'time_gap_pen_lift_s': 1.5,
                           'selection': 'post-hoc diagnostic detail windows; not independent evidence of whole-run superiority',
                           'ground_truth': 'not drawn; no synthetic continuous truth'},
              'initial_frame_rtk': {'rotation': r.tolist(), 'translation_m': t.tolist(), 'lever_m': lever.tolist()},
              'initial_frame_804': {'rotation': r804.tolist(), 'translation_m': t804.tolist()},
              'same_input_rtk': paired, 'same_input_uwb_raw_sha256': uwb_raw_hash,
              'figures': records}
    (output / 'evidence/provenance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(records, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
