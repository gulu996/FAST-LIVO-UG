#!/usr/bin/env python3
"""Presentation cards with an original route and a linked metric-scale detail."""

import argparse
import csv
import json
from pathlib import Path

import numpy as np
import plot_ppt_real_trajectory_details as previous
import matplotlib.pyplot as plt
from matplotlib.patches import ConnectionPatch, FancyBboxPatch, Rectangle


BLUE, GREY = '#1879BD', '#939CA8'
OUTPUT = Path(__file__).resolve().parents[1] / 'artifacts/ppt_real_20260906_v2'


def reference_points_in_enu(rows, origin, coords):
    """Invert the official coordinate map, keeping the existing curves intact."""
    def forward(position):
        return previous.original.enu_positions_to_utm(position[None, :], origin, coords)[0]

    zero_utm = forward(np.zeros(3))
    result = []
    for row in rows:
        target = np.asarray(row['position'], dtype=float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError('Invalid measured reference coordinates')
        position = target - zero_utm
        # ponytail: this bounded Newton solve is for nearby survey points in
        # one local ENU frame; use a global projection inverse for wider areas.
        for _ in range(6):
            residual = forward(position) - target
            if np.linalg.norm(residual) < 1e-7:
                break
            jacobian = np.column_stack([(forward(position + unit) - forward(position - unit)) / 2
                                        for unit in np.eye(3)])
            position -= np.linalg.solve(jacobian, residual)
        error = float(np.linalg.norm(forward(position) - target))
        if not np.isfinite(position).all() or error > 1e-6:
            raise ValueError('Official reference coordinate inverse failed')
        result.append(dict(row, position=position, utm_position=target,
                           coordinate_roundtrip_error_m=error))
    return result


def view_transform(curves):
    """One rigid camera rotation for both curves, never one fit per method."""
    origin = curves[1][0, 1:3].copy()
    points = np.vstack([curve[:, 1:3] for curve in curves])
    _, _, vt = np.linalg.svd(points - points.mean(axis=0), full_matrices=False)
    direction = vt[0]
    if np.dot(curves[1][-1, 1:3] - origin, direction) < 0:
        direction = -direction
    r = np.array([direction, [-direction[1], direction[0]]])
    return origin, r


def limits(points, width_over_height):
    lo, hi = points.min(axis=0), points.max(axis=0)
    center = (lo + hi) / 2
    extent = np.maximum(hi - lo, 0.001) * 1.20
    extent[0] = max(extent[0], extent[1] * width_over_height)
    extent[1] = extent[0] / width_over_height
    return center - extent / 2, center + extent / 2


def route(ax, data, xy, color, label=None, width=1.25):
    vertices = previous.split_at_gaps(data[:, 0], xy)
    line, = ax.plot(vertices[:, 0], vertices[:, 1], color=color, lw=width,
                    linestyle='-', solid_capstyle='round',
                    solid_joinstyle='round', label=label, zorder=5)
    return line


def endpoint(ax, point, color):
    ax.scatter(*point, s=110, facecolors='white', edgecolors=color,
               linewidths=2.2, zorder=9)
    ax.scatter(*point, s=15, color=color, zorder=10)


def card(output, name, curves, labels, overview_window, detail_window, t0, show_numbers, references=None):
    common_start = max(max(curve[0, 0] for curve in curves), t0 + overview_window[0])
    common_end = min(min(curve[-1, 0] for curve in curves), t0 + overview_window[1])
    curves = [curve[(curve[:, 0] >= common_start) & (curve[:, 0] <= common_end)]
              for curve in curves]
    origin, r = view_transform(curves)
    xy = [(curve[:, 1:3] - origin) @ r.T for curve in curves]
    masks = [(curve[:, 0] - t0 >= detail_window[0]) &
             (curve[:, 0] - t0 <= detail_window[1]) for curve in curves]
    details = [curve[mask] for curve, mask in zip(curves, masks)]
    detail_xy = [points[mask] for points, mask in zip(xy, masks)]
    if any(len(curve) < 2 for curve in details):
        raise ValueError('Empty detail window')

    fig = plt.figure(figsize=(9, 4.1), facecolor='white')
    ax = fig.add_axes([0.038, 0.11, 0.52, 0.65])
    zoom = fig.add_axes([0.625, 0.22, 0.335, 0.50])
    zoom.set_zorder(4)
    # The border identifies a magnification of the overview, without adding
    # a grid or a decorative terrain/background that could resemble data.
    panel = FancyBboxPatch((0.593, 0.105), 0.387, 0.66,
                           boxstyle='round,pad=0.008,rounding_size=0.025',
                           transform=fig.transFigure, facecolor='white',
                           edgecolor='#C7DDEB', linewidth=1.1, zorder=2)
    fig.add_artist(panel)
    handles = []
    for a, points, cut, cut_xy, label, color in zip(curves, xy, details, detail_xy, labels, (GREY, BLUE)):
        handles.append(route(ax, a, points, color, label))
        # Plot the whole route and let the axes clip it: every revisit inside
        # the displayed rectangle remains visible in the magnifier.
        route(zoom, a, points, color, width=1.4)

    for axis, points in ((ax, np.vstack(xy)), (zoom, np.vstack(detail_xy))):
        box = axis.get_position()
        ratio = box.width * 9 / (box.height * 4.1)
        low, high = limits(points, ratio)
        axis.set_xlim(low[0], high[0])
        axis.set_ylim(low[1], high[1])
        axis.set_aspect('equal', adjustable='box')
        axis.set_axis_off()

    endpoint(ax, xy[1][0], '#299D77')
    endpoint(ax, xy[1][-1], '#E1786A')
    ax.scatter(*xy[0][-1], s=12, color=GREY, zorder=11)
    low = np.array([zoom.get_xlim()[0], zoom.get_ylim()[0]])
    high = np.array([zoom.get_xlim()[1], zoom.get_ylim()[1]])
    roi = Rectangle(low, *(high - low),
                    fill=False, ec='#65A4CE', lw=1.1, zorder=8)
    ax.add_patch(roi)
    connector = ConnectionPatch(xyA=(high[0], (low[1] + high[1]) / 2),
                               coordsA=ax.transData, xyB=(0.596, 0.435),
                               coordsB=fig.transFigure, color='#A8C6DB',
                               linewidth=1.0, zorder=1)
    fig.add_artist(connector)

    # Three actual paired timestamps make the correspondence visible in the
    # magnifier. Dots are samples, and solid thin ties are same-time distances.
    i = previous.original.nearest_indices(details[0][:, 0], details[1][:, 0])
    exact = np.flatnonzero(np.abs(details[0][i, 0] - details[1][:, 0]) < 1e-6)
    if len(exact) >= 3:
        chosen = exact[np.linspace(0, len(exact) - 1, 3).astype(int)]
        for j in chosen:
            pair = np.vstack((detail_xy[0][i[j]], detail_xy[1][j]))
            zoom.plot(pair[:, 0], pair[:, 1], color='#CFD9E0', lw=0.9, zorder=3)
            for point, color in zip(pair, (GREY, BLUE)):
                zoom.scatter(*point, s=12, facecolors='white', edgecolors=color,
                             linewidths=0.9, zorder=7)

    labels = list(labels)
    reference_records = []
    if references is not None:
        selected_refs = [row for row in references if common_start <= row['timestamp'] <= common_end]
        ref_xy = np.asarray([(row['position'][:2] - origin) @ r.T for row in selected_refs]).reshape(-1, 2)
        for axis in (ax, zoom):
            ref_handle = axis.scatter(ref_xy[:, 0], ref_xy[:, 1], marker='*', s=85,
                                      facecolors='#263443', edgecolors='white',
                                      linewidths=0.6, zorder=12)
        handles.append(ref_handle)
        labels.append('实测参考点')
        for row, point in zip(selected_refs, ref_xy):
            reference_records.append({'id': row['id'], 'timestamp': row['timestamp'],
                                      'elapsed_s': row['timestamp'] - t0,
                                      'utm_position_m': row['utm_position'].tolist(),
                                      'enu_position_m': row['position'].tolist(),
                                      'view_xy_m': point.tolist(),
                                      'visible_in_magnifier': bool(np.all(point >= low) and np.all(point <= high)),
                                      'coordinate_roundtrip_error_m': row['coordinate_roundtrip_error_m']})
        with (output/'evidence'/(name+'_references.csv')).open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow(['point_id', 'timestamp', 'elapsed_s', 'easting_m', 'northing_m', 'height_m',
                             'enu_x_m', 'enu_y_m', 'enu_z_m', 'view_x_m', 'view_y_m', 'visible_in_magnifier'])
            for row in reference_records:
                writer.writerow([row['id'], row['timestamp'], row['elapsed_s'], *row['utm_position_m'],
                                 *row['enu_position_m'], *row['view_xy_m'], int(row['visible_in_magnifier'])])

    fig.legend(handles=handles, labels=labels, loc='upper center',
               bbox_to_anchor=(0.5, 0.96), ncol=len(labels), frameon=False,
               fontsize=14.0, handlelength=2.1, columnspacing=2.0)
    metric = previous.separation(details[1], details[0])
    if show_numbers:
        value = metric['max_xy_separation_m']
        text = '最大轨迹差 {:.1f} cm'.format(value * 100) if value < 1 else '最大轨迹差 {:.2f} m'.format(value)
        fig.text(0.785, 0.148, text, ha='center', color='#527083', fontsize=10.5)

    fig.canvas.draw()
    for axis in (ax, zoom):
        p = axis.transData.transform([[0, 0], [1, 0], [0, 1]])
        assert np.isclose(np.linalg.norm(p[1]-p[0]), np.linalg.norm(p[2]-p[0]), rtol=1e-9)
    assert not any(c in label for label in labels for c in '()（）')
    for suffix in ('png', 'svg'):
        fig.savefig(str(output / (name + '.' + suffix)), dpi=300, facecolor='white')
    plt.close(fig)
    for j, (a, points, mask) in enumerate(zip(curves, xy, masks), 1):
        np.savetxt(str(output / 'evidence' / '{}_{}.csv'.format(name, j)),
                   np.column_stack((a[:, :4], points, mask.astype(int))),
                   delimiter=',', fmt='%.12f', comments='',
                   header='timestamp,source_x_m,source_y_m,source_z_m,view_x_m,view_y_m,in_focus_time_window')
    return {'name': name, 'labels': labels, 'time_origin': t0,
            'overview_window_s': [common_start-t0, common_end-t0],
            'overview_sample_windows_s': [[a[0, 0]-t0, a[-1, 0]-t0] for a in curves],
            'detail_window_s': list(detail_window),
            'magnifier_content': 'all overview samples spatially clipped; detail time window selects bounds and paired markers only',
            'magnifier_limits_view_m': [low.tolist(), high.tolist()],
            'overview_samples': [len(a) for a in curves],
            'detail_samples': [len(a) for a in details],
            'shared_origin_xy_m': origin.tolist(), 'shared_rotation': r.tolist(),
            'pair_start_separation_xy_m': float(np.linalg.norm(xy[0][0]-xy[1][0])),
            'measured_references': reference_records,
            'detail_separation_not_accuracy': metric}


def self_check():
    a = np.array([[1, 3, 4, 0], [2, 5, 7, 0], [3, 9, 6, 0]], dtype=float)
    b = a.copy()
    b[:, 2] += [.0, .2, .4]
    origin, r = view_transform([a, b])
    assert np.allclose(r @ r.T, np.eye(2)) and np.isclose(np.linalg.det(r), 1)
    x, y = (a[:, 1:3]-origin) @ r.T, (b[:, 1:3]-origin) @ r.T
    assert np.allclose(x[0], y[0])
    assert np.allclose(np.linalg.norm(x-y, axis=1), [0, .2, .4])
    assert np.allclose(np.linalg.norm(np.diff(x, axis=0), axis=1),
                       np.linalg.norm(np.diff(a[:, 1:3], axis=0), axis=1))
    lo, hi = limits(np.vstack([x, y]), 1.8)
    assert np.isclose((hi[0]-lo[0])/(hi[1]-lo[1]), 1.8)
    archive = previous.ARCHIVE
    _, coords, _ = previous.original.load_official_helpers(archive/'official_eval')
    enu_origin = previous.original.read_origin(archive/'runs/dynamic')
    expected = np.array([[0., 0., 0.], [-100., -40., 1.], [10., 5., 2.]])
    utm = previous.original.enu_positions_to_utm(expected, enu_origin, coords)
    restored = reference_points_in_enu([{'position': point} for point in utm], enu_origin, coords)
    assert np.allclose([row['position'] for row in restored], expected, atol=1e-6, rtol=0)
    print('PASS shared rigid view preserves alignment, distances and metric scale')
    print('PASS official UTM/ENU reference roundtrip including height convention')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=OUTPUT)
    parser.add_argument('--numbers', action='store_true')
    parser.add_argument('--reference-points', action='store_true')
    parser.add_argument('--self-check', action='store_true')
    args = parser.parse_args()
    if args.self_check:
        self_check()
        return
    output = args.output_dir.resolve()
    (output / 'evidence').mkdir(parents=True, exist_ok=True)
    previous.original.configure_chinese_font()
    plt.rcParams.update({'path.simplify': False, 'svg.fonttype': 'path'})
    archive = previous.ARCHIVE
    dynamic, fixed = archive/'runs/dynamic', archive/'runs/fixed'
    config = previous.original.audit_ablation_configs(dynamic, fixed)
    paired = previous.original.audit_same_native_inputs(dynamic, fixed)
    paths = {'raw': dynamic/'native/livo_raw_online.tum',
             'online': dynamic/'native/rtk_optimized_online.tum',
             'fixed': fixed/'native/rtk_optimized_online.tum',
             'final': dynamic/'native/rtk_optimized_final.tum'}
    data = {k: previous.original.read_tum(v) for k, v in paths.items()}
    lever = previous.original.scalar_vector(config['dynamic'], 'result_pose_lever_arm_body_m')
    raw, initial_r, initial_t = previous.initial_frame(data['raw'], data['online'], lever)
    t0 = json.loads((archive/'metrics.json').read_text())['window']['start_unix_s']
    references = None
    reference_sources = {}
    if args.reference_points:
        _, coords, readers = previous.original.load_official_helpers(archive/'official_eval')
        reference_sources = {'ground_truth': archive/'ground_truth/stadtgarten_seq2.csv',
                             'official_coordinates': archive/'official_eval/coords.py',
                             'enu_origin': dynamic/'enu_origin.json'}
        references = reference_points_in_enu(readers.read_ground_truth(str(reference_sources['ground_truth'])),
                                             previous.original.read_origin(dynamic), coords)
    rows = [('01_gnss_fusion', [raw, data['online']], ['LIVO 前端', 'GNSS 融合'], (0., 360.), (276., 320.)),
            ('02_adaptive_weight', [data['fixed'], data['online']], ['GNSS 固定权重', 'GNSS 自适应权重'], (180., 335.), (280., 318.)),
            ('03_fixed_lag', [data['online'], data['final']], ['在线输出', '20 s 固定滞后结果'], (275., 316.), (310., 315.))]
    results = [card(output, name, curves, labels, overview, window, t0, args.numbers, references)
               for name, curves, labels, overview, window in rows]
    paths.update(reference_sources)
    report = {'sources': [{'role': k, 'path': str(v), 'sha256': previous.original.sha256(v)} for k,v in paths.items()],
              'protocol': 'archived actual samples, shared initial frame, shared rigid camera rotation, equal metric scale, no per-method refitting or smoothing',
              'same_input': paired, 'initial_frame_rotation': initial_r.tolist(),
              'initial_frame_translation_m': initial_t.tolist(), 'result_lever_m': lever.tolist(),
              'annotations': 'names and numeric separation' if args.numbers else 'names only',
              'reference_semantics': 'independent sparse surveyed points, transformed using inverse official coordinate map, unconnected markers only' if args.reference_points else None,
              'figures': results}
    (output/'evidence/provenance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
