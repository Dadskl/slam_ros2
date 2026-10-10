"""Independent checks of experiment files emitted by the real SLAM executable."""
import csv
import math
from pathlib import Path

import numpy as np
import pytest


def validate_experiment(root, *, mode, expected_frames=None, expect_map=True, run_path=None):
    runs = [Path(run_path)] if run_path else sorted(p for p in Path(root).iterdir() if p.is_dir())
    assert len(runs) == 1, runs
    run = runs[0]
    assert {p.name for p in run.iterdir() if p.is_dir()} == {
        'trajectories', 'statistics', 'maps', 'config'}
    assert (run / 'README.txt').stat().st_size > 0
    assert (run / 'config/camera.yaml').read_text().startswith('%YAML:1.0')
    assert 'sensor_mode' in (run / 'config/parameters.txt').read_text()
    provenance = dict(line.split('=', 1) for line in (run / 'config/build_revision.txt').read_text().splitlines())
    assert len(provenance['source_sha256']) == 64
    with (run / 'statistics/frames.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    summary = dict(line.split('=', 1) for line in (run / 'statistics/summary.txt').read_text().splitlines())
    assert summary['sensor_mode'] == mode
    assert int(summary['completed_tracking_calls']) == len(rows)
    if expected_frames is not None:
        assert len(rows) == expected_frames
    received = int(summary['received_images_or_synchronized_pairs'])
    assert received >= len(rows)
    assert int(summary['unprocessed_or_failed_inputs']) == received - len(rows)
    valid = sum(int(row['valid_pose']) for row in rows)
    assert int(summary['valid_pose_frames']) == valid
    assert float(summary['valid_pose_ratio_processed']) == pytest.approx(valid / len(rows) if rows else 0)
    assert float(summary['valid_pose_ratio_received']) == pytest.approx(valid / received if received else 0)
    for i, row in enumerate(rows):
        assert all(math.isfinite(float(v)) for v in row.values())
        assert int(row['processed_index']) == i + 1
        assert 1 <= int(row['input_index']) <= received
        if i:
            assert float(row['timestamp']) > float(rows[i-1]['timestamp'])
            assert int(row['input_index']) > int(rows[i-1]['input_index'])
        assert int(row['tracking_state']) in (-1, 0, 1, 2, 3, 4, 5)
        assert row['valid_pose'] in ('0', '1')
        if row['valid_pose'] == '1':
            assert int(row['tracking_state']) in (2, 5)
        else:
            assert int(row['map_inlier_associations']) == 0
        assert int(row['map_inlier_associations']) <= int(row['features_left']) + int(row['features_right'])
        assert float(row['tracking_ms']) >= max(float(row['extraction_left_ms']), float(row['extraction_right_ms'])) >= 0
        if mode in ('rgbd', 'mono'):
            assert float(row['extraction_right_ms']) == 0
            assert int(row['features_right']) == 0
    times = np.array([float(r['tracking_ms']) for r in rows])
    for name, value in [('mean', times.mean() if len(times) else 0),
                        ('median', np.median(times) if len(times) else 0),
                        ('max', times.max() if len(times) else 0),
                        ('p95', np.sort(times)[math.ceil(.95*len(times))-1] if len(times) else 0)]:
        assert float(summary['tracking_ms_' + name]) == pytest.approx(value)
    with (run / 'statistics/tracking_losses.csv').open() as stream:
        losses = list(csv.DictReader(stream))
    expected_losses = 0
    active = False
    for row in rows:
        if int(row['tracking_state']) in (3, 4) and not active:
            expected_losses += 1
            active = True
        if row['valid_pose'] == '1':
            active = False
    assert len(losses) == int(summary['tracking_loss_events']) == expected_losses
    for loss in losses:
        start = int(loss['first_processed_index']) - 1
        assert int(rows[start]['tracking_state']) in (3, 4)
        end = next((j for j in range(start, len(rows)) if rows[j]['valid_pose'] == '1'), len(rows))
        assert int(loss['recovered']) == int(end < len(rows))
        assert float(loss['end_timestamp']) == float(rows[min(end, len(rows)-1)]['timestamp'])
        assert float(loss['start_timestamp']) == float(rows[start]['timestamp'])
        assert float(loss['observed_duration_seconds']) == pytest.approx(
            float(loss['end_timestamp']) - float(loss['start_timestamp']))
    with (run / 'maps/index.csv').open() as stream:
        maps = list(csv.DictReader(stream))
    assert bool(maps) == expect_map
    assert int(summary['exported_maps']) == len(maps)
    if len(maps) > 1:
        assert not (run / 'trajectories/camera_tum.txt').exists()
        assert not (run / 'trajectories/camera_kitti.txt').exists()
    assert sum(int(m['keyframes']) for m in maps) == int(summary['exported_keyframes'])
    assert sum(int(m['points']) for m in maps) == int(summary['exported_sparse_points'])
    trajectories = {}
    for path in (run / 'trajectories').glob('*.txt'):
        if not path.read_text().strip():
            trajectories[path.name] = np.empty((0, 8))
            continue
        data = np.loadtxt(path, ndmin=2)
        assert np.isfinite(data).all(), path
        assert data.shape[1] == (12 if 'kitti' in path.name else 8)
        if 'kitti' not in path.name:
            assert np.all(np.diff(data[:, 0]) > 0), path
            np.testing.assert_allclose(np.linalg.norm(data[:, 4:8], axis=1), 1, atol=1e-5)
        trajectories[path.name] = data
    for entry in maps:
        stem = 'map_' + entry['map_id']
        keys = trajectories[stem + '_keyframes_tum.txt']
        assert len(keys) == int(entry['keyframes'])
        np.testing.assert_allclose(keys[0, 1:4], 0, atol=1e-5)
        np.testing.assert_allclose(np.abs(keys[0, 4:8]), [0, 0, 0, 1], atol=1e-5)
        camera = trajectories[stem + '_camera_tum.txt']
        kitti = trajectories[stem + '_camera_kitti.txt']
        assert len(kitti) == len(camera)
        if len(kitti):
            kitti = kitti.reshape(-1, 3, 4)
            np.testing.assert_allclose(kitti[:, :, 3], camera[:, 1:4], atol=1e-5)
            np.testing.assert_allclose(np.linalg.det(kitti[:, :, :3]), 1, atol=1e-5)
        valid_times = np.array([float(r['timestamp']) for r in rows if r['valid_pose'] == '1'])
        for stamp in camera[:, 0]:
            assert np.any(np.abs(valid_times - stamp) < 1e-5), ('invalid frame exported', stamp)
        cloud = (run / 'maps' / (stem + '_sparse.ply')).read_text().splitlines()
        header_end = cloud.index('end_header')
        count = int(next(line for line in cloud[:header_end] if line.startswith('element vertex ')).split()[-1])
        assert count == int(entry['points']) == len(cloud) - header_end - 1
        if count:
            xyz = np.array([[float(x) for x in line.split()] for line in cloud[header_end+1:]])
            assert xyz.shape == (count, 3)
            assert np.isfinite(xyz).all()
            if mode == 'rgbd':
                assert 1.8 < np.median(xyz[:, 2]) < 2.2
    if len(maps) == 1 and len(trajectories.get('camera_tum.txt', [])):
        legacy = trajectories['camera_tum.txt']
        per_map = trajectories['map_' + maps[0]['map_id'] + '_camera_tum.txt']
        assert legacy.shape == per_map.shape
        # Legacy timestamps have six decimals; pose composition uses float32 in a
        # different multiplication order (~20 micrometres difference at 100 m).
        np.testing.assert_allclose(legacy[:, 0], per_map[:, 0], atol=5.1e-7, rtol=0)
        np.testing.assert_allclose(legacy[:, 1:4], per_map[:, 1:4], atol=1e-4, rtol=1e-6)
        np.testing.assert_allclose(np.abs(np.sum(legacy[:, 4:8] * per_map[:, 4:8], axis=1)), 1, atol=1e-5)
        kitti = trajectories['camera_kitti.txt'].reshape(-1, 3, 4)
        assert len(kitti) == len(legacy)
        np.testing.assert_allclose(kitti[:, :, 3], legacy[:, 1:4], atol=1e-5)
        rotation = kitti[:, :, :3]
        np.testing.assert_allclose(np.linalg.det(rotation), 1, atol=1e-5)
    return run, rows, summary
