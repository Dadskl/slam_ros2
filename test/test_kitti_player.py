"""Validate KITTI color calibration and the actual stereo topic pipeline."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import cv2
import numpy as np
import pytest
from experiment_assertions import validate_experiment

ROOT = Path(__file__).resolve().parents[1]
player = pytest.importorskip('kitti_dataset_tools.player', reason='Install optional kitti_dataset_tools for KITTI integration testing')


def make_sequence(directory, frames=3):
    for name in ('image_2', 'image_3'):
        (directory / name).mkdir(parents=True)
    rng = np.random.default_rng(19)
    texture = cv2.GaussianBlur(rng.integers(0, 256, (480, 640, 3), dtype=np.uint8), (3, 3), 0)
    for i in range(frames):
        left = np.roll(texture, i // 3, axis=1)
        right = np.roll(left, -14, axis=1)
        cv2.imwrite(str(directory / 'image_2' / f'{i:06d}.png'), left)
        cv2.imwrite(str(directory / 'image_3' / f'{i:06d}.png'), right)
    (directory / 'times.txt').write_text(''.join(f'{i * 0.1:.6f}\n' for i in range(frames)))
    # Nonzero left projection translation catches the incorrect -P3[0,3]/fx baseline formula.
    (directory / 'calib.txt').write_text(
        'P2: 280 0 320 7 0 280 240 0 0 0 1 0\n'
        'P3: 280 0 320 -21 0 280 240 0 0 0 1 0\n')
    return directory


def test_stereo_topics(tmp_path, monkeypatch):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import PointCloud2
    from nav_msgs.msg import Path as PathMsg
    seq = make_sequence(tmp_path / '00', frames=40)
    settings = tmp_path / 'camera.yaml'
    player.main(['--sequence-dir', str(seq), '--settings-output', str(settings), '--prepare-only'])
    domain = 40 + os.getpid() % 150
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=domain)
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor(context=context)
    node = rclpy.create_node('stereo_smoke', context=context)
    executor.add_node(node)
    received = {'pose': [], 'cloud': [], 'path': []}
    subscriptions = [node.create_subscription(kind, topic, lambda msg, k=key: received[k].append(msg), 10)
                     for key, kind, topic in [('pose', PoseStamped, '/camera_pose'),
                                              ('cloud', PointCloud2, '/point_cloud_ref'),
                                              ('path', PathMsg, '/camera_path')]]
    executable = os.environ.get('ORB_SLAM3_EXECUTABLE', 'orb_slam3_main')
    command = [executable, '--ros-args', '-p', 'sensor_mode:=stereo',
               '-p', f'settings_file:={settings}', '-p', 'enable_pangolin:=false',
               '-p', 'use_sim_time:=true', '-p', 'image_reliability:=reliable',
               '-p', f'experiment_output_dir:={tmp_path}/results',
               '-p', f'vocabulary_file:={ROOT}/orb_slam3/Vocabulary/ORBvoc.txt.bin']
    slam = replay = None
    log_path = tmp_path / 'stereo.log'
    try:
        with log_path.open('w') as log:
            slam = subprocess.Popen(command, env=env, stdout=log, stderr=log)
            replay = subprocess.Popen([sys.executable, str(ROOT / 'scripts/kitti_player.py'),
                                       '--sequence-dir', str(seq), '--publish-clock'],
                                      env=env, stdout=log, stderr=log)
            deadline = time.monotonic() + 70
            while replay.poll() is None:
                assert slam.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                executor.spin_once(timeout_sec=0.05)
            assert replay.returncode == 0, log_path.read_text()
            until = time.monotonic() + 3
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.05)
            assert all(received.values()), log_path.read_text()
            cloud = received['cloud'][-1]
            assert cloud.width * cloud.height > 100
            offset = next(field.offset for field in cloud.fields if field.name == 'x')
            x = np.ndarray((cloud.width * cloud.height,), dtype='<f4', buffer=bytes(cloud.data),
                           offset=offset, strides=(cloud.point_step,))
            assert np.isfinite(x).all()
            assert 1.7 < np.median(x) < 2.3
            assert 0 < len(received['path'][-1].poses) <= 40
            # Headers use dataset timestamps, not the wall clock.
            assert received['pose'][-1].header.stamp.sec < 10
            slam.send_signal(signal.SIGINT)
            assert slam.wait(timeout=20) == 0, log_path.read_text()
            validate_experiment(tmp_path / 'results', mode='stereo')
    finally:
        for process in (replay, slam):
            if process is not None and process.poll() is None:
                process.kill()
                process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()
