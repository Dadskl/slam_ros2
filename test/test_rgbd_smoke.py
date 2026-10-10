"""Exercise the real node with best-effort RGB-D, without a camera or display."""
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import cv2
import numpy as np
import pytest
from experiment_assertions import validate_experiment
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from sensor_msgs.msg import Image, PointCloud2


ROOT = Path(os.environ.get('ORB_SLAM3_SOURCE_DIR', Path(__file__).resolve().parents[1]))
EXE = os.environ.get('ORB_SLAM3_EXECUTABLE', 'orb_slam3_main')


def command(settings):
    return [EXE, '--ros-args',
            '-p', f'vocabulary_file:={ROOT}/orb_slam3/Vocabulary/ORBvoc.txt.bin',
            '-p', f'settings_file:={settings}',
            '-p', 'enable_pangolin:=false', '-p', 'publish_rate_hz:=20.0',
            '-p', 'rgb_topic:=/test/rgb', '-p', 'depth_topic:=/test/depth']


def test_missing_settings():
    result = subprocess.run(command('/nonexistent/orbslam3-test.yaml'),
                            capture_output=True, text=True, timeout=20)
    assert result.returncode == 1
    assert 'Required file does not exist' in result.stderr


@pytest.mark.parametrize('encoding,scenario', [
    ('16UC1', 'normal'), ('32FC1', 'normal'), ('32FC1', 'lost'), ('32FC1', 'empty'), ('32FC1', 'blank')])
def test_rgbd_pipeline(tmp_path, encoding, scenario, monkeypatch):
    # Separate domain from a user's running camera/robot.
    domain = 40 + os.getpid() % 150
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    monkeypatch.setenv('ROS_AUTOMATIC_DISCOVERY_RANGE', 'LOCALHOST')
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    settings = tmp_path / 'camera.yaml'
    text = (ROOT / 'orb_slam3/config/RGB-D/RealSense_D435i.yaml').read_text()
    for key in ['k1', 'k2', 'p1', 'p2', 'k3']:
        text = re.sub(rf'^Camera1\.{key}:.*$', f'Camera1.{key}: 0.0', text, flags=re.M)
    settings.write_text(text)
    log_path = tmp_path / 'node.log'
    context = rclpy.context.Context()
    rclpy.init(context=context, domain_id=domain)
    from rclpy.executors import SingleThreadedExecutor
    executor = SingleThreadedExecutor(context=context)
    node = rclpy.create_node('rgbd_smoke', context=context)
    executor.add_node(node)
    # Fault injection must arrive; normal scenarios retain best-effort camera coverage.
    reliable_input = scenario in ('lost', 'blank')
    input_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE) if reliable_input else qos_profile_sensor_data
    rgb_pub = node.create_publisher(Image, '/test/rgb', input_qos)
    depth_pub = node.create_publisher(Image, '/test/depth', input_qos)
    received = {'pose': [], 'cloud': [], 'path': [], 'frame': []}
    subscriptions = [
        node.create_subscription(msg_type, topic, lambda msg, k=key: received[k].append(msg), 10)
        for key, msg_type, topic in [
            ('pose', PoseStamped, '/camera_pose'),
            ('cloud', PointCloud2, '/cloudPointMapping'),
            ('path', PathMsg, '/camera_path'),
            ('frame', Image, '/current_frame'),
        ]
    ]
    rng = np.random.default_rng(7)
    texture = rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)
    texture = cv2.GaussianBlur(texture, (3, 3), 0)
    depth = np.full((480, 640), 2000 if encoding == '16UC1' else 2.0,
                    dtype=np.uint16 if encoding == '16UC1' else np.float32)
    depth[:12, :12] = 0 if encoding == '16UC1' else np.nan
    if encoding == '32FC1':
        depth[12:24, :12] = np.inf
        depth[24:36, :12] = -1.0
    process = None
    try:
        with log_path.open('w') as log:
            process = subprocess.Popen(command(settings) + ['-p', f'experiment_output_dir:={tmp_path}/results',
                '-p', 'image_reliability:=' + ('reliable' if reliable_input else 'best_effort')], env=env, stdout=log, stderr=log)
            deadline = time.monotonic() + 45
            while rgb_pub.get_subscription_count() == 0 or depth_pub.get_subscription_count() == 0:
                assert process.poll() is None, log_path.read_text()
                assert time.monotonic() < deadline, log_path.read_text()
                executor.spin_once(timeout_sec=0.1)

            def send_frame(index, blank=False):
                rgb = Image()
                rgb.header.stamp = node.get_clock().now().to_msg()
                rgb.header.frame_id = 'camera_color_optical_frame'
                rgb.height, rgb.width, rgb.encoding, rgb.step = 480, 640, 'rgb8', 640 * 3
                pixels = np.zeros_like(texture) if blank else np.roll(texture, index // 3, axis=1)
                rgb.data = pixels.tobytes()
                dep = Image()
                dep.header = rgb.header
                dep.height, dep.width, dep.encoding, dep.step = 480, 640, encoding, 640 * depth.itemsize
                dep.data = depth.tobytes()
                rgb_pub.publish(rgb)
                depth_pub.publish(dep)
                until = time.monotonic() + 0.08
                while time.monotonic() < until:
                    executor.spin_once(timeout_sec=0.01)
                assert process.poll() is None, log_path.read_text()

            if scenario == 'empty':
                process.send_signal(signal.SIGINT)
                assert process.wait(timeout=20) == 0, log_path.read_text()
                validate_experiment(tmp_path / 'results', mode='rgbd', expected_frames=0, expect_map=False)
                return

            # Empty initialization previously exposed an unchecked vpKFs[0].
            for i in range(5):
                send_frame(i, blank=True)
            if scenario == 'blank':
                until = time.monotonic() + 1
                while time.monotonic() < until:
                    executor.spin_once(timeout_sec=0.05)
                process.send_signal(signal.SIGINT)
                assert process.wait(timeout=20) == 0, log_path.read_text()
                _, rows, summary = validate_experiment(tmp_path / 'results', mode='rgbd', expected_frames=5, expect_map=False)
                assert rows and int(summary['valid_pose_frames']) == 0
                return
            for i in range(80):
                send_frame(i)
            if scenario == 'lost':
                for i in range(10):
                    send_frame(i, blank=True)
                for i in range(40):
                    send_frame(i)
            until = time.monotonic() + 5
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.05)
            assert all(received.values()), {key: len(value) for key, value in received.items()}
            cloud = received['cloud'][-1]
            assert cloud.width * cloud.height > 100
            # The optical Z axis becomes ground X; both input encodings must yield 2 metres.
            x_offset = next(field.offset for field in cloud.fields if field.name == 'x')
            x = np.ndarray((cloud.width * cloud.height,), dtype='<f4', buffer=bytes(cloud.data),
                           offset=x_offset, strides=(cloud.point_step,))
            assert np.isfinite(x).all()
            assert 1.8 < np.median(x) < 2.2
            assert len(received['path'][-1].poses) <= (120 if scenario == 'lost' else 80)
            assert all(msg.header.frame_id == 'odom' for msg in received['pose'])
            last_size = len(received['path'][-1].poses)
            until = time.monotonic() + 1
            while time.monotonic() < until:
                executor.spin_once(timeout_sec=0.05)
            assert len(received['path'][-1].poses) == last_size
            process.send_signal(signal.SIGINT)
            assert process.wait(timeout=20) == 0, log_path.read_text()
            _, rows, summary = validate_experiment(tmp_path / 'results', mode='rgbd')
            if scenario == 'lost':
                assert int(summary['tracking_loss_events']) > 0
                assert any(r['valid_pose'] == '0' for r in rows)
    finally:
        if process is not None and process.poll() is None:
            process.kill()
            process.wait()
        executor.shutdown()
        node.destroy_node()
        context.shutdown()


def test_repeated_empty_runs(tmp_path, monkeypatch):
    monkeypatch.delenv('ROS_LOCALHOST_ONLY', raising=False)
    env = dict(os.environ, ROS_DOMAIN_ID='220', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    settings = ROOT / 'orb_slam3/config/RGB-D/RealSense_D435i.yaml'
    results = tmp_path / 'results'
    previous = None
    for i in range(2):
        log_path = tmp_path / f'repeat_{i}.log'
        with log_path.open('w') as log:
            process = subprocess.Popen(command(settings) + ['-p', f'experiment_output_dir:={results}'],
                                       env=env, stdout=log, stderr=log)
            try:
                deadline = time.monotonic() + 45
                while 'rgbd subscriptions ready' not in log_path.read_text():
                    assert process.poll() is None, log_path.read_text()
                    assert time.monotonic() < deadline, log_path.read_text()
                    time.sleep(.05)
                process.send_signal(signal.SIGINT)
                assert process.wait(timeout=20) == 0, log_path.read_text()
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait()
        runs = sorted(results.iterdir())
        assert len(runs) == i + 1
        for run in runs:
            validate_experiment(results, mode='rgbd', expected_frames=0, expect_map=False, run_path=run)
        if previous:
            assert previous[0].read_bytes() == previous[1]
        previous = (runs[0] / 'statistics/summary.txt', (runs[0] / 'statistics/summary.txt').read_bytes())
