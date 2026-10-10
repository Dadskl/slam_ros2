#!/usr/bin/env python3
"""Publish rectified KITTI odometry color pairs; derive ORB settings from P2/P3."""
import argparse
from pathlib import Path
import time

import cv2
import numpy as np


def read_sequence(directory):
    directory = Path(directory).expanduser().resolve()
    left = sorted((directory / 'image_2').glob('*.png'))
    right = sorted((directory / 'image_3').glob('*.png'))
    if not left or [p.name for p in left] != [p.name for p in right]:
        raise ValueError('image_2 and image_3 must contain matching, nonempty PNG frame lists')
    timestamps = np.atleast_1d(np.loadtxt(directory / 'times.txt', dtype=np.float64))
    if timestamps.ndim != 1 or len(timestamps) != len(left):
        raise ValueError('times.txt must contain exactly one timestamp per image pair')
    if not np.isfinite(timestamps).all() or (timestamps < 0).any() or (np.diff(timestamps) <= 0).any():
        raise ValueError('Timestamps must be finite, nonnegative and strictly increasing')
    expected = [f'{i:06d}.png' for i in range(len(left))]
    if [p.name for p in left] != expected:
        raise ValueError('Expected contiguous KITTI frames starting at 000000.png; use --start-frame to skip frames')
    return left, right, timestamps


def read_calibration(directory):
    matrices = {}
    for line in (Path(directory) / 'calib.txt').read_text().splitlines():
        key, separator, values = line.partition(':')
        if separator and key in ('P2', 'P3'):
            array = np.fromstring(values, sep=' ')
            if array.size != 12 or not np.isfinite(array).all():
                raise ValueError(f'{key} must contain 12 finite projection-matrix values')
            matrices[key] = array.reshape(3, 4)
    if set(matrices) != {'P2', 'P3'}:
        raise ValueError('calib.txt must contain P2 and P3 for the color cameras')
    left, right = matrices['P2'], matrices['P3']
    for projection in (left, right):
        if projection[0, 0] <= 0 or projection[1, 1] <= 0 or not np.allclose(projection[2, :3], [0, 0, 1]):
            raise ValueError('Invalid rectified camera intrinsics')
    if not np.allclose(left[:3, :3], right[:3, :3], rtol=1e-5, atol=1e-4):
        raise ValueError('This ORB rectified-stereo input requires matching left/right intrinsics')
    # P = K [I | t]. Use both color camera centers, not the gray P0/P1 baseline.
    center_left = -np.linalg.solve(left[:3, :3], left[:, 3])
    center_right = -np.linalg.solve(right[:3, :3], right[:, 3])
    delta = center_right - center_left
    baseline = float(np.linalg.norm(delta))
    if delta[0] <= 0 or baseline <= 0:
        raise ValueError('Expected image_2 on the left and image_3 on the right with positive baseline')
    return left, right, baseline


def write_settings(path, directory, first_left, first_right, timestamps):
    left, right, baseline = read_calibration(directory)
    image = cv2.imread(str(first_left), cv2.IMREAD_COLOR)
    other = cv2.imread(str(first_right), cv2.IMREAD_COLOR)
    if image is None or other is None or image.shape != other.shape:
        raise ValueError('Cannot read the first matching stereo pair')
    height, width = image.shape[:2]
    fps = max(1, round(1.0 / float(np.median(np.diff(timestamps))))) if len(timestamps) > 1 else 10
    lines = ['%YAML:1.0', '# KITTI rectified COLOR cameras: calib.txt P2/P3',
             'File.version: "1.0"', 'Camera.type: "Rectified"']
    for index, projection in enumerate((left, right), 1):
        for name, value in zip(('fx', 'fy', 'cx', 'cy'),
                               (projection[0, 0], projection[1, 1], projection[0, 2], projection[1, 2])):
            lines.append(f'Camera{index}.{name}: {float(value):.12f}')
    lines += [f'Camera.width: {width}', f'Camera.height: {height}', f'Camera.fps: {fps}',
              'Camera.RGB: 0', f'Stereo.b: {baseline:.12f}', 'Stereo.ThDepth: 35.0',
              'ORBextractor.nFeatures: 2000', 'ORBextractor.scaleFactor: 1.2',
              'ORBextractor.nLevels: 8', 'ORBextractor.iniThFAST: 20', 'ORBextractor.minThFAST: 7',
              'Viewer.KeyFrameSize: 0.6', 'Viewer.KeyFrameLineWidth: 2.0', 'Viewer.GraphLineWidth: 1.0',
              'Viewer.PointSize: 2.0', 'Viewer.CameraSize: 0.7', 'Viewer.CameraLineWidth: 3.0',
              'Viewer.ViewpointX: 0.0', 'Viewer.ViewpointY: -100.0', 'Viewer.ViewpointZ: -0.1',
              'Viewer.ViewpointF: 2000.0']
    output = Path(path).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text('\n'.join(lines) + '\n')
    print(f'Wrote {output}: {width}x{height}, {fps} Hz, color baseline {baseline:.6f} m', flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--sequence-dir', type=Path, help='Path to sequences/00 (or another sequence)')
    source.add_argument('--dataset-root', type=Path, help='Dataset root containing sequences/')
    parser.add_argument('--sequence', help='Sequence number 00..21; required with --dataset-root')
    parser.add_argument('--settings-output', type=Path, help='Write calibrated ORB-SLAM3 stereo YAML here')
    parser.add_argument('--prepare-only', action='store_true', help='Validate data and write settings without publishing')
    parser.add_argument('--left-topic', default='/kitti/left/image_raw')
    parser.add_argument('--right-topic', default='/kitti/right/image_raw')
    parser.add_argument('--mode', choices=('stereo', 'mono'), default='stereo',
                        help='Publish both color cameras or only image_2')
    parser.add_argument('--reliability', choices=('reliable', 'best_effort'), default='reliable',
                        help='Image QoS reliability; reliable supports reliable and best-effort subscribers')
    parser.add_argument('--speed', type=float, default=1.0, help='Playback speed; 0.5 is half real time')
    parser.add_argument('--start-frame', type=int, default=0)
    parser.add_argument('--max-frames', type=int)
    parser.add_argument('--wait-for-subscribers', type=float, default=60.0, help='Timeout in seconds; 0 disables waiting')
    parser.add_argument('--publish-clock', action='store_true', help='Publish dataset time to /clock; set SLAM use_sim_time:=true')
    args = parser.parse_args(argv)
    try:
        if not np.isfinite(args.speed) or args.speed <= 0:
            raise ValueError('--speed must be positive')
        if not np.isfinite(args.wait_for_subscribers) or args.wait_for_subscribers < 0:
            raise ValueError('--wait-for-subscribers must be nonnegative')
        if args.prepare_only and not args.settings_output:
            raise ValueError('--prepare-only requires --settings-output')
        if args.dataset_root:
            if args.sequence is None or not args.sequence.isdecimal() or not 0 <= int(args.sequence) <= 21:
                raise ValueError('--dataset-root requires --sequence between 00 and 21')
            directory = args.dataset_root / 'sequences' / f'{int(args.sequence):02d}'
        else:
            if args.sequence is not None:
                raise ValueError('Use --sequence with --dataset-root, not --sequence-dir')
            directory = args.sequence_dir
        directory = directory.expanduser().resolve()
        left, right, timestamps = read_sequence(directory)
        read_calibration(directory)
        print(f'Sequence: {directory}; {len(left)} stereo pairs; speed: {args.speed}x', flush=True)
        if not 0 <= args.start_frame < len(left) or (args.max_frames is not None and args.max_frames < 1):
            raise ValueError('Invalid --start-frame or --max-frames')
        if args.settings_output:
            write_settings(args.settings_output, directory, left[0], right[0], timestamps)
        if args.prepare_only:
            return 0
    except (ValueError, OSError) as error:
        parser.error(str(error))

    # Keep validation/config generation usable without a running ROS graph.
    import rclpy
    from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    from builtin_interfaces.msg import Time
    rclpy.init(args=[])
    node = rclpy.create_node('kitti_odometry_player')
    image_qos = QoSProfile(depth=10,
                           reliability=(ReliabilityPolicy.RELIABLE if args.reliability == 'reliable'
                                        else ReliabilityPolicy.BEST_EFFORT),
                           durability=DurabilityPolicy.VOLATILE)
    topics = (args.left_topic, args.right_topic) if args.mode == 'stereo' else (args.left_topic,)
    publishers = [node.create_publisher(Image, topic, image_qos) for topic in topics]
    clock_pub = node.create_publisher(Clock, '/clock', qos_profile_sensor_data) if args.publish_clock else None
    try:
        node.get_logger().info(f'Publishing {", ".join(topics)} (bgr8, {args.reliability}, volatile, depth=10)')
        if args.wait_for_subscribers:
            node.get_logger().info('Waiting for subscribers on BOTH image topics...')
        deadline = time.monotonic() + args.wait_for_subscribers
        while args.wait_for_subscribers and any(pub.get_subscription_count() == 0 for pub in publishers):
            if not rclpy.ok():
                return 0
            if time.monotonic() >= deadline:
                raise RuntimeError('Timed out waiting for image subscribers; start the matching SLAM launch first')
            rclpy.spin_once(node, timeout_sec=0.1)
        start = args.start_frame
        end = min(len(left), start + args.max_frames) if args.max_frames else len(left)
        base_ns = 0 if args.publish_clock else node.get_clock().now().nanoseconds - round(timestamps[start] * 1e9)
        wall_start = time.monotonic()
        expected_shape = None
        for i in range(start, end):
            due = wall_start + (timestamps[i] - timestamps[start]) / args.speed
            while rclpy.ok() and time.monotonic() < due:
                remaining = max(0.0, due - time.monotonic())
                rclpy.spin_once(node, timeout_sec=min(0.02, remaining))
            if not rclpy.ok():
                break
            images = [cv2.imread(str(paths[i]), cv2.IMREAD_COLOR) for paths in (left, right)]
            if any(image is None for image in images) or images[0].shape != images[1].shape:
                raise RuntimeError(f'Unreadable or mismatched image pair at frame {i}')
            if expected_shape is None:
                expected_shape = images[0].shape
            if images[0].shape != expected_shape:
                raise RuntimeError(f'Image dimensions change at frame {i}')
            nanoseconds = base_ns + round(timestamps[i] * 1e9)
            stamp = Time(sec=int(nanoseconds // 1000000000), nanosec=int(nanoseconds % 1000000000))
            if clock_pub:
                clock_pub.publish(Clock(clock=stamp))
            for pub, image, frame in zip(publishers, images, ('kitti_left_optical_frame', 'kitti_right_optical_frame')):
                message = Image()
                message.header.stamp = stamp
                message.header.frame_id = frame
                message.height, message.width = image.shape[:2]
                message.encoding = 'bgr8'
                message.step = message.width * 3
                message.data = image.tobytes()
                pub.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            if (i - start) % 100 == 0:
                node.get_logger().info(f'Published {i - start + 1}/{end - start} pairs ({100 * (i - start + 1) / (end - start):.1f}%), frame {i}')
        # Give DDS time to send the last frame pair before destroying the publishers.
        until = time.monotonic() + 0.5
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(node, timeout_sec=0.05)
        return 0
    except KeyboardInterrupt:
        return 0
    except RuntimeError as error:
        node.get_logger().error(str(error))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
