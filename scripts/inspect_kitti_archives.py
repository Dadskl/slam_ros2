#!/usr/bin/env python3
"""Compatibility entry point; implementation lives in kitti_dataset_tools."""
try:
    from kitti_dataset_tools.inspect_archives import *  # noqa: F401,F403
except ModuleNotFoundError as error:
    if error.name != 'kitti_dataset_tools':
        raise
    raise SystemExit('Build kitti_dataset_tools and source its install/setup.bash first.') from error

if __name__ == '__main__':
    raise SystemExit(main())
