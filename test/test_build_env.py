"""Exercise environment-check policy without changing the host OS or using apt."""
import os
from pathlib import Path
import shlex
import subprocess

import pytest

SCRIPT = Path(__file__).resolve().parents[1] / 'scripts/check_build_env.sh'


def shell(body, env=None):
    clean = dict(os.environ)
    for name in ('ROS_DISTRO', 'ROS_VERSION', 'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH',
                 'COLCON_PREFIX_PATH', 'LD_LIBRARY_PATH', 'PYTHONPATH'):
        clean.pop(name, None)
    clean.update(env or {})
    return subprocess.run(['bash', '-c', 'source ' + shlex.quote(str(SCRIPT)) + '\n' + body],
                          env=clean, text=True, capture_output=True, timeout=20)


def platform(tmp_path, version='24.04', setup=True):
    os_release = tmp_path / 'os-release'
    os_release.write_text(f'ID=ubuntu\nVERSION_ID="{version}"\nPRETTY_NAME="Ubuntu test"\n')
    ros = tmp_path / 'setup.bash'
    if setup:
        ros.write_text('export ROS_VERSION=2\nexport ROS_DISTRO=jazzy\n')
    return f'OS_RELEASE_FILE={shlex.quote(str(os_release))}\nROS_SETUP_FILE={shlex.quote(str(ros))}\n'


def test_supported_platform_and_unsourced_ros(tmp_path):
    result = shell(platform(tmp_path) + 'check_platform\nload_ros\nprintf "%s/%s" "$ROS_VERSION" "$ROS_DISTRO"')
    assert result.returncode == 0, result.stderr
    assert result.stdout.endswith('2/jazzy')


@pytest.mark.parametrize('version,env,setup,message', [
    ('22.04', {}, True, '要求 Ubuntu 24.04'),
    ('24.04', {'ROS_DISTRO': 'humble'}, True, 'ROS_DISTRO=humble'),
    ('24.04', {'ROS_VERSION': '1'}, True, 'ROS_VERSION=1'),
    ('24.04', {}, False, '没有找到'),
    ('24.04', {'CMAKE_PREFIX_PATH': '/opt/ros/jazzy:/opt/ros/humble'}, True, '混入其他 ROS'),
])
def test_platform_rejected_before_install(tmp_path, version, env, setup, message):
    marker = tmp_path / 'install-called'
    body = platform(tmp_path, version, setup)
    body += f'find_missing_packages() {{ touch {shlex.quote(str(marker))}; }}\nmain\n'
    result = shell(body, env)
    assert result.returncode == 2, result.stdout + result.stderr
    assert message in result.stderr
    assert not marker.exists()


def test_lists_missing_packages():
    # Mock dpkg results, not the machine's package database.
    result = shell('''
        dpkg-query() {
            if [[ "${*: -1}" == libopencv-dev ]]; then return 1; fi
            printf 'install ok installed'
        }
        find_missing_packages
        printf 'MISSING=%s\\n' "${MISSING_PACKAGES[*]}"
    ''')
    assert result.returncode == 0
    assert 'MISSING=libopencv-dev' in result.stdout


def test_auto_installs_only_missing_packages(tmp_path):
    calls = tmp_path / 'calls'
    result = shell(f'''
        LOG_DIR={shlex.quote(str(tmp_path))}
        MISSING_PACKAGES=(libeigen3-dev ros-jazzy-cv-bridge)
        sudo() {{ return 0; }}
        run_privileged() {{ printf '%s\\n' "$*" >> {shlex.quote(str(calls))}; }}
        find_missing_packages() {{ MISSING_PACKAGES=(); }}
        install_missing_packages
    ''')
    assert result.returncode == 0, result.stderr
    assert calls.read_text().splitlines() == [
        'apt-get update', 'apt-get install -y --no-remove libeigen3-dev ros-jazzy-cv-bridge']


def test_check_only_never_installs_packages(tmp_path):
    marker = tmp_path / 'called'
    result = shell(f'''
        CHECK_ONLY=1
        MISSING_PACKAGES=(libeigen3-dev)
        run_privileged() {{ touch {shlex.quote(str(marker))}; }}
        install_missing_packages
    ''')
    assert result.returncode == 3
    assert not marker.exists()


def test_install_failure_is_reported(tmp_path):
    result = shell(f'''
        LOG_DIR={shlex.quote(str(tmp_path))}
        MISSING_PACKAGES=(libeigen3-dev)
        sudo() {{ return 0; }}
        run_privileged() {{ echo 'simulated network failure'; return 1; }}
        install_missing_packages
    ''')
    assert result.returncode == 3
    assert 'apt 更新失败' in result.stderr


@pytest.mark.parametrize('check_only', [False, True])
def test_pangolin_repair_or_read_only(tmp_path, check_only):
    marker = tmp_path / 'pangolin-install-called'
    installer = tmp_path / 'install_pangolin.sh'
    installer.write_text(f'#!/usr/bin/env bash\nprintf "%s" "$1" > {shlex.quote(str(marker))}\n')
    (tmp_path / 'pangolin-initial.log').write_text('simulated missing Pangolin\n')
    result = shell(f'''
        LOG_DIR={shlex.quote(str(tmp_path))}
        SCRIPT_DIR={shlex.quote(str(tmp_path))}
        PANGOLIN_PREFIX={shlex.quote(str(tmp_path / 'prefix'))}
        CHECK_ONLY={int(check_only)}
        write_pangolin_probe() {{ :; }}
        probe_count=0
        probe_pangolin() {{ probe_count=$((probe_count+1)); ((probe_count == 2)); }}
        ensure_pangolin
    ''')
    assert result.returncode == (4 if check_only else 0), result.stderr
    assert marker.exists() is not check_only
    if not check_only:
        assert marker.read_text() == str(tmp_path / 'prefix')


def test_bad_arguments():
    result = subprocess.run(['bash', str(SCRIPT), '--jobs', '0'], text=True, capture_output=True)
    assert result.returncode == 64
