#!/usr/bin/env bash
# Prepare dependencies; this does not compile the SLAM project itself.
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
OS_RELEASE_FILE=/etc/os-release
ROS_SETUP_FILE=/opt/ros/jazzy/setup.bash
CHECK_ONLY=0
BUILD_JOBS=2
PANGOLIN_PREFIX="${HOME}/.local"
LOG_DIR=""
MISSING_PACKAGES=()

info() { printf '[信息] %s\n' "$*"; }
ok() { printf '[通过] %s\n' "$*"; }
problem() { printf '[问题] %s\n' "$*" >&2; }
fatal() { local code="$1"; shift; problem "$*"; exit "$code"; }

usage() {
    cat <<'HELP'
用法：bash scripts/check_build_env.sh [选项]

默认：检查 Ubuntu 24.04 / ROS 2 Jazzy，自动用 apt 安装缺少的依赖；
      Pangolin 不可用时构建安装 v0.9.2，最后验证项目 CMake 配置。
      不自动安装/切换 Ubuntu 或 ROS，不编译完整 SLAM 项目。

  --check-only           只检查，不安装包、不构建安装 Pangolin
  --jobs N               Pangolin 构建并行数，默认 2
  --pangolin-prefix DIR  Pangolin 的优先查找及修复安装目录，默认 ~/.local
  --log-dir DIR          保存检查日志、探针和后续编译环境脚本的目录
  -h, --help             显示帮助

两种模式都会编译并运行无图形窗口的 C++ 探针，验证 Pangolin，
并在日志目录内配置项目；不会修改 shell 启动文件或删除已有构建目录。
缺少系统包时默认模式需要 root 或 sudo（终端可能提示输入密码）。
退出码：0=可以编译；2=系统/ROS 不符合；3=系统包问题；
        4=Pangolin 问题；5=环境/配置问题；64=参数错误。
HELP
}

parse_args() {
    while (($#)); do
        case "$1" in
            --check-only) CHECK_ONLY=1; shift ;;
            --jobs|--pangolin-prefix|--log-dir)
                (($# >= 2)) || fatal 64 "$1 缺少参数"
                [[ -n "$2" && "$2" != --* ]] || fatal 64 "$1 缺少有效参数"
                case "$1" in
                    --jobs) BUILD_JOBS="$2" ;;
                    --pangolin-prefix) PANGOLIN_PREFIX="$2" ;;
                    --log-dir) LOG_DIR="$2" ;;
                esac
                shift 2 ;;
            -h|--help) usage; exit 0 ;;
            *) fatal 64 "未知参数：$1；使用 --help 查看用法" ;;
        esac
    done
    [[ "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] || fatal 64 '--jobs 必须是正整数'
    [[ "$PANGOLIN_PREFIX" == /* ]] || PANGOLIN_PREFIX="$PWD/$PANGOLIN_PREFIX"
}

check_platform() {
    local issues=() ID='' VERSION_ID='' PRETTY_NAME='未知系统'
    if [[ -r "$OS_RELEASE_FILE" ]]; then
        # Standard distribution-owned metadata, not user-provided shell input.
        source "$OS_RELEASE_FILE"
        info "系统：${PRETTY_NAME:-$ID $VERSION_ID}；架构：$(uname -m)"
        [[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] ||
            issues+=("要求 Ubuntu 24.04 LTS（含 24.04.x）；当前是 $ID $VERSION_ID。不会自动升级系统。")
    else
        issues+=("无法读取 $OS_RELEASE_FILE，不能确认系统版本。")
    fi
    if [[ -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != jazzy ]]; then
        issues+=("当前 ROS_DISTRO=$ROS_DISTRO；要求 jazzy。请在未加载其他 ROS 的新终端中运行。")
    fi
    if [[ -n "${ROS_VERSION:-}" && "$ROS_VERSION" != 2 ]]; then
        issues+=("当前 ROS_VERSION=$ROS_VERSION；要求 ROS 2。")
    fi
    [[ -r "$ROS_SETUP_FILE" ]] || issues+=("没有找到 $ROS_SETUP_FILE；请先安装 ROS 2 Jazzy，再运行本脚本。")
    local name value
    for name in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH LD_LIBRARY_PATH PYTHONPATH; do
        value="${!name:-}"
        if [[ "$value" =~ /opt/ros/([^/:]+) ]]; then
            # Inspect every ROS installation path, not just the first match.
            local part
            local -a parts
            IFS=: read -r -a parts <<< "$value"
            for part in "${parts[@]}"; do
                if [[ "$part" =~ /opt/ros/([^/]+) && "${BASH_REMATCH[1]}" != jazzy ]]; then
                    issues+=("$name 混入其他 ROS 路径：$part。请清理混用环境。")
                fi
            done
        fi
    done
    if ((${#issues[@]})); then
        for value in "${issues[@]}"; do problem "$value"; done
        fatal 2 '系统或 ROS 版本不符合要求，已退出；没有安装任何依赖。'
    fi
    ok 'Ubuntu 版本及 ROS 安装路径符合要求'
}

load_ros() {
    # ROS setup files may reference unset variables.
    set +u
    if source "$ROS_SETUP_FILE"; then
        set -u
    else
        set -u
        fatal 2 "加载 $ROS_SETUP_FILE 失败"
    fi
    [[ "${ROS_DISTRO:-}" == jazzy && "${ROS_VERSION:-}" == 2 ]] || fatal 2 '加载后仍不是 ROS 2 Jazzy'
    ok '已加载 ROS 2 Jazzy 环境'
}

find_missing_packages() {
    local packages=(
        build-essential cmake git pkg-config ca-certificates
        python3-colcon-common-extensions python3-rosdep python3-pytest python3-numpy python3-opencv
        libeigen3-dev libopencv-dev libpcl-dev libboost-serialization-dev libssl-dev
        libglew-dev libepoxy-dev libgl1-mesa-dev libegl1-mesa-dev libx11-dev libwayland-dev libxkbcommon-dev
        ros-jazzy-ament-cmake ros-jazzy-ament-index-cpp ros-jazzy-ament-index-python
        ros-jazzy-rclcpp ros-jazzy-rclpy ros-jazzy-std-msgs ros-jazzy-sensor-msgs
        ros-jazzy-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-rosgraph-msgs
        ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-message-filters
        ros-jazzy-pcl-conversions ros-jazzy-tf2-ros ros-jazzy-ament-cmake-pytest
        ros-jazzy-launch-ros ros-jazzy-ros2launch ros-jazzy-ros2cli
    )
    local package status
    MISSING_PACKAGES=()
    for package in "${packages[@]}"; do
        status="$(dpkg-query -W -f='${Status}' "$package" 2>/dev/null || true)"
        [[ "$status" == 'install ok installed' ]] || MISSING_PACKAGES+=("$package")
    done
    if ((${#MISSING_PACKAGES[@]})); then
        problem "缺少 ${#MISSING_PACKAGES[@]} 个系统/ROS 依赖："
        printf '  %s\n' "${MISSING_PACKAGES[@]}" >&2
    else
        ok '系统、ROS、Python 及 Pangolin 构建依赖包已安装'
    fi
}

run_privileged() {
    if ((EUID == 0)); then "$@"; else sudo "$@"; fi
}

install_missing_packages() {
    ((${#MISSING_PACKAGES[@]})) || return 0
    if ((CHECK_ONLY)); then
        return 3
    fi
    if ((EUID != 0)); then
        command -v sudo >/dev/null || fatal 3 '缺少 sudo，请由管理员安装列出的包'
        sudo -v || fatal 3 '未取得 sudo 权限，未能安装依赖'
    fi
    command -v apt-get >/dev/null || fatal 3 '没有找到 apt-get'
    info "自动安装依赖；日志：$LOG_DIR/apt.log"
    if ! run_privileged apt-get update >"$LOG_DIR/apt.log" 2>&1; then
        tail -n 25 "$LOG_DIR/apt.log" >&2
        fatal 3 'apt 更新失败，请检查网络、软件源及 apt/dpkg 锁；不会强制解锁'
    fi
    if ! run_privileged apt-get install -y --no-remove "${MISSING_PACKAGES[@]}" >>"$LOG_DIR/apt.log" 2>&1; then
        tail -n 25 "$LOG_DIR/apt.log" >&2
        fatal 3 '依赖安装失败；请检查日志和 Jazzy 软件源后重新运行'
    fi
    find_missing_packages
    ((${#MISSING_PACKAGES[@]} == 0)) || fatal 3 '安装后仍有依赖缺失'
}

write_pangolin_probe() {
    mkdir -p "$LOG_DIR/pangolin-probe"
    cat >"$LOG_DIR/pangolin-probe/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(pangolin_environment_probe LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Pangolin CONFIG REQUIRED)
message(STATUS "Pangolin config: ${Pangolin_DIR}; version: ${Pangolin_VERSION}")
file(WRITE "${CMAKE_BINARY_DIR}/pangolin-dir.txt" "${Pangolin_DIR}")
add_executable(pangolin_probe main.cpp)
target_include_directories(pangolin_probe PRIVATE ${Pangolin_INCLUDE_DIRS})
target_link_libraries(pangolin_probe PRIVATE ${Pangolin_LIBRARIES})
CMAKE
    cat >"$LOG_DIR/pangolin-probe/main.cpp" <<'CPP'
#include <pangolin/pangolin.h>
#include <pangolin/utils/uri.h>
#include <filesystem>
#include <iostream>
int main() {
    // Both calls resolve library symbols; no display/context is created.
    pangolin::OpenGlMatrix matrix;
    matrix.SetIdentity();
    const auto uri = pangolin::ParseUri("file:///tmp/orbslam3-env-probe");
    std::cout << "Pangolin link/load OK; C++17 path: " << std::filesystem::path(".") << '\n';
    return matrix.m[0] == 1.0 && uri.scheme == "file" ? 0 : 1;
}
CPP
}

probe_pangolin() {
    local attempt="$1"
    local build="$LOG_DIR/pangolin-probe-$attempt"
    local log="$LOG_DIR/pangolin-$attempt.log"
    local directory
    local -a cmake_options=()
    # Explicitly select a repaired/custom installation even if Pangolin_DIR in
    # the caller's environment still points to an older broken installation.
    for directory in "$PANGOLIN_PREFIX/lib/cmake/Pangolin" "$PANGOLIN_PREFIX/lib64/cmake/Pangolin" \
                     "$PANGOLIN_PREFIX"/lib/*/cmake/Pangolin; do
        if [[ -f "$directory/PangolinConfig.cmake" ]]; then
            cmake_options+=("-DPangolin_DIR=$directory")
            break
        fi
    done
    if ! cmake -S "$LOG_DIR/pangolin-probe" -B "$build" -DCMAKE_BUILD_TYPE=Release \
        "${cmake_options[@]}" >"$log" 2>&1; then return 1; fi
    if ! cmake --build "$build" --parallel "$BUILD_JOBS" >>"$log" 2>&1; then return 1; fi
    if ! "$build/pangolin_probe" >>"$log" 2>&1; then return 1; fi
    PANGOLIN_CONFIG_DIR="$(cat "$build/pangolin-dir.txt")"
    ok "Pangolin 编译、链接及动态库加载通过：$PANGOLIN_CONFIG_DIR"
}

ensure_pangolin() {
    write_pangolin_probe
    if probe_pangolin initial; then return 0; fi
    problem "Pangolin 不可用（未安装、CMake 配置、编译或动态库加载失败）；日志：$LOG_DIR/pangolin-initial.log"
    tail -n 15 "$LOG_DIR/pangolin-initial.log" >&2
    if ((CHECK_ONLY)); then return 4; fi
    info "尝试构建安装 Pangolin v0.9.2 到 $PANGOLIN_PREFIX；日志：$LOG_DIR/pangolin-install.log"
    if ! CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS" bash "$SCRIPT_DIR/install_pangolin.sh" "$PANGOLIN_PREFIX" \
        >"$LOG_DIR/pangolin-install.log" 2>&1; then
        tail -n 25 "$LOG_DIR/pangolin-install.log" >&2
        fatal 4 'Pangolin 构建安装失败；请检查网络、编译日志和安装目录权限'
    fi
    if ! probe_pangolin repaired; then
        tail -n 25 "$LOG_DIR/pangolin-repaired.log" >&2
        fatal 4 'Pangolin 安装后仍无法编译/链接/运行探针'
    fi
}

check_python_and_tools() {
    local cmd problems=0
    for cmd in cmake c++ make git colcon ros2; do
        if ! command -v "$cmd" >/dev/null; then problem "PATH 中找不到 $cmd"; problems=1; fi
    done
    if ! /usr/bin/python3 -c 'import rclpy, cv_bridge, ament_index_python, cv2, numpy, pytest' \
        >"$LOG_DIR/python.log" 2>&1; then
        problem "系统 Python 无法导入 ROS/测试依赖；日志：$LOG_DIR/python.log"
        cat "$LOG_DIR/python.log" >&2
        problems=1
    fi
    if [[ -n "${CONDA_PREFIX:-}" || -n "${VIRTUAL_ENV:-}" ]]; then
        problem '检测到 Conda/virtualenv。请先退出虚拟环境，以免 ROS 和系统 Python ABI 混用。'
        problems=1
    fi
    ((problems == 0))
}

check_project_configuration() {
    info '验证项目全部 CMake 依赖（包含测试依赖，不编译 SLAM）'
    if ! cmake -S "$PROJECT_ROOT" -B "$LOG_DIR/project-configure" \
        -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
        -DPython3_EXECUTABLE=/usr/bin/python3 -DPangolin_DIR="$PANGOLIN_CONFIG_DIR" \
        >"$LOG_DIR/project-configure.log" 2>&1; then
        tail -n 35 "$LOG_DIR/project-configure.log" >&2
        problem "项目 CMake 配置失败；完整日志：$LOG_DIR/project-configure.log"
        return 5
    fi
    ok '项目全部 CMake 依赖解析通过'
}

write_build_environment() {
    local file="$LOG_DIR/build_env.sh"
    {
        printf '# Generated by check_build_env.sh; source this before colcon build.\n'
        printf 'source %q\n' "$ROS_SETUP_FILE"
        printf 'export CMAKE_PREFIX_PATH=%q"${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"\n' "$PANGOLIN_PREFIX"
        printf 'export Pangolin_DIR=%q\n' "$PANGOLIN_CONFIG_DIR"
        printf 'export CMAKE_BUILD_PARALLEL_LEVEL=%q\n' "$BUILD_JOBS"
        printf 'export MAKEFLAGS=%q\n' "-j$BUILD_JOBS -l$BUILD_JOBS"
    } >"$file"
    printf '\n[可以编译] 所有环境检查通过。\n'
    printf '日志目录：%s\n' "$LOG_DIR"
    printf '先在当前终端执行（脚本无法修改父终端环境）：\n  source %q\n' "$file"
    printf '然后在工作空间根目录执行：\n'
    printf '  colcon build --packages-select orbslam3_dense_ros2 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON -DPython3_EXECUTABLE=/usr/bin/python3 -DPangolin_DIR="$Pangolin_DIR"\n'
}

main() {
    parse_args "$@"
    check_platform  # No apt or source build is allowed before this gate.
    load_ros
    if [[ -z "$LOG_DIR" ]]; then
        LOG_DIR="${XDG_CACHE_HOME:-$HOME/.cache}/orbslam3_dense_ros2/environment/$(date +%Y%m%d-%H%M%S)-$$"
    fi
    mkdir -p "$LOG_DIR"
    LOG_DIR="$(cd -- "$LOG_DIR" && pwd)"
    info "检查日志：$LOG_DIR"
    export CMAKE_PREFIX_PATH="$PANGOLIN_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    find_missing_packages
    local missing=0 failed=0 pangolin_failed=0
    if ! install_missing_packages; then missing=1; fi
    # New ROS dependencies may contribute environment hooks.
    load_ros
    export CMAKE_PREFIX_PATH="$PANGOLIN_PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
    if ! check_python_and_tools; then failed=1; fi
    if ! command -v cmake >/dev/null || ! command -v c++ >/dev/null || ! command -v make >/dev/null; then
        fatal 3 '缺少构建工具，无法继续验证 Pangolin；请运行默认自动修复模式'
    fi
    if ! ensure_pangolin; then pangolin_failed=1; fi
    if ((missing)); then fatal 3 '检查完成：存在缺失依赖；不可以编译。去掉 --check-only 可自动安装'; fi
    ((pangolin_failed == 0)) || fatal 4 "Pangolin 检查未通过，不可以编译；日志：$LOG_DIR"
    ((failed == 0)) || fatal 5 '检查完成：上述环境检查未通过，不可以编译'
    check_project_configuration || fatal 5 '不可以编译；请根据上述配置错误修复环境'
    write_build_environment
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then main "$@"; fi
