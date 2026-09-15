#!/usr/bin/env bash
# ROS 2 の rust_packages インデックスを復元する。
#
# colcon-ros-cargo は .cargo/config.toml の [patch.crates-io.*] を生成する際、
#   <prefix>/share/ament_index/resource_index/rust_packages/
# を列挙して対象パッケージを決める（colcon_ros_cargo/task/ament_cargo/build.py の
# find_installed_cargo_packages）。このディレクトリが無いと patch が一件も
# 生成されず、sensor_msgs 等が crates.io から解決されてビルドが失敗する。
#
# 一方、このインデックスを収録している deb は存在しない（dpkg -S で所有者なし）。
# そのため apt の更新で消えることがあり、実際 2026-09-15 の自動更新で消失した。
# Rust バインディング本体（share/<pkg>/rust/）は deb に含まれているので、
# その実在を根拠にインデックスを再生成する。
#
# 冪等なので、colcon build の前に何度実行しても構わない。

set -euo pipefail

ROS_PREFIX="${ROS_PREFIX:-/opt/ros/jazzy}"
INDEX_DIR="$ROS_PREFIX/share/ament_index/resource_index/rust_packages"

if [ ! -d "$ROS_PREFIX/share" ]; then
  echo "エラー: ROS が見つかりません: $ROS_PREFIX" >&2
  echo "別の場所にある場合は ROS_PREFIX=/path/to/ros $0 のように指定してください。" >&2
  exit 1
fi

# share/<pkg>/rust/Cargo.toml を持つものだけを対象にする。
# rust/ があっても Cargo.toml が無ければ patch 先として機能しないため。
mapfile -t packages < <(
  for dir in "$ROS_PREFIX"/share/*/rust; do
    [ -f "$dir/Cargo.toml" ] || continue
    basename "$(dirname "$dir")"
  done | sort
)

if [ "${#packages[@]}" -eq 0 ]; then
  echo "エラー: $ROS_PREFIX/share/*/rust が一つも見つかりません。" >&2
  echo "ros-jazzy-rosidl-generator-rs と各メッセージパッケージを apt で導入してください。" >&2
  exit 1
fi

SUDO=""
if [ ! -w "$(dirname "$INDEX_DIR")" ]; then
  SUDO="sudo"
fi

$SUDO mkdir -p "$INDEX_DIR"

created=0
for pkg in "${packages[@]}"; do
  if [ ! -e "$INDEX_DIR/$pkg" ]; then
    $SUDO touch "$INDEX_DIR/$pkg"
    created=$((created + 1))
  fi
done

echo "rust_packages インデックスを復元しました: $INDEX_DIR"
echo "  対象 ${#packages[@]} 件 / 新規作成 ${created} 件"

if [ "$created" -eq 0 ]; then
  echo "  (すべて既存。変更はありません)"
fi
