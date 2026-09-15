# airtable_raspi_ws

エアテーブルロボット用の ROS 2 ワークスペースです。ハードウェア層
（`airtable_hardware`）は Rust（[rclrs](https://github.com/ros2-rust/ros2_rust)）
で実装されており、`ament_cargo` ビルドタイプを使用します。

## セットアップ（Jazzy）

### 1. ROS 2 Jazzy と依存パッケージ

```bash
sudo apt-get install -y ros-jazzy-desktop
sudo apt-get install -y ros-jazzy-rosidl-generator-rs ros-jazzy-test-msgs
sudo rosdep init && rosdep update
```

- `rosidl-generator-rs`: メッセージ定義からRustバインディングを生成
- `test-msgs`: `rclrs` のビルドスクリプトがリンクフラグを無条件に出力するため、
  実際には使用しなくてもリンク時に必要

### 2. Rustツールチェーン

```bash
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --profile minimal
export PATH="$HOME/.cargo/bin:$PATH"
cargo install cargo-ament-build
```

`airtable_hardware/Cargo.toml` は `edition = "2024"` を使用するため、
Rust 1.85 以降が必要です。

### 3. colcon拡張（uv管理）

`colcon-cargo` / `colcon-ros-cargo` が `ament_cargo` ビルドタイプを提供します。

```bash
uv sync
```

### 4. `rust_packages` インデックスの確認

`colcon-ros-cargo` は `/opt/ros/jazzy/share/ament_index/resource_index/rust_packages/`
を見て `.cargo/config.toml` の patch 対象を決めますが、このインデックスを
収録した deb は存在せず、apt の更新で消えることがあります。消えたまま
ビルドすると `sensor_msgs` 等が crates.io から解決され失敗します
（詳細は [0002. `rust_packages` インデックスを自前で復元する](docs/adr/0002-rust-packages-index-loss.md)）。

```bash
./scripts/restore_rust_packages_index.sh
```

冪等なので、ビルド前に毎回実行しても問題ありません。

### 5. ビルド

**ROSの`PYTHONPATH`を維持したまま**、venv側のcolconを使用します。
`ament_package` などのROS Pythonモジュールは
`/opt/ros/jazzy/lib/python3.12/site-packages` にあるため、
`env -u PYTHONPATH` を付けるとビルドが失敗します。

```bash
source /opt/ros/jazzy/setup.bash
export PATH="$HOME/.cargo/bin:$PATH"
.venv/bin/colcon build
source install/setup.bash
```

### 6. 動作確認

```bash
ros2 run airtable_hardware airtable_hardware
```

別シェルから:

```bash
ros2 topic pub --once /actuators/thrusters/setpoint airtable_interfaces/msg/ThrusterSetpoint \
  '{stamp: {sec: 0, nanosec: 0}, names: ["thruster_1", "thruster_2", "thruster_3"], thrust: [0.25, 0.5, 0.0]}'
```

ダミーハードウェアは `thruster_1` / `thruster_2` / `thruster_3`（各最大0.5N）を
持ちます。ESCドライバとPWM出力は現状スタブ実装（`DummyPwm`）で、
標準出力にログを出すだけです。

## パッケージ構成

| パッケージ | 言語 | 役割 | 状況 |
|---|---|---|---|
| `airtable_interfaces` | IDL | 独自メッセージ定義（msg 3種） | 実装済み |
| `airtable_hardware` | Rust (rclrs) | 実機 I/O。ESC / ステッパ / バルブ / IMU | 実装済み（I/O層はスタブ） |
| `airtable_sim` | Rust (rclrs) + MuJoCo | 物理シミュレーション | 実装済み |
| `airtable_controller` | C++ | 手動ミキサと制御配分 | 実装済み |
| `airtable_teleop` | C++ | ゲームパッド入力 → 手動指令 | 実装済み |
| `airtable_bringup` | Python launch | 起動構成（実機 / シミュレーション） | 実装済み |
| `airtable_description` | URDF/xacro | ロボットモデル | URDF と display launch のみ |
| `airtable_estimator` | C++ | （未定義） | 未実装 |
| `airtable_ground` | C++ | （未定義） | 未実装 |
| `airtable_safety` | C++ | （未定義） | 未実装 |
| `airtable_tracking` | C++ | （未定義） | 未実装 |

未実装の4パッケージは `CMakeLists.txt` と `package.xml` のみで、`package.xml` の
`<description>` も `TODO` のままです。役割を決めた時点で追記してください。

### データフロー

```
/joy (sensor_msgs/Joy)
  └─ airtable_teleop
       └─ /commands/manual/motion (ManualMotionCommand)
            └─ airtable_controller
                 ├─ /actuators/thrusters/pulse    (ThrusterPulseCommand)
                 └─ /actuators/thrusters/setpoint (ThrusterSetpoint)
                      └─ airtable_sim / airtable_hardware
```

`airtable_controller` は `/wrench_cmd` (geometry_msgs/WrenchStamped) も購読し、
制御配分を通して `setpoint` を出しますが、この上流ノードは未実装です。
`airtable_hardware` はさらに `actuators/xy_table/trajectory` を購読し、
`imu/data_raw` を publish します。

### 起動

```bash
ros2 launch airtable_bringup simulation_control.launch.py   # シミュレーション
ros2 launch airtable_bringup manual_control.launch.py       # 実機
```

## 設計判断の記録

調査して結論の出たハマりどころは [docs/adr/](docs/adr/) に1件1ファイルで
残しています。

- [0001. `COLCON_IGNORE` を削除する](docs/adr/0001-colcon-ignore-removal.md)
  — これがあると `.cargo/config.toml` の patch 生成が連鎖的に失敗する
- [0002. `rust_packages` インデックスを自前で復元する](docs/adr/0002-rust-packages-index-loss.md)
  — apt更新で消え、`sensor_msgs` が crates.io から解決されてビルドが落ちる
- [0003. ROS 2 Humble から Jazzy へ移行する](docs/adr/0003-jazzy-migration.md)
  — ros2_rust のソース取り込みが不要になり構成が簡素化

## 進行中の作業と残課題

[docs/WORKLOG.md](docs/WORKLOG.md) を参照してください。

## AIエージェント向けの前提

[CLAUDE.md](CLAUDE.md) にビルド手順の厳守事項、探索しなくてよいディレクトリ、
実装上の約束をまとめています。
