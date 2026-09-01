# airtable_raspi_ws

エアテーブルロボット用の ROS 2 ワークスペースです。ハードウェア層
（`airtable_hardware`）は Rust（[rclrs](https://github.com/ros2-rust/ros2_rust)）
で実装されており、`ament_cargo` ビルドタイプを使用します。

> **ROS 2 Jazzy移行について**
> 本ワークスペースは ROS 2 Humble (Ubuntu 22.04) から ROS 2 Jazzy
> (Ubuntu 24.04) への移行を`jazzy`ブランチで試行中です。mainブランチは
> 引き続きHumble前提のままです。移行状況は
> [ROS 2 Jazzy移行（作業中）](#ros-2-jazzy移行作業中) を参照してください。

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

### 4. ビルド

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

### 5. 動作確認

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

## ROS 2 Jazzy移行（作業中）

### 結論: Humble時代より大幅に簡素化されました

Humbleでは `ros2_rust` をソースからワークスペースに取り込む必要がありましたが、
Jazzyでは apt と crates.io だけで完結します。移行前の `Cargo.lock` には
`examples_rclrs_*`（ros2_rustのサンプルパッケージ）が含まれ、
`rosidl_runtime_rs` がパス依存になっていましたが、いずれも不要になりました。

**Jazzyで判明した改善点**
- `ros-jazzy-rosidl-generator-rs` が apt で提供される（0.4.12）
- **標準メッセージパッケージのdeb自体にRustバインディングが同梱**されている。
  `ros-jazzy-builtin-interfaces` などが `/opt/ros/jazzy/share/<pkg>/rust/` を
  インストールするため、`colcon-ros-cargo` が生成する `.cargo/config.toml` の
  `[patch.crates-io.*]` は39/39すべて実体に解決される
- `rosidl_runtime_rs` (0.6.1) と `rclrs` (0.7.0) は crates.io から取得

**確認済み**
- `rm -rf build install log .cargo` で完全にクリーンにした状態から、
  `colcon build` ワンショットで全11パッケージのビルド成功。
  `airtable_interfaces` → `airtable_hardware` の依存順も自動解決される
- 実行時の動作確認: ROSメッセージ受信 → デバイス層 → ESCドライバ →
  PWM出力までの全レイヤーと、`names`/`thrust` の長さ不一致に対する
  バリデーションが正しく機能
- `pyproject.toml` の `requires-python` を `>=3.12,<3.13` に更新
  （Jazzy標準のPython 3.12に追従）
- `colcon-common-extensions` を dev 依存に追加。これが無いと venv 側の
  colcon に `--packages-select` などの拡張が入らない

### `.cargo/config.toml` が生成されない問題について

Humble環境では、`colcon-ros-cargo` が `.cargo/config.toml` を正しく生成せず、
`airtable_interfaces` のパス依存が解決できない問題が発生していました。
**この問題の直接の原因は `src/airtable_interfaces/COLCON_IGNORE` です。**

`airtable_interfaces` の patch エントリには鶏と卵の依存関係があります。
patch先の `install/airtable_interfaces/share/airtable_interfaces/rust` は
`airtable_interfaces` をビルドして初めて生成されるため、`colcon-ros-cargo` が
`airtable_hardware` のビルド時点でそれを発見できる必要があります。
`COLCON_IGNORE` があると `airtable_interfaces` が colcon から完全に不可視になり、

1. `install/` に生成物が出ない
2. → `colcon-ros-cargo` が patch エントリを作れない
3. → `Cargo.toml` の path 依存が解決できない

という連鎖が起きます。そのため `COLCON_IGNORE` は**削除しました**（再追加しないこと）。

検証として `.cargo/` ごと削除した状態から `colcon build` を実行し、
以下を確認済みです。

- `.cargo/config.toml` が新規に自動生成される
- `[patch.crates-io.airtable_interfaces]` が
  `install/airtable_interfaces/share/airtable_interfaces/rust` を正しく指す
- patch対象39件すべてが実在するパスに解決される（欠損ゼロ）

なお、この問題自体はJazzy固有ではなくHumbleでも `COLCON_IGNORE` を外せば
解消した可能性が高いですが、Jazzyではros2_rustのソースビルドが不要になった分、
構成がシンプルで再現性が高くなっています。

**未対応**
- Raspberry Pi実機での検証（本作業はx86_64のWSL上のみ）
- `io/pwm.rs` は `DummyPwm` のスタブ実装のまま。実機のGPIO/PWM
  ドライバは未実装
- `airtable_bringup` / `airtable_controller` / `airtable_description` /
  `airtable_estimator` / `airtable_ground` / `airtable_safety` /
  `airtable_sim` / `airtable_teleop` / `airtable_tracking` は
  依存宣言のみのスケルトンで、実装が入った時点で改めてAPI互換の確認が必要
