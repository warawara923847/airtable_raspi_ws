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

自己位置推定・マーカートラッキングの依存パッケージ:

```bash
sudo apt-get update
sudo apt-get install --no-remove --no-install-recommends -y \
  ros-jazzy-realsense2-camera ros-jazzy-realsense2-description \
  ros-jazzy-aruco-ros ros-jazzy-robot-localization \
  ros-jazzy-rclpy ros-jazzy-tf2-ros ros-jazzy-tf-transformations \
  ros-jazzy-geometry-msgs ros-jazzy-sensor-msgs ros-jazzy-nav-msgs \
  ros-jazzy-visualization-msgs python3-numpy
```

RealSense SDK は `realsense2_camera` の apt 依存で導入します。
旧 `~/Documents/airtable_ws/src/marker_pkg` の検出処理は `aruco_ros`、
平面座標変換は Python / NumPy / `tf_transformations` を使っています。
この経路の流用に追加の pip 版 OpenCV は不要です。
依存関係は `airtable_tracking` / `airtable_estimator` の `package.xml` にも
登録しています。トラッキング本体とlaunchは移植済みで、EKF設定は未実装です。

導入後の確認:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg executables realsense2_camera
ros2 pkg executables aruco_ros
ros2 pkg executables robot_localization
/usr/bin/python3 -c 'import rclpy, numpy, tf_transformations; from tf2_ros import TransformBroadcaster; from geometry_msgs.msg import PoseWithCovarianceStamped; from visualization_msgs.msg import Marker; print("tracking dependencies: OK")'
```

流用元の構成と移植時の修正点は
[0005. 旧マーカートラッキング実装の依存関係を準備する](docs/adr/0005-tracking-dependencies.md)
を参照してください。

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
| `airtable_sim` | Rust (rclrs) + MuJoCo | 物理・仮想カメラシミュレーション | 描画画像からのマーカー検出まで検証済み |
| `airtable_controller` | C++ | 手動ミキサと制御配分 | 実装済み |
| `airtable_teleop` | C++ | ゲームパッド入力 → 手動指令 | 実装済み |
| `airtable_bringup` | Python launch | 起動構成（実機 / シミュレーション） | 実装済み |
| `airtable_description` | URDF/xacro | ロボットモデル | URDF と display launch のみ |
| `airtable_estimator` | C++ | 外部Pose / IMUによる状態推定 | 依存定義のみ |
| `airtable_ground` | C++ | （未定義） | 未実装 |
| `airtable_safety` | C++ | （未定義） | 未実装 |
| `airtable_tracking` | Python / ament_cmake | マーカーから機体Pose観測を生成 | MuJoCo描画画像で検証済み、実機は未検証 |

`airtable_estimator` / `airtable_ground` / `airtable_safety` の実行ノードは未実装で、
ソース構成は `CMakeLists.txt` と `package.xml` のみです。`airtable_estimator` は
役割と実行時依存を定義済みです。`airtable_ground` / `airtable_safety` は役割も未定義です。

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

### マーカートラッキング

`airtable_tracking` は旧 `airtable_ws` と同様に、外部固定カメラで固定マーカー4個と
機体上の可動マーカーを観測します。検出は `aruco_ros/single`、座標変換は
Pythonノードで行います。画像入力・検出・機体Pose変換を独立に起動できます。

```bash
source /opt/ros/jazzy/setup.bash
.venv/bin/colcon build --symlink-install --packages-select airtable_tracking \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch airtable_tracking tracking.launch.py start_camera:=true
```

起動前に `src/airtable_tracking/config/tracking.yaml` を実環境に合わせてください。

- `fixed_marker_ids` と `fixed_marker_positions`: 同じ順序で、テーブル座標系における
  各マーカー中心のXYZ座標[m]を指定します。既定値は1 m × 2 mの配置例です。
  既知の配置と観測した4点を剛体位置合わせし、カメラ位置・姿勢を求めます。
  カメラ位置の事前入力は不要です。テーブル原点・軸は、この既知の配置で定まります。
- `base_to_marker_translation` / `base_to_marker_rpy`: `T_base_marker`、つまり
  base_linkから見たマーカー原点の位置[m]と姿勢[rad]です。
  `aruco_ros` が出力するマーカー座標軸に合わせて測定します。
- `base_height`: 平面運動として出力するbase_linkの高さ[m]です。
  シミュレータの現モデルと合わせる場合は0.01 m、マーカー高さとは別の値です。
- `pose_covariance_diagonal`: テーブル座標系での観測分散です。実機で調整してください。

別の設定ファイルは `params_file:=/absolute/path/tracking.yaml` で指定できます。
物理カメラをすでに起動している場合は `start_camera:=false`（既定値）、仮想の
`PoseStamped` を直接入力する場合はさらに `start_detectors:=false` を指定します。
`namespace` の既定値は `robot_1` です。入力と出力は以下になります。

```text
/camera/camera/infra1/image_rect_raw + camera_info
  → aruCo検出器 × マーカー数
  → /robot_1/tracking/aruco_single_<ID>/pose (PoseStamped、カメラ座標)
  → airtable_tracking
  → /robot_1/measurements/external_pose (PoseWithCovarianceStamped、table_1座標のbase_link)
```

画像トピックは `image_topic` / `camera_info_topic`、入力座標系は `camera_frame` で
変更できます。画像はrectified画像を前提とします。出力は元の画像の観測時刻を保持し、
位置・姿勢を変換して取り付け位置を補正した後、平面Poseに射影します。
欠測中は新しいPoseを出さず、重複・順序逆転した可動マーカー観測も出力しません。
固定点は同一フレーム・指定時間幅内で全点が揃い、配置残差が閾値以下の場合だけ採用します。
初期化後はカメラを固定する前提です。カメラを動かした場合や時刻を巻き戻すbag再生では
ノードを再起動して再校正してください。機体TF・速度推定・EKFはこのノードでは配信しません。

検証コマンド:

```bash
.venv/bin/colcon test --packages-select airtable_tracking
.venv/bin/colcon test-result --test-result-base build/airtable_tracking/test_results --verbose
```

仮想Poseでの座標変換検証と画像による検出検証の違いは
[0006. トラッキングの移植と段階的なシミュレーション検証](docs/adr/0006-tracking-migration.md)
を参照してください。

### シミュレータのカメラでトラッキングする

MuJoCoに固定マーカーID 10〜13、機体マーカーID 50と固定カメラを配置しています。
`aruco_ros` に渡すのは描画画像とCameraInfoで、検出Poseを真値から生成する処理はありません。

変更したパッケージのビルド:

```bash
source /opt/ros/jazzy/setup.bash
./scripts/restore_rust_packages_index.sh
.venv/bin/colcon build --symlink-install --packages-select airtable_tracking airtable_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
.venv/bin/colcon build --packages-select airtable_sim
source install/setup.bash
```

操縦系と一緒に起動:

```bash
ros2 launch airtable_bringup simulation_control.launch.py tracking:=true
```

カメラ・トラッキングのみを起動する場合:

```bash
ros2 launch airtable_bringup simulation_tracking.launch.py
# ウィンドウなし。画像描画にはEGL / OpenGLが必要です。
ros2 launch airtable_bringup simulation_tracking.launch.py viewer:=false
```

上記のlaunchはどれか1つを起動してください。操縦launchの `tracking:=true` は、
通常のsimノードを置き換えてカメラ付きsimノードを1つだけ起動します。
従来どおり `tracking:=false`（既定値）なら画像配信を無効にして起動します。

| トピック | 内容 |
|---|---|
| `/sim/camera/infra1/image_rect_raw` | 1280×960、mono8、既定30 Hzの描画画像 |
| `/sim/camera/infra1/camera_info` | 描画投影と一致する内部パラメータ、歪みなし |
| `/robot_1/measurements/external_pose` | 画像検出によるtable_1基準の機体Pose観測 |
| `/robot_1/sim/ground_truth` | 物理計算による機体真値 |
| `/clock` | シミュレーション時刻、100 Hz |

カメラは `src/airtable_sim/mujoco/airtable.xml` の `tracking_camera` で、
テーブル中心の高さ2.2 m、垂直画角55度です。位置・姿勢・画角を変える場合は
ここを編集してください。解像度を変える場合は `<global offwidth/offheight>` も
描画解像度以上に設定します。内部パラメータはモデルの解像度と画角から計算します。
これはD435のIR1相当の理想単眼モデルで、レンズ歪み・IR投光器・露光・ブレは再現しません。
`camera_fps:=15.0` などで配信頻度を変更できます。描画や検出の負荷が高いと欠落が生じます。

固定マーカー中心と取り付け位置は
`src/airtable_tracking/config/simulation_tracking.yaml` にモデルと合わせて設定しています。
マーカーの黒枠の外寸は0.10 mで、白い余白を含むカード全体は0.15 mです。
マーカー模様は `DICT_ARUCO_ORIGINAL`。再生成は以下で行えます。

```bash
/usr/bin/python3 scripts/generate_sim_markers.py
```

真値比較の再現用スクリプトはシミュレーションを自動起動・停止します。
既存のsimを停止した状態で実行してください。`--exercise-motion` は初期化後に
thruster_1を0.25秒噴射し、移動・回転中のXY / yaw誤差を評価します。
画像・CameraInfo・推定Pose・真値が同じ時刻のサンプルだけを比較します。

```bash
source install/setup.bash
/usr/bin/python3 scripts/check_sim_tracking.py --exercise-motion
# 操縦系とウィンドウ表示も含めて検証する場合:
/usr/bin/python3 scripts/check_sim_tracking.py --exercise-motion --control --viewer
```

検証では操縦系・ウィンドウ併用時の129サンプルで、XY RMSE 9.6 mm、最大18.9 mm、
yaw RMSE 0.24度、最大0.48度でした。これは現在の理想画像・配置に対する値です。
比較結果は標準出力、カメラ画像は既定で `/tmp/airtable_sim_camera.png` に保存します。
詳細は [0007. MuJoCo画像によるArUcoトラッキング](docs/adr/0007-sim-camera-tracking.md) を参照。

### RVizで観測Pose・カメラを表示する

シミュレータとトラッキングが起動済みなら、別の端末でRVizだけを追加できます。

```bash
source install/setup.bash
ros2 launch airtable_bringup rviz.launch.py
```

操縦系・シミュレータ・RVizをまとめて起動する場合は以下です。

```bash
ros2 launch airtable_bringup simulation_control.launch.py tracking:=true rviz:=true
# MuJoCoのウィンドウを省略し、RVizを表示する場合は viewer:=false を追加
```

操縦系が不要なら `simulation_tracking.launch.py rviz:=true` でも起動できます。
既に起動中のsimへ接続する場合は `rviz.launch.py` を使い、simを二重起動しないでください。
設定ファイルは `src/airtable_bringup/rviz/tracking.rviz` です。
Fixed Frameは `table_1`、RVizの `use_sim_time` はtrueに設定しています。

| 表示 | 配信元 | RViz Display |
|---|---|---|
| ArUco観測による機体Pose（緑） | `/robot_1/measurements/external_pose` (`PoseWithCovarianceStamped`) | PoseWithCovariance |
| 機体真値（青） | `/robot_1/sim/ground_truth` (`Odometry`) | Odometry |
| カメラ位置・姿勢の座標軸 | `/tf_static` の `sim_camera_infra1_optical_frame` | Axes / TF |
| カメラ画像 | `/sim/camera/infra1/image_rect_raw` (`Image`) | Image |
| 内部パラメータ | `/sim/camera/infra1/camera_info` (`CameraInfo`) | Camera表示を追加する場合に使用 |

カメラは `world → sim_camera_infra1_optical_frame` の静的TFとして配信され、
テーブルとの関係は `world → table_1` を通して取得できます。
現在の `robot_1/base_link` のTFはシミュレータの真値です。
緑のPoseはArUco単独の観測結果で、IMUとのEKF融合はまだ行っていません。
Poseの共分散は未使用軸に大きな分散を設定しているため、RViz設定では共分散描画を無効にしています。
画像とカメラTFは `tracking:=true` または `simulation_tracking.launch.py` の起動時に配信されます。

手動設定の場合もFixed Frameを `table_1` にして、上表のDisplayと配信元を選択してください。
実機用にこのRViz起動を使う場合は `use_sim_time:=false` を指定し、トピックとフレームを
実機の構成に合わせて変更してください。

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
