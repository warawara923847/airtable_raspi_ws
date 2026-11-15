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
ros2 topic pub --once /actuators/thrusters/pulse airtable_interfaces/msg/ThrusterPulseCommand \
  '{names: ["thruster_1"], pulse_duration: {sec: 0, nanosec: 200000000}}'
```

ダミーハードウェアは `thruster_1`〜`thruster_4`（各公称0.5 N）を持ちます。
スラスタの実I/Oはスタブで、標準出力にログを出します。
上記はhardware単体起動時のトピックです。bringup経由では先頭に `/robot_1` を付けます。

## パッケージ構成

| パッケージ | 言語 | 役割 | 状況 |
|---|---|---|---|
| `airtable_interfaces` | IDL | 独自メッセージ定義（msg 3種） | 実装済み |
| `airtable_hardware` | Rust (rclrs) | 実機 I/O。ESC / ステッパ / バルブ / IMU | 実装済み（I/O層はスタブ） |
| `airtable_sim` | Rust (rclrs) + MuJoCo | 物理・仮想カメラシミュレーション | 描画画像からのマーカー検出まで検証済み |
| `airtable_controller` | C++ | 手動ミキサ・姿勢固定の軌道追従・制御配分・パルス変調 | シミュレーションで検証済み |
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
                 └─ actuators/thrusters/pulse (ThrusterPulseCommand)
                      └─ airtable_sim / airtable_hardware

状態Odometry → 姿勢固定の直線単振動 / 円軌道追従
  └─ wrench_cmd (WrenchStamped、機体座標)
       └─ airtable_controller の制御配分
            └─ actuators/thrusters/setpoint (ThrusterSetpoint)
                 └─ airtable_pulse_modulator
                      └─ actuators/thrusters/pulse → airtable_sim / airtable_hardware
```

bringupではteleop / controller / hardwareを `robot_1` 名前空間に揃えています。
`airtable_controller` は `wrench_cmd` (geometry_msgs/WrenchStamped) も購読し、
制御配分を通して `setpoint` を出します。軌道追従用launchでは追従制御と
推力→パルス変調も起動します。sim / hardwareは `pulse` を受信します。
`airtable_hardware` はさらに `actuators/xy_table/trajectory` を購読し、
`imu/data_raw` を publish します。

### 起動

```bash
ros2 launch airtable_bringup simulation_control.launch.py   # シミュレーション
ros2 launch airtable_bringup manual_control.launch.py       # 実機
```

### スラスタモデルと操作方向

MuJoCoと制御配分設定のスラスタ番号・位置・推力方向・上限を揃えています。
機体座標は **+Xが前、+Yが左、正yawが反時計回り** です。
手動操作もこの定義に変更したため、従来の前後左右・旋回指令とは運動方向が変わります。
推力方向は排気の向きではなく、機体に加わる力の向きです。

現在は本体0.20×0.20×0.02 m、質量1 kg、各スラスタ0.5 Nの公称モデルです。
実機の実測値ではありません。URDFの本体表示寸法も揃えていますが、
URDFの可動XYステージの物理挙動はMuJoCoでは未実装です。
配置と操作対応表は
[0009. スラスタモデルの整合](docs/adr/0009-thruster-model-consistency.md) を参照してください。

再検証は以下で行えます。統合検証は専用ドメイン（既定173）でsim / controllerを
自動起動・停止し、制御配分6方向、単独噴射4基、手動運動6方向を確認します。

```bash
/usr/bin/python3 scripts/check_sim_model.py --static-only
source install/setup.bash
/usr/bin/python3 scripts/check_sim_model.py
```

配分器は推力上下限付き最小二乗へ変更済みです。実現可能な力・トルクを再現し、
飽和時はモーメントアームで重み付けした誤差を最小化します。

### 姿勢固定で直線単振動・円軌道を追従する

設定の共通化・自動生成は行っていません。MuJoCoと `thrusters.yaml` に加え、
追従制御側の質量・慣性は `src/airtable_controller/config/trajectory.yaml` に設定します。
現在の1 kgモデルに合わせた値です。`check_sim_model.py --static-only` はこの質量・慣性も照合します。

```bash
source /opt/ros/jazzy/setup.bash
.venv/bin/colcon build --symlink-install --packages-select airtable_controller airtable_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
.venv/bin/colcon build --packages-select airtable_sim
source install/setup.bash

# 機体のyawを0 radに保ち、テーブルX方向に振幅0.10 m・周期20秒の単振動
ros2 launch airtable_bringup simulation_trajectory.launch.py trajectory:=line rviz:=true

# 上のlaunchを終了してから、中心(0.5, 1.0)、半径0.20 m・周期20秒の円軌道
ros2 launch airtable_bringup simulation_trajectory.launch.py trajectory:=circle radius:=0.20 rviz:=true
```

直線の中心は開始位置です。円の中心は既定でテーブル中央(0.5, 1.0)に固定し、
simの初期位置を円周の右端へ自動配置します。半径0.20 mなら初期位置は(0.7, 1.0)です。
円は半径を保ち、5秒で角速度を滑らかに立ち上げます。直線は従来どおり5秒で振幅を導入します。
機体姿勢と移動方向は独立で、円の接線方向へ旋回しません。
姿勢目標は `fixed_yaw:=0.6` [rad]、直線の方向は `direction:=1.57079632679` [rad]、
周期・振幅・半径は `period:=20.0 amplitude:=0.10 radius:=0.10` で変更できます。
省略した軌道引数は `trajectory.yaml` の値を使用します。
`viewer:=false` でMuJoCoウィンドウを省略できます。
RVizでは目標軌道・目標姿勢が緑、実際の軌跡・姿勢が青です。
手動操縦用やカメラ用のsim launchとは同時に起動しないでください。

**既定の制御入力はMuJoCoの真値** `/robot_1/sim/ground_truth` です。
このlaunchでは既定で仮想カメラとArUco検出器を起動しません。
画像観測も比較したい場合は `tracking:=true` を追加します。simを二重起動せず、
`/sim/camera/infra1/image_rect_raw` と `/robot_1/measurements/external_pose` を配信します。
このオプションでも制御入力は真値のままで、カメラ観測による速度推定・EKF融合は未接続です。
マーカーの形状・テクスチャは常にMJCFに残っています。機体マーカーを隠していた大きな
RW表示siteは非表示のgroup 3へ分離しました。ViewerでSite 3を有効にすると表示できます。

初期位置・円の中心は以下で指定できます。

```bash
ros2 launch airtable_bringup simulation_trajectory.launch.py \
  trajectory:=circle radius:=0.20 circle_center_x:=0.5 circle_center_y:=1.0 \
  initial_x:=0.7 initial_y:=1.0 initial_yaw:=0.0 rviz:=true
# カメラ画像とマーカー観測も並行して取得する場合は tracking:=true を追加
```

XYは対で指定してください。円の中心は `trajectory.yaml` の `trajectory.circle_center_m` でも
指定でき、`[]` にすると旧仕様の開始位置からの相対中心へ戻せます。
線形軌道で初期位置を省略した場合はMJCFの位置を保持します。
明示した初期位置が円周の右端と異なる場合は、5秒の接続区間で指定円へ移ります。
初期位置を動かしても半径は変わらないので、大きい円には `radius` を指定します。

配信周期は真値100 Hz、目標・制御50 Hz、実軌跡Path20 Hz、RViz描画設定30 fpsです。
実時間よりsimが遅く動く場合は、ROS時刻基準の制御・Pathの壁時計上の頻度も下がります。
RViz Odometry表示のPosition / Angle Toleranceは0に設定しています。
旧設定では既定の0.1 m / 0.1 radが使われ、姿勢固定・小半径の運動が飛び飛びに見えていました。
詳細は[0011. 追従の状態入力・表示周期と円の初期位置](docs/adr/0011-trajectory-observation-and-startup.md)。

| トピック / サービス | 内容 |
|---|---|
| `/robot_1/trajectory/reference` | 現在の目標Odometry。poseはtable_1、twistは目標姿勢の機体座標 |
| `/robot_1/trajectory/path` | 目標軌道の表示用Path |
| `/robot_1/trajectory/actual_path` | 入力状態による実際の軌跡 |
| `/robot_1/trajectory/error` | 診断値。x/yは位置誤差[m]、zはyaw誤差[rad] |
| `/robot_1/trajectory/active` | 自動追従が有効かどうか |
| `/robot_1/trajectory/enable` | `std_srvs/SetBool`。現在位置から滑らかに再開。絶対指定した円中心は保持 |

```bash
# 噴射を無効化
ros2 service call /robot_1/trajectory/enable std_srvs/srv/SetBool '{data: false}'
# 新鮮な状態が届いていることを確認して再開
ros2 service call /robot_1/trajectory/enable std_srvs/srv/SetBool '{data: true}'
```

無効化は噴射の停止で、機体は惰性で動きます。通常の減速停止・境界手前での制動は未実装です。
状態の欠測・古い時刻・座標系不一致・非有限値・指定範囲逸脱で自動追従を無効化し、
明示的な再開を要求します。既定の状態タイムアウトは0.25秒、パルス幅は10 msです。
これは公称値で、実機の最小噴射時間・弁応答に合わせた調整が必要です。

実機や外部推定器へ接続する場合は、共通制御launchを使用します。既定では出力無効です。

```bash
ros2 launch airtable_bringup trajectory_control.launch.py \
  state_topic:=/robot_1/state/odometry trajectory:=line
```

このlaunchはhardware・カメラ・推定器を起動しません。入力は新鮮な `nav_msgs/Odometry` とし、
poseのframeは `table_1`、twistのframeは `robot_1/base_link` が既定です。
実機の状態推定とスラスタI/Oは別途必要で、実機追従は未検証です。

再現用の検証:

```bash
.venv/bin/colcon test --packages-select airtable_controller --ctest-args -R test_trajectory_control
/usr/bin/python3 scripts/check_sim_trajectory.py
# 0 rad以外の姿勢でも円軌道を確認
/usr/bin/python3 scripts/check_sim_trajectory.py --trajectory circle --fixed-yaw 0.6 --cycles 1
# 初期位置、中心・半径、配信周期、カメラ観測の確認
/usr/bin/python3 scripts/check_sim_trajectory_setup.py --viewer --rviz
/usr/bin/python3 scripts/check_sim_trajectory_setup.py --tracking
```

詳細・検証値は[0010. 姿勢固定の軌道追従](docs/adr/0010-fixed-yaw-trajectory-tracking.md)を参照してください。

### 1基／2基の構成ファイルで起動する

テーブル2台に機体を1基ずつ置き、各機体が自分のテーブル座標で独立に追従する構成を
構成ファイルで切り替えられます（編隊・協調制御はありません）。構成ファイルは
ROSパラメータファイルとは別物で、`src/airtable_bringup/config/robots/` にあります。

| 構成 | 内容 | simモデル |
|---|---|---|
| `single_robot.yaml` | table_1/robot_1（従来と同じ要素名） | `airtable.xml` |
| `dual_robot.yaml` | table_1/robot_1（直線）、table_2/robot_2（円） | `airtable_dual.xml` |
| `robot_2_only.yaml` | table_2/robot_2のみ | `airtable_robot_2.xml` |

```bash
CFG=$(ros2 pkg prefix airtable_bringup)/share/airtable_bringup/config/robots

# 1基（既定は single_robot.yaml）。従来の simulation_trajectory.launch.py もそのまま使える
ros2 launch airtable_bringup simulation_robots.launch.py trajectory:=circle

# robot_2だけ（robot_1/table_1を含まないモデル）
ros2 launch airtable_bringup simulation_robots.launch.py robot_config:=$CFG/robot_2_only.yaml

# 2基を1つのMuJoCoで同時に追従。RVizはworld基準で両機を色分け表示
ros2 launch airtable_bringup simulation_robots.launch.py robot_config:=$CFG/dual_robot.yaml rviz:=true

# 2基同時にテーブルごとのカメラで画像観測（比較用。制御入力は真値のまま）
ros2 launch airtable_bringup simulation_robots.launch.py robot_config:=$CFG/dual_robot.yaml \
  tracking:=true viewer:=false rviz:=true

# 2基構成から1基だけ選ぶ／外乱をrobot_2だけに加える
ros2 launch airtable_bringup simulation_robots.launch.py robot_config:=$CFG/dual_robot.yaml robots:=robot_2
ros2 launch airtable_bringup simulation_robots.launch.py robot_config:=$CFG/dual_robot.yaml \
  disturbance:=true disturbance_robot:=robot_2
```

実機向けはPCとRaspberry Piで別々に起動します。ROS launchが別ホストを起動することはありません。

```bash
# PC: 両機の追従制御（既定で出力無効、入力は /robot_N/state/odometry、wall time）
ros2 launch airtable_bringup robots_trajectory_control.launch.py robot_config:=$CFG/dual_robot.yaml
# 各Pi: 自機のhardwareだけ（teleop・手動controllerは起動しない）
ros2 launch airtable_bringup robot_hardware.launch.py robot_config:=$CFG/dual_robot.yaml robot:=robot_1
ros2 launch airtable_bringup robot_hardware.launch.py robot_config:=$CFG/dual_robot.yaml robot:=robot_2
# 観測（カメラ・ArUco → /robot_N/measurements/external_pose）。未設定の機体は拒否される
ros2 launch airtable_bringup robot_tracking.launch.py robot_config:=$CFG/dual_robot.yaml robot:=robot_1
# 推定器が /robot_N/state/odometry を出してから機体ごとに有効化
ros2 service call /robot_2/trajectory/enable std_srvs/srv/SetBool "{data: true}"
```

- パラメータの優先順位は 参照ファイル ＜ 機体の `*_overrides` ＜ launch引数（全選択機体に適用）
  ＜ launchが導出する値（`reference_frame`=`table_frame`、`body_frame`、状態トピック、`enabled`）。
- 相対パスは構成ファイルの場所基準、`package://pkg/path` はshare基準。
  `robot_config:=` 自体は絶対パス、カレントディレクトリ基準の相対パス、または
  `package://airtable_bringup/config/robots/dual_robot.yaml` で指定する。
- 構成の確認と、録画用に解決済みパラメータを書き出すCLI:
  `ros2 run airtable_bringup robot_config check $CFG/dual_robot.yaml --simulation`、
  `ros2 run airtable_bringup robot_config export $CFG/dual_robot.yaml --output /tmp/cfg`
- 2基の録画は `experiment_bag record --profile dual_sim`（実機は `dual_real`）。
- simの2基モデルは、テーブル原点を短辺方向（world +X）に5 m離し、テーブルごとに
  固定カメラを1台置きます。マーカーIDはtable_1が固定10〜13・機体50、table_2が固定20〜23・機体51。
  2カメラ同時だとsimの描画レートは各約13 Hz（1カメラ時の約半分、sim側の制約）です。
  詳細は[0015](docs/adr/0015-per-table-camera-tracking.md)。
- 実機の world 配置・カメラserial・マーカー・IMU取り付け・物性値は未測定のため設定していません。
  robot_2の実カメラ観測設定も未作成で、`robot_tracking.launch.py robot:=robot_2` は拒否されます。
- 実機閉ループ追従には速度推定／EKFと実スラスタI/Oが別途必要です（未検証）。

検証は `scripts/check_sim_multi_robot.py`（robot_2単独・2基同時・故障／指令／外乱の分離・
録画・PC/Pi分離起動・設定エラー・テーブルごとのカメラ観測）。schema、DDS／時刻同期の運用条件、未実装範囲は
[0014. 1基／2基の起動構成](docs/adr/0014-multi-robot-configuration.md)を参照。

### 外乱力・外乱トルクを加えて追従を検証する

MuJoCoの機体重心へ、スラスタとは独立に平面外乱 `Fx / Fy / Mz` を加えられます。
力はN、トルクはN mです。既定では外乱ゼロで、明示的に有効にした場合だけ入力します。

```bash
source install/setup.bash
ros2 launch airtable_bringup simulation_trajectory.launch.py \
  trajectory:=circle radius:=0.20 disturbance:=true rviz:=true
# 直線単振動は trajectory:=line
```

既定は追従開始から8〜10秒の間、テーブル+Xへ0.02 N、正yawへ0.0005 N mの外乱です。
設定は `src/airtable_bringup/config/disturbance.yaml` で変更できます。
別ファイルは `disturbance_params_file:=/absolute/path/disturbance.yaml` で指定します。
外乱条件は起動時の読み取り専用パラメータです。変更はYAMLを編集して再起動し、
実行中の任意入力には下記のトピックを使います。

| 設定 | 内容 |
|---|---|
| `force_x_n / force_y_n / torque_z_nm` | 外乱の符号付き大きさ。gaussianでは平均値 |
| `frame_id` | 力の座標軸。`table_1`、`robot_1/base_link`、`world` |
| `profile` | `pulse`、継続する`step`、正負に変化する`sine`、確率外乱`gaussian` |
| `start_delay_s` | 追従開始時刻からの待機時間 |
| `duration_s` | 印加時間。sine / gaussianでは0なら継続、stepでは使わない |
| `period_s` | sineの周期 |
| `publish_rate_hz` | 配信周波数。既定50 Hz。watchdogの0.25秒より十分短い間隔にする |
| `wait_for_trajectory` | trueなら追従が無効の間は外乱もゼロ。falseならノード開始時刻から測る |

例えば`sine`で2周期加える場合は、`period_s: 5.0`、`duration_s: 10.0`にします。

時間はシミュレーション時刻です。`trajectory/path`の開始時刻を使うので、ノードの
起動順によるずれを避けられます。既に終了した時間窓の実験へ後から接続すると、
そのパルスは再実行しません。再開した追従では新しい開始時刻に合わせます。
手動で好きな時刻に入力したい場合は、次のトピックを使えます。

```bash
# 外乱生成ノードを起動していない状態で、別ターミナルから入力
ros2 topic pub --rate 50 /robot_1/sim/disturbance/wrench geometry_msgs/msg/WrenchStamped \
  '{header: {frame_id: table_1}, wrench: {force: {x: 0.02, y: 0.0}, torque: {z: 0.0005}}}'
```

`frame_id`は方向の座標軸を表し、印加点とトルクの基準点は常に機体重心です。
機体座標の外力は機体の回転に合わせて方向が変わり、テーブル座標の外力は固定方向です。
Z方向の力とroll / pitchトルクは、この平面モデルでは受け付けません。
外乱トピックの配信元は1つにしてください。複数の入力を加算する方式ではありません。

指令が途絶えると既定0.25秒で外乱を解除します。ゼロ指令でも解除できます。
タイムスタンプ0は受信時刻での即時入力、非ゼロはsim時刻として鮮度を検証します。
不正な座標系・NaN・古い／未来の時刻・重複指令は外乱を解除して拒否します。
`ros2 topic pub --once`は1回だけの速度インパルスではなく、期限までの有限時間の力入力です。

| トピック | 内容 |
|---|---|
| `/robot_1/sim/disturbance/wrench` | 要求した外乱。WrenchStamped |
| `/robot_1/sim/disturbance/applied` | 直近の物理ステップで実際に加えた外乱。機体座標、重心基準 |

適用値は100 Hzで出力し、stampは該当する物理ステップの開始時刻です。
Viewerのマウス摂動は別経路で加算され、この適用値には含まれません。
RVizでは外乱力を赤、トルクを黄で表示します。
MCAP録画の`sim`プロファイルにも入力と適用値を追加しています。
`--snapshot src/airtable_bringup/config/disturbance.yaml`で条件も記録できます。

外乱なし／ありの比較、印加後の回復、入力の座標変換・期限切れ・不正入力を自動検証できます。

```bash
/usr/bin/python3 scripts/check_sim_disturbance.py --plot
# 円運動だけ比較する場合
/usr/bin/python3 scripts/check_sim_disturbance.py --trajectory circle --plot
```

結果のCSV・JSON・比較グラフは表示された `/tmp/airtable_disturbance_check_*` に保存します。
自動回復検証は既定の有限パルスを対象とし、step / sineは設定を変えて起動できます。
現在のPD制御には積分項がないため、継続する一定外乱に対しては定常偏差が残ります。
この機能はsim専用で、実機へ外力を発生させるI/Oは追加していません。
仕様と検証値は[0012. 外乱入力による追従検証](docs/adr/0012-external-disturbance-tracking.md)を参照。

### 分散を持つ未知外力・外乱トルク

各軸独立の正規分布から外乱を生成する`gaussian`プロファイルも使えます。
`Fx / Fy / Mz`それぞれの平均・標準偏差、乱数シード、更新間隔を指定します。
実現した外乱はMuJoCoへ直接加え、制御器や推定器への既知入力には渡しません。
記録用の要求値・実適用値は従来の外乱トピックへ出します。

```bash
ros2 launch airtable_bringup simulation_trajectory.launch.py \
  trajectory:=circle radius:=0.20 disturbance:=true rviz:=true \
  disturbance_params_file:="$(ros2 pkg prefix --share airtable_bringup)/config/stochastic_disturbance.yaml"
```

設定は`src/airtable_bringup/config/stochastic_disturbance.yaml`です。
既定は追従開始8〜28秒、平均ゼロ、Fx/Fyの標準偏差0.02 N、Mzの標準偏差0.0005 N m。
0.1 sim秒ごとに新しい値を生成し、次の更新まで保持します。配信は50 Hzです。

| 設定 | 内容 |
|---|---|
| `force_x_n / force_y_n / torque_z_nm` | 各軸の平均。非ゼロで偏った未知外力も再現できる |
| `force_x_stddev_n / force_y_stddev_n` | 力の標準偏差[N]。分散はその2乗[N²] |
| `torque_z_stddev_nm` | トルクの標準偏差[N m]。分散はその2乗[(N m)²] |
| `noise_sample_period_s` | 乱数を更新するsim秒の間隔。配信間隔以上にする |
| `noise_seed` | 非負整数の乱数シード。同じ条件では同じサンプル系列 |

既定の外乱共分散は`diag(4e-4, 4e-4, 2.5e-7)`です。成分順は`[Fx,Fy,Mz]`で、
座標軸は`frame_id`に従います。軸間共分散はゼロとし、標準偏差0の軸は平均値を保持します。
`duration_s: 0.0`なら継続します。無効化中・開始前・有限時間の終了後は全軸ゼロです。
乱数はシードと開始時刻からの区間番号に対応するため、コールバックの欠落・途中接続でも
区間ごとの値は一致します。ROS通信・物理ステップへの到着には遅延があるため、
同じシードでも実際の適用時刻や軌跡の完全一致は保証しません。比較には`applied`を使います。
正規分布は裾を持ち、指定した標準偏差が外乱の上限になるわけではありません。

これは一定間隔で保持する離散的な外乱で、連続時間白色雑音のスペクトル密度とは異なります。
外乱の分散をそのままカルマンフィルタの状態プロセス雑音共分散`Q`には使いません。
例えば質量`m`の1軸で、区間`h`全体に同じ力が作用する場合、状態`[位置,速度]`に対し
`G = [h²/(2m), h/m]ᵀ`、`Q = G σ_F² Gᵀ`と離散化できます。
yawでは質量を慣性モーメントに置き換えます。推定周期が外乱更新間隔より短い場合、
複数の推定ステップに同じ外乱が残るため、ステップ間の相関も考慮する必要があります。
観測雑音の追加・EKF自体の実装や評価は含みません。追従の状態入力は現在も真値です。

乱数の分布・再現性の単体テストに加え、直線／円の外乱なし・ありの比較、
ROS出力のサンプル保持・シードとの一致・実印加・解除後の回復を検証できます。

```bash
/usr/bin/python3 scripts/check_sim_disturbance.py --plot \
  --disturbance-params src/airtable_bringup/config/stochastic_disturbance.yaml
```

CSV・JSON・比較グラフに加え、乱数条件・独立サンプルの実測平均と不偏分散を保存します。
録画時も`--snapshot src/airtable_bringup/config/stochastic_disturbance.yaml`で条件を残せます。
仕様と検証結果は[0013. 確率外乱](docs/adr/0013-stochastic-disturbance.md)を参照。

### 実験ログの録画とFoxglove用MCAP

録画対象は `src/airtable_bringup/config/recording.yaml` で管理します。
sim / 実機のプロファイル、画像の追加、名前空間・保存先の切り替えに対応し、
実験条件・設定ファイル・Git revisionをbagと一緒に保管できます。
通常はMCAPへ直接録画し、既存のSQLite bagには変換コマンドを使用します。

```bash
ros2 run airtable_bringup experiment_bag record --profile sim --dry-run
ros2 run airtable_bringup experiment_bag record --profile sim --label circle_r020
# 終了はCtrl+C。実機では --profile real を指定
ros2 run airtable_bringup experiment_bag convert /path/to/existing_bag
```

ビルド、YAMLの拡張、条件・設定の保存、画像付き録画、Foxgloveでの再生は
[実験ログの手順](src/airtable_bringup/README.md)を参照してください。

### SG90によるRW接続デバッグ

SG90をRWのダミーとして接続する場合は、
[SG90デバッグ手順](docs/sg90-debug.md) を参照してください。
device部はRustで実装し、GPIOなしの確認とPC / Raspberry Pi 5の分割起動に対応しています。
SG90は指令トルクを角度で表示し、RW状態は仮想モデルの計算値です。
設計判断は [0008. SG90をデバッグ用RWとして接続する](docs/adr/0008-sg90-debug-reaction-wheel.md)。

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
- [0014. 1基／2基の起動構成を設定ファイルで切り替える](docs/adr/0014-multi-robot-configuration.md)
  — 構成ファイルとROSパラメータの分離、2基sim、PC/Pi分離起動、検証結果

## 進行中の作業と残課題

[docs/WORKLOG.md](docs/WORKLOG.md) を参照してください。

## AIエージェント向けの前提

[CLAUDE.md](CLAUDE.md) にビルド手順の厳守事項、探索しなくてよいディレクトリ、
実装上の約束をまとめています。
