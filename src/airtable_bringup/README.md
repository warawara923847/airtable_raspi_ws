# 実験ログの録画とMCAP変換

実験の起動と録画を独立させ、同じ録画コマンドをMuJoCo、手動操作、軌道追従、
Raspberry Pi上の実験に使います。録画コマンド自身はsim・hardware・制御を起動しません。

ROS 2のrosbagはMCAPを保存形式に選べます。通常は**直接MCAPへ録画**し、既存の
SQLite bagだけ変換します。ライブ表示用のFoxglove BridgeやFoxgloveへのログインは
録画には不要です。クラウドへのアップロードは別途行います。

## 準備

```bash
sudo apt install ros-jazzy-ros2bag ros-jazzy-rosbag2-transport \
  ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap \
  ros-jazzy-rosbag2-storage-sqlite3 python3-yaml

cd ~/Documents/airtable_raspi_ws
source /opt/ros/jazzy/setup.bash
.venv/bin/colcon build --symlink-install --packages-select airtable_bringup \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

以下の各ターミナルでもROSと`install/setup.bash`をsourceしてください。
ビルド前にソースから動かす場合は、`ros2 run airtable_bringup experiment_bag`の代わりに
`/usr/bin/python3 src/airtable_bringup/scripts/experiment_bag.py`を使用できます。

## 基本の録画

まずYAMLから解決したトピック・QoS・時刻・コマンドを確認します。ファイルは生成しません。

```bash
ros2 run airtable_bringup experiment_bag record --profile sim --dry-run
```

ターミナル①で録画を開始します。`--label`は英数字・`_`・`-`・`.`で80文字まで。
`--meta`の値は文字列として保存され、日本語のメモは`--note`に書けます。

```bash
ros2 run airtable_bringup experiment_bag record \
  --profile sim --label circle_r020 \
  --meta trajectory=circle --meta radius_m=0.20 --meta period_s=20.0 \
  --note '公称1 kgモデル、真値による追従制御' \
  --snapshot src/airtable_controller/config/trajectory.yaml \
  --snapshot src/airtable_controller/config/thrusters.yaml \
  --snapshot src/airtable_sim/mujoco/airtable.xml \
  --snapshot src/airtable_bringup/launch/simulation_trajectory.launch.py
```

トピックを探索し続けるため、simより先に録画を起動できます。ターミナル②で実験を開始します。

```bash
ros2 launch airtable_bringup simulation_trajectory.launch.py \
  trajectory:=circle radius:=0.20 period:=20.0 viewer:=false
```

終了時は**録画側でCtrl+Cを押して保存完了を待ってから**simを終了します。
SIGTERMも内部でSIGINTとしてrosbag2に伝え、書き込み完了を待ちます。
SIGKILLや電源断では正常終了時のメタデータを保存できない場合があります。

外乱付きの追従実験では、実験側へ`disturbance:=true`を追加します。
`sim`プロファイルは要求外乱`{ns}/sim/disturbance/wrench`と、実適用値
`{ns}/sim/disturbance/applied`も保存します。条件は
`--snapshot src/airtable_bringup/config/disturbance.yaml`で一緒に残せます。
確率外乱の場合は`disturbance_params_file`で`config/stochastic_disturbance.yaml`を指定し、
このファイルを`--snapshot`へ渡してください。平均・標準偏差・更新間隔・シードを記録できます。

`sim`は`/clock`に基づくシミュレーション時刻で記録し、最初のclock受信まで待ちます。
simを再起動して時刻を0へ戻す場合は録画も新しく開始してください。

## 画像・実機・別の実験条件

画像は容量を抑えるため既定では保存しません。必要なときだけグループを追加します。
このオプションは録画対象の追加だけを行い、カメラや検出器は起動しません。

```bash
# 実験側のlaunchにもtracking:=trueを指定して画像を配信する
ros2 run airtable_bringup experiment_bag record \
  --profile sim --include-group sim_images --label circle_with_camera

# Raspberry Piなど、実機のROSトピックを受信できる環境で実行する
ros2 run airtable_bringup experiment_bag record \
  --profile real --namespace robot_1 --label manual_trial01 \
  --output-root /mnt/experiment_logs --note '手動操作試験'

# 別機体の名前空間。録画側だけを変更するため、実験側の名前空間も合わせる
ros2 run airtable_bringup experiment_bag record \
  --profile real --namespace robot_2 --include-group real_images --label trial02
```

`real`はシステム時刻で記録し、`/clock`を保存しません。PCからPiのトピックを録画する
場合はDDSのネットワーク設定・`ROS_DOMAIN_ID`を合わせます。通信切断の影響を避けたい
実機実験では、Piに接続したストレージへPi自身で録画してください。

現在のhardwareのIMUはDummyImuで、SG90のRW stateも仮想モデルです。録画した値が実測値に
なるわけではありません。位置・速度推定器は未実装で、予約してある`state/odometry`は
現時点では出力されません。

## 2テーブル2基構成の録画

`dual_sim`／`dual_real`は両機の絶対トピック（状態・目標・誤差・有効状態・wrench・
setpoint・pulse、simでは外乱とIMU、実機では観測・IMU）と`/tf`・`/tf_static`を列挙した
プロファイルです。`--namespace`は影響しません。構成ファイルと、機体別に解決した
ROSパラメータを書き出してから一緒に保存します。

```bash
CFG=$(ros2 pkg prefix airtable_bringup)/share/airtable_bringup/config/robots
ros2 run airtable_bringup robot_config export $CFG/dual_robot.yaml --output /tmp/dual_cfg
ros2 run airtable_bringup experiment_bag record --profile dual_sim --label dual_line_circle \
  $(for f in /tmp/dual_cfg/*; do echo --snapshot $f; done) \
  --snapshot src/airtable_sim/mujoco/airtable_dual.xml
```

`export`は構成ファイルのコピー、`resolved_robot_config.yaml`、`robot_N_trajectory_controller.yaml`・
`robot_N_thrusters.yaml`（上書き適用後の`ros__parameters`）を出力します。
launch引数で変えた条件は`--meta trajectory=circle`などで残してください。

## YAMLの管理

[config/recording.yaml](config/recording.yaml)に設定をまとめています。

| 設定 | 用途 |
|---|---|
| `defaults.namespace` | 既定の機体名。`{ns}`を`/robot_1`などへ展開 |
| `defaults.output_root` | 保存先。相対パスは実行時のカレントディレクトリ基準 |
| `defaults.storage` | `mcap`（既定）または`sqlite3` |
| `defaults.storage_preset_profile` | `zstd_fast`、`zstd_small`、`none`。MCAP内部の圧縮 |
| `defaults.max_bag_duration` | 分割間隔［秒］。既定300、0で無効 |
| `defaults.max_bag_size` | 分割サイズ［byte］。既定1 GiB、0で無効 |
| `topic_groups` | 再利用するトピックの集合 |
| `profiles` | グループ・追加トピック・時刻の組み合わせ |
| `qos_overrides` | トピックごとのQoS。`/tf_static`をtransient_localで受信 |

サイズと時間のどちらかへ先に到達すると分割されます。MCAPの内部chunk圧縮なので
Foxgloveで直接開けます。ROSのfile圧縮による`.mcap.zstd`は生成しません。

たとえば推定器を追加した実験は、設定ファイルをコピーし`profiles`へ以下を追加します。
`topics`には既存グループにない実験用のトピックを書けます。

```yaml
profiles:
  estimator_trial:
    use_sim_time: true
    groups: [common, control, simulation, tracking]
    topics:
      - '{ns}/estimator/debug'
```

```bash
ros2 run airtable_bringup experiment_bag record \
  --config /path/to/experiment_recording.yaml \
  --profile estimator_trial --label estimator_trial01
```

別のROSシステムでも、YAMLにプロファイル・グループを追加すれば同じCLIを使用できます。
グローバルな`/tf`や`/clock`、カメラ名は`--namespace`では変わりません。
RealSenseの名前は環境で異なるため、`real_images`を実際の`ros2 topic list`と照合してください。
単体hardware起動のように名前空間がない場合は`--namespace /`を指定します。

未配信のトピックを含むグループも選べます。終了時に実際のメッセージ件数と未取得の
トピックを報告するので、必須のセンサが記録されたか確認してください。
全体で0件なら失敗として扱います。必要なトピックが一部欠けた場合は警告し、他の記録は残します。

## 保存される内容

```text
recordings/<UTC日時>_<label>_<一意な接尾辞>/
  experiment.yaml          # 条件、メモ、Git revision/dirty、ROS環境、終了状態、実際の件数
  recording_source.yaml    # 使用した録画YAMLのコピー
  resolved_recording.yaml  # 展開後のトピック・実効録画オプション
  qos.yaml                 # rosbag2へ渡したQoS
  rosbag2.log              # 録画プロセスの出力
  inputs/                  # --snapshotで明示した入力ファイルとSHA-256を記録
  bag/
    metadata.yaml
    bag_0.mcap
    bag_1.mcap             # 時間・サイズによる分割があれば追加
```

Gitのdirty状態は記録しますが、作業ツリーの差分や実行ノードの全パラメータは自動取得しません。
再現に必要な設定・モデルは`--snapshot`でコピーし、launch引数などの変更条件は`--meta`へ
記載してください。`parameter_events`は録画開始後のイベントで、起動時の全パラメータを
保証するものではありません。

`/rosout`はROSへ配信されたログだけです。各ノードの標準出力やROSへ送っていないRustの
ログはbagに入りません。`rosbag2.log`も実験全体の標準出力ではありません。
必要なら実験launchの出力・ROSログディレクトリを併せて保管してください。

ファイルは既存の録画を上書きしない新規ディレクトリへ保存します。
`recordings/`とbagファイルはGit管理対象から除外しています。外部保存先もバックアップを
行ってください。実行環境全体や認証情報の自動収集はしません。

## 既存bagからMCAPへの変換

```bash
# 分割bagはmetadata.yamlのあるディレクトリを渡すと全セグメントを変換
ros2 run airtable_bringup experiment_bag convert /path/to/old_rosbag \
  --label old_trial --output-root recordings

# 1ファイルだけの変換にも対応。分割bag中の.db3の場合、そのファイルの分だけを変換
ros2 run airtable_bringup experiment_bag convert /path/to/old_rosbag/old_rosbag_0.db3

# 必要なら新規録画もSQLiteにできる
ros2 run airtable_bringup experiment_bag record --profile real --storage sqlite3
```

変換はトピックをROSへ再配信せず、全トピック（hidden topicを含む）を読み込んで別のMCAPを
作ります。元のbagは保持します。終了時にトピックごとの件数を入力と照合し、不一致は
失敗にします。分割された元ファイルと`metadata.yaml`は揃えてください。
SQLiteから独自メッセージを変換する場合は、その型をビルドしたワークスペースもsource
しておきます。失われたメッセージ定義を変換だけで復元することはできません。

変換結果も同じ実験ディレクトリ形式で、`convert.yaml`、`experiment.yaml`、`rosbag2.log`、
`bag/*.mcap`を保存します。単一ファイル変換ではJazzyの分割bagパス解決に対応するため、
一時ディレクトリに元ファイルへの参照と専用metadataを作り、処理後に削除します。

## Foxgloveで開く

Foxgloveの「Open local file(s)」で`bag/*.mcap`を選択します。分割されている場合は全ファイルを
まとめて開いてください。3D表示には`/tf`・`/tf_static`も録画してあります。
今回のsimの表示座標系は`table_1`です。クラウド保管には別途「Recordings」からインポートします。
実験条件を記録した`experiment.yaml`や`inputs/`はMCAPとは別のファイルなので一緒に保管します。

## 検証

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
.venv/bin/colcon test --packages-select airtable_bringup \
  --ctest-args -R test_experiment_bag --output-on-failure

# 専用ドメイン195でheadless simと合成センサだけを起動する。実機hardwareは起動しない
/usr/bin/python3 scripts/check_experiment_bag.py
```

統合検証では、sim/実機用時刻の区別、名前空間、画像の明示追加、best-effort IMU、
遅れて参加した録画への静的TF、SIGINT/SIGTERMでの正常保存を確認します。
分割SQLite・単一SQLite・MCAPの変換はトピックごとのCDRデータとタイムスタンプを比較し、
独自メッセージ定義もMCAPへ埋め込まれていることを確認します。
生成した診断ログとbagは`/tmp/airtable_bag_check_*`に保存します。

参考: [rosbag2 Jazzy](https://github.com/ros2/rosbag2/blob/jazzy/README.md)、
[MCAP storage plugin](https://github.com/ros2/rosbag2/blob/jazzy/rosbag2_storage_mcap/README.md)、
[FoxgloveのROS 2接続](https://docs.foxglove.dev/docs/getting-started/frameworks/ros2)。
