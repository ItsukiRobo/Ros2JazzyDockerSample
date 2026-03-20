# sample_project（ROS 2 Jazzy / Docker）README
## 概要

sample_project は、Ubuntu 24.04 + ROS 2 Jazzy 環境で動作するサンプルプロジェクトです。
Docker コンテナ内で ROS 2 を起動し、実行時の設定（parameters）と実行データ（rosbag2）を /artifacts 配下に一括保存します。さらに、標準メッセージのみを抽出した rosbag2 から ROS1 bag（.bag）へ変換する機能を備えます（ROS1/Noetic は使用しません）。

## 目的（このプロジェクトでやること）
- bringup package の launch からシステムを起動
- 起動後に parameter dump を params_dump.txt に保存（任意）
- 実行中の rosbag2 記録
- rosbag2_all：全トピック（独自msg含む）
- rosbag2_std：標準メッセージのみ（ros1変換用）
- 実行終了（Ctrl+C / docker stop）後に rosbag2_std を ROS1 bag へ変換（任意）
- 生成物を /artifacts/<run_id>/... に集約し、ホストへ出力

## 想定ディレクトリ構成
```
sample_project/
  README.md
  board_drivers/
    start_driver.sh          # ホスト側でIOボードドライバをロード
    AI1616LLPE/
      Makefile
      AI1616L.c
    AO1608LLPE/
      Makefile
      AO1608L.c
  docker/
    Dockerfile
    entrypoint.sh
  ros2_ws/
    src/
      sample_project_bringup/
        package.xml
        setup.py
        launch/
          bringup.launch.py
      sample_project_tools/
        package.xml
        setup.py
        sample_project_tools/
          dump_params.py   # params_dump 用
  artifacts/              # ホスト側に成果物が溜まる（bind mount）
```
## 前提環境
- Ubuntu 24.04
- Docker（docker engine / docker compose plugin）
- インターネット接続（初回 build の apt / pip に必要）

## ビルド

プロジェクト直下で：
```
cd ~/sample_project
docker build -t sample_project:jazzy -f docker/Dockerfile .
```
## 実行（基本）
PC 再起動後はホスト側でボードドライバが未ロードの状態に戻るため、Docker 起動前に1回 `board_drivers/start_driver.sh` を実行してください。
```
cd ~/sample_project
sudo bash ./board_drivers/start_driver.sh
```

artifacts/ をホスト側に出す（run_id ごとに保存される）：
```
docker run --rm -it \
  -e TZ=Asia/Tokyo \
  -e HOST_UID=$(id -u) \
  -e HOST_GID=$(id -g) \
  --device /dev/AI \
  --device /dev/AO \
  --device /dev/CNT \
  -v ~/sample_project/artifacts:/artifacts \
  sample_project:jazzy
```
AI-1616L-LPE / AO-1608L-LPE / CNT-3204MT-LPE を使う場合は、対応する `/dev/AI`, `/dev/AO`, `/dev/CNT` をコンテナへ渡す必要があります。渡していない場合、`peripheral` ノードはデバイス open に失敗して終了します。

## 停止
実行中のターミナルで Ctrl + C

または別ターミナルから docker stop（コンテナ名を付けて起動した場合）

停止すると cleanup が走り、bag/変換/保存が実行されます。

成果物（/artifacts/<run_id>/）

例：
```
/artifacts/20260304_112107/
  params_dump.txt                # ENABLE_PARAMS_DUMP=1 のとき
  rosbag2_all/run/               # 全部入り（独自msg含む）
    metadata.yaml
    *.mcap など
  rosbag2_std/run/               # 標準msgだけ（変換用、sqlite3推奨）
    metadata.yaml
    *.db3 など
  ros1bag/run.bag                # ENABLE_ROS1_CONVERT=1 のとき
  std_topics_used.txt            # std側で録音したトピック一覧
  convert_rosbags_stdout.log
  convert_rosbags_stderr.log
  ```
## フラグ（機能ON/OFF）
### parameter dump のON/OFF

ENABLE_PARAMS_DUMP=1（デフォルト：有効）

ENABLE_PARAMS_DUMP=0（無効）

例：
```
docker run --rm -it \
  -e TZ=Asia/Tokyo \
  -e ENABLE_PARAMS_DUMP=0 \
  -v ~/sample_project/artifacts:/artifacts \
  sample_project:jazzy
```
### ROS1 bag 変換のON/OFF

ENABLE_ROS1_CONVERT=1（デフォルト：有効）

ENABLE_ROS1_CONVERT=0（無効）

例：
```
docker run --rm -it \
  -e TZ=Asia/Tokyo \
  -e ENABLE_ROS1_CONVERT=0 \
  -v ~/sample_project/artifacts:/artifacts \
  sample_project:jazzy
``` 
## STD_TOPICS（標準メッセージのみを自動選別）

rosbag2_std は ros1変換の安定性のため、標準 msg パッケージ（allowlist）に一致するトピックのみを録音します。

allowlist（デフォルト想定）
- std_msgs（例：Float32MultiArray / Float64MultiArray を含む）
- sensor_msgs
- geometry_msgs
- nav_msgs
- tf2_msgs
- diagnostic_msgs
- rcl_interfaces（/rosout, /parameter_events）
- builtin_interfaces

（必要に応じて trajectory_msgs 等追加してください）

### allowlist を上書きする
```
docker run --rm -it \
  -e TZ=Asia/Tokyo \
  -e STD_ALLOW_PKGS="std_msgs geometry_msgs sensor_msgs nav_msgs tf2_msgs diagnostic_msgs rcl_interfaces builtin_interfaces" \
  -v ~/sample_project/artifacts:/artifacts \
  sample_project:jazzy
```
注意：独自msgパッケージを allowlist に入れると、ros1変換で失敗しやすくなります。
独自msgは rosbag2_all に残し、std側には入れない運用を推奨します。

## 権限（artifacts が消せない問題への対策）

コンテナが root で書き込むと、ホスト側の成果物が root 所有になり削除できない場合があります。

### 対策1：実行後に所有権をホストユーザへ戻す
OST_UID と HOST_GID の両方がある時だけ 処理が呼ばれます。
```
docker run --rm -it \
  -e TZ=Asia/Tokyo \
  -e HOST_UID=$(id -u) \
  -e HOST_GID=$(id -g) \
  -v ~/sample_project/artifacts:/artifacts \
  sample_project:jazzy
```
### 対策2：すでに root 所有になってしまった場合（ホストで）
sudo chown -R $USER:$USER ~/sample_project/artifacts

## bringup の変更方法

起動対象は bringup package の launch です。
例：sample_project_bringup/launch/bringup.launch.py

まずは動作確認として demo_nodes_cpp の talker を起動できます（コンテナに ros-jazzy-demo-nodes-cpp が入っている必要があります）。

## cylinder_force_controller の action

`cylinder_force_controller` は起動直後、`startup_target_force_n` を目標に `HOLD` モードで制御します。
その後、action `/cylinder_force_controller/track_sine_force` に goal を送ると `SINE_TRACKING` モードへ切り替わり、loadcell の force が目標 sin 波に追従するように制御します。
goal の終了または cancel 後は `HOLD` モードへ戻ります。

### action の型
`controller/action/TrackSineForce`

Goal:
- `amplitude_n`
- `offset_n`
- `frequency_hz`
- `duration_s`
- `phase_rad`

### 実行例
コンテナ内または ROS 2 環境を source 済みの端末で実行します。

```bash
ros2 action send_goal /cylinder_force_controller/track_sine_force controller/action/TrackSineForce "{amplitude_n: 50.0, offset_n: 0.0, frequency_hz: 0.5, duration_s: 10.0, phase_rad: 0.0}"
```

例の意味:
- 振幅 50 N
- オフセット 0 N
- 周波数 0.5 Hz
- 10 秒間
- 初期位相 0 rad

### feedback / result
feedback では以下を返します。
- `elapsed_time_s`
- `target_force_n`
- `measured_force_n`

result では以下を返します。
- `success`
- `message`

別端末から `ros2 action send_goal` を再度送ると、新しい goal が優先され、前の goal は中断されます。

## トラブルシュート
### `start_driver.sh` 実行時に `Invalid module format` が出る

事象の例：

```text
sudo bash ./board_drivers/start_driver.sh
Enabling registers for IO boards...
Loading board driver modules...
insmod: ERROR: could not insert module ...: Invalid module format
```

この場合は、`.ko` が現在のホストカーネル向けにビルドされていない可能性が高いです。
Ubuntu 更新後や、以前のカーネル向けに作ったドライバをそのまま使った場合に起きます。

まず現在のカーネルと、ドライバの `vermagic` を確認します。

```bash
uname -r
modinfo ~/sample_project/board_drivers/AI1616LLPE/AI1616L.ko | grep vermagic
modinfo ~/sample_project/board_drivers/AO1608LLPE/AO1608L.ko | grep vermagic
modinfo ~/sample_project/board_drivers/CNT3204MTLPE/CNT3204MT.ko | grep vermagic
```

`vermagic` と `uname -r` が一致しない場合は、ドライバの再 `make` が必要です。

```bash
cd ~/sample_project/board_drivers/AI1616LLPE
make clean
make

cd ~/sample_project/board_drivers/AO1608LLPE
make clean
make

cd ~/sample_project/board_drivers/CNT3204MTLPE
make clean
make
```

必要に応じて、先にカーネルヘッダを入れてください。

```bash
sudo apt update
sudo apt install build-essential linux-headers-$(uname -r)
```

再ビルド後にもう一度 `vermagic` を確認し、最後にロードし直します。

```bash
cd ~/sample_project
sudo bash ./board_drivers/start_driver.sh
```

詳しい手順は [board_drivers/README.md](/home/kklab/sample_project/board_drivers/README.md) も参照してください。
