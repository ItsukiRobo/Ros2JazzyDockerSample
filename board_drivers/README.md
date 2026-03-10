# board_drivers

このディレクトリには、CONTEC の IO ボード用カーネルドライバを置く。

## 置いてあるもの

- `AI1616LLPE/`: AI-1616L-LPE 用ドライバ
- `AO1608LLPE/`: AO-1608L-LPE 用ドライバ
- `start_driver.sh`: ドライバロードと `/dev/AI`, `/dev/AO` の作成

## 注意

このドライバは ROS 2 ノードではなく Linux カーネルモジュール (`.ko`) なので、実行中のホストカーネルに合わせてビルドされている必要がある。

そのため、同じ PC 上のファイルをコピーした場合でも、以下のような条件ではそのまま使えないことがある。

- Ubuntu 更新などでカーネルバージョンが変わった
- 以前のカーネル向けにビルドした `.ko` をそのまま持ってきた

その場合、`start_driver.sh` 実行時に以下のようなエラーになる。

```text
insmod: ERROR: could not insert module ...: Invalid module format
```

`modinfo` で見る `vermagic` と `uname -r` が一致していなければ、カーネル不一致が原因。

## 再ビルド手順

まず現在のカーネルを確認する。

```bash
uname -r
```

必要ならカーネルヘッダを入れる。

```bash
sudo apt update
sudo apt install build-essential linux-headers-$(uname -r)
```

その後、各ドライバを現在のカーネル向けにビルドし直す。

```bash
cd ~/sample_project/board_drivers/AI1616LLPE
make clean
make

cd ~/sample_project/board_drivers/AO1608LLPE
make clean
make
```

ビルド後に `vermagic` が現在カーネルと一致することを確認する。

```bash
modinfo ~/sample_project/board_drivers/AI1616LLPE/AI1616L.ko | grep vermagic
modinfo ~/sample_project/board_drivers/AO1608LLPE/AO1608L.ko | grep vermagic
uname -r
```

## ドライバのロード

PC 再起動後はドライバが未ロード状態に戻るため、OS 起動後に 1 回 `start_driver.sh` を実行する。

```bash
cd ~/sample_project
sudo bash ./board_drivers/start_driver.sh
```

これは Docker を起動するたびに毎回必要なわけではなく、同じ OS 起動中なら通常は 1 回でよい。

## 確認方法

ロード後は以下を確認する。

```bash
lsmod | grep -E 'AI1616L|AO1608L'
ls -l /dev/AI /dev/AO
cat /proc/devices | grep -E 'AI1616L|AO1608L'
```

期待する状態:

- `lsmod` に `AI1616L` と `AO1608L` が出る
- `/dev/AI`, `/dev/AO` が存在する
- `/proc/devices` に `AI1616L`, `AO1608L` が登録されている
