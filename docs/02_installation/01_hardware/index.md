# ROSにインストールするハードウェア

ROSのインストールについて説明します。

## 目的・目標

どのハードウェアにROSを入れたら良いのかを示します。

<br>

## ROS2の要件

ROSのソースコードはコアな部分についてはオープンになっています。

そのため、依存関係を頑張って解消してビルドを通せば（極端な話）基本的にはどのOSやアーキテクチャに対してもインストールできます。

### OS

- Linux
- Windows
- Mac (公式にメンテナンスされなくなりました)

<br>

## ハードウェア

### デスクトップ

- x86-64 であれば、ほとんどの場合使用できます。

### ノートPC

- x86-64 であれば、ほとんどの場合使用できます。ただし、Ubuntuをインストールしやすい構造（SSD・BIOSへのアクセスなど…）のノートPCをお勧めします。

- MacBookは非推奨

### シングルボードコンピュータなど

| 名称 | 特徴 | 消費電力(目安) | 公式価格 | 補足 | URL |
| --- | --- | --- | --- | --- | --- |
| RaspberryPi4 | 小型 | 6.4W (推奨: 15W) | $35~ | | [URL](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/)
| Jetson Nano | NVIDIA Maxwell™ architecture with 128 NVIDIA CUDA® cores and 0.5 TFLOPS (FP16) | 5W | $99 | Ubuntu18.04まで |　[URL](https://developer.nvidia.com/embedded/jetson-nano-developer-kit) |

<br>

### その他

Intel Nuc

<!-- 未確認リスト -->
<!-- Rock 3A -->
<!-- RaspberryPi Zero 2W -->

<br>
