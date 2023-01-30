# インストール（Ubuntu）

Ubuntuへのインストールを行います。

<br>

## 要件

> Ubuntuは偶数年の最初にリリースされたバージョンのみをサポートしています。例えば20.10などはサポート対象外です。（ただし、自力でビルドすることは可能です。）

| Distro | OS |
| --- | --- |
| Humble | Ubuntu 22.04 |
| Galactic | Ubuntu 20.04 |
| Foxy | Ubuntu 20.04 |

<br>

## インストール

> Ubuntu 22.04をインストールした状態で、以下のコマンドを実行してください。

参考URL

- [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
- [Galactic](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html)
- [Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

```bash
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
```

```bash
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

<br>

ROS-Humbleをインストールします。`ros-humble-desktop`はビジュアライゼーションツール等を含む開発者向けのパッケージです。

```bash
sudo apt update
sudo apt install ros-humble-desktop
```

必要に応じて次のパッケージを選択することもできます。

- -desktop-full : シミュレーション系も含むパッケージ
- -base : ビジュアライゼーションツールを含まないパッケージ
- -core : rclcppやrclpyのAPIが含まれる最も基本のパッケージ

<br>

開発者向けのツールをインストールします。

```bash
sudo apt install ros-dev-tools
```

<br>

## 簡単な動作確認

`ros-humble-desktop`をインストールした場合、次のコマンドを入力してROS2の動作を確認できます。

まずは、C++のTalkerを起動します。

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

別のターミナルを開き、PythonのListenerを起動します。

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

これで、ROS2のC++のAPIとPythonのAPIが正常に動作していることが確認できます。

<br>

## アンインストール

```bash
sudo apt remove ros-humble-desktop-*
sudo apt autoremove
```

<br>
