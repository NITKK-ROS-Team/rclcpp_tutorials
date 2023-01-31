# 基本のPub-Sub (Turtlesim)

<br>

turtlesimとは、カメのキャラクターを使用したROSのGUIチュートリアルプログラムです。

turtlesimに登場するカメは各ROSのディストリビューションのキャラクターです。

![](https://storage.googleapis.com/zenn-user-upload/d020c6e2bfb3-20230121.png)

[ros-tutorials](https://github.com/ros/ros_tutorials)より

このカメは`geometry_msgs/msg/Twist`（カメの速度を送信するメッセージ）を受信し、`turtlesim/msg/Pose`（カメの位置を送信するメッセージ）と`turtlesim::msg::Color`（カラーセンサ）を送信することができます。

<br>

## 互換性

> 最も基本のチュートリアルパッケージに含まれており、ほとんどのROSのディストリビューションで使用できます。

| Distro | チェック |
| --- | --- |
| Galactic | ✅ |
| Humble | ✅ |

| OS | ターゲット | チェック |
| --- | --- | --- |
| Ubuntu22.04 | Intel Core-i5 Laptop | ✅ |
| Ubuntu22.04 | RaspberryPi4 | ✅ |

<br>

## turtlesimの起動

以下のコマンドを実行して、turtlesimを起動します。

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

![](https://storage.googleapis.com/zenn-user-upload/5dfd5a5ab08c-20230121.png)

カメが出現します。カメの種類は毎回ランダムです。

まずはTurtleを1秒だけ動かしましょう。

「ros2 topic pub」コマンドでturtlesimに対してコマンドを送ることができます。

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

![](https://storage.googleapis.com/zenn-user-upload/00d226e6a3ed-20230121.png)

`--once`を使うことで1回のみ送ることができます。

プログラムモジュールに対してメッセージを送りたい場合は`ros2 topic pub`を使用しましょう。


次に、以下のコマンドを入力してこのカメを操縦してみます。

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/turtle1/cmd_vel
```

![](https://storage.googleapis.com/zenn-user-upload/d7745a4edb15-20230121.png)

キーボードを押すことでカメを操縦することができます。

このプログラムでは、押されたキーを元にカメに`/turtle1/cmd_vel`（Twist）を送信します。

「Ctrl+c」を押してプログラムを終了します。