
# Publish

次は、カメを動かす簡単なプログラムを作成してみましょう。

<br>

## パッケージの作成

C++を使用して直進を行うプログラムを作成してみましょう。

ROS2は1つのパッケージの作成に必要なファイルがいくつかあります。

最初はかなり面倒だとは思いますが、頑張って慣れていきましょう！

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
source /opt/ros/humble/setup.bash

ros2 pkg create --build-type ament_cmake --depend rclcpp geometry_msgs turtlesim --node-name my_node my_package
```

パッケージ作成を行ったので、ひとまずビルドしましょう。

```bash
cd ~/ros2_ws/
colcon build
```

> launchファイルを毎回書き換えるたびにビルドが必要です。`--symlink-install`オプションを使用するとその手間が省けます。

<br>

## プログラムの作成

`code ~/ros2_ws/src/my_package/src/my_node.cpp` コマンドでVSCodeを開いて以下のコードに置き換えます。

```c
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

class MyClass : public rclcpp::Node
{
public:
    MyClass() : Node("my_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MyClass::timer_callback, this));
    }

private:
    void timer_callback()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 1.0;
        message.angular.z = 1.0;
        publisher_->publish(message);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyClass>());
    rclcpp::shutdown();
    return 0;
}
```

ビルドには`colcon build`を使用します。

<br>

## 実行

turtlesimを起動します（既に起動しているなら不要です）

```bash
source ~/ros2_ws/install/setup.bash
ros2 run turtlesim turtlesim_node
```

次に、作成したプログラムを実行します。

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_package my_node --ros-args -r /cmd_vel:=/turtle1/cmd_vel
```

半時計回りにカメが回転します。

![](https://storage.googleapis.com/zenn-user-upload/a46b9985aae2-20230121.png)

<br>