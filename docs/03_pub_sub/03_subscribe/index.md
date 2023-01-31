# Subscribe

ここでは、カメの現在位置をSubscribeします。

先に述べたようにカメからは自己位置とカラーセンサのトピックが出力されており、ここでは自己位置を取得することを目標にします。

新しく `sub_node` を作成します。


```bash
cd ~/ros2_ws
touch ~/ros2_ws/src/my_package/src/sub_node.cpp
# code ~/ros2_ws/src/my_package/src/sub_node.cpp
```

以下のプログラムを追記します。

```c
#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>

class SubClass : public rclcpp::Node
{
public:
    SubClass() : Node("sub_node")
    {
        using namespace std::placeholders;
        subscription_ = this->create_subscription<turtlesim::msg::Pose>("pose", 10, std::bind(&SubClass::topic_callback, this, _1));
    }

private:
    void topic_callback(const turtlesim::msg::Pose::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: x: '%f', y: '%f'. r: '%f'", msg->x, msg->y, msg->theta);
    }

    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubClass>());
    rclcpp::shutdown();
    return 0;
}
```

<br>

次はCMakeListsにコンパイル対象を追加していきます。

ただし、今回追記する部分は被っている部分が多いので、`set()`で変数を指定します。

14行目付近に`set(TARGET my_node)`を追加します。27行目の`install`の部分以外のmy_nodeを`${TARGET}`に変更します。

以下の`CMakeLists.txt`に置き換えます。

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

set(TARGET my_node)
add_executable(${TARGET} src/${TARGET}.cpp)
target_include_directories(${TARGET} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${TARGET} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ${TARGET}
  "rclcpp"
  "geometry_msgs"
  "turtlesim"
)

install(TARGETS my_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

<br>

以下の画像はTARGET変数を設定した後です。

![](https://storage.googleapis.com/zenn-user-upload/a79fc69e8282-20221123.png)

`sub_node`をコンパイル対象に追加します。上の水色枠を下にコピペしてTARGETの変数を変更します。
また、`install`のターゲットにsub_nodeを追記します。

以下の画像はその作業を行った後です。

![](https://storage.googleapis.com/zenn-user-upload/23020957fb57-20221123.png)

コピペではなく、自分で修正を行ってみましょう。

<br>

## ビルド・実行

```bash
cd ~/ros2_ws
colcon build
```

実行してみます

**turtlesim側**


```bash
ros2 run turtlesim turtlesim_node
```

**sub_node側**

```bash
ros2 run my_package sub_node --ros-args -r pose:=/turtle1/pose
```

2つ目のウィンドウで（sub_node側）は、次のようなログコマンドが流れてきます。

```bash
pi:~/ros2_ws$ ros2 run my_package sub_node --ros-args -r pose:=/turtle1/pose
[INFO] [1669130545.834893426] [sub_node]: I heard: x: '5.544445', y: '5.544445'. r: '0.000000'
[INFO] [1669130545.851114887] [sub_node]: I heard: x: '5.544445', y: '5.544445'. r: '0.000000'
...
```

これはturltesimが出力した自己位置ですが、変化していません。（動いていないので当然ですね）

<br>

## プログラムの同時起動

カメを動かすために、前章のプログラムも同時に起動させてみましょう。

同時に複数のプログラムを起動する時は、launchファイルを作成・利用します。
ここでは、

- turtlesim_node
- my_node
- sub_node

を同時起動するように作成していきます。

<br>

### launchディレクトリの作成

まずは、launchディレクトリ・ファイルを作成して編集します。

```bash
mkdir ~/ros2_ws/src/my_package/launch
touch ~/ros2_ws/src/my_package/launch/turtlesim.launch.py
# code ~/ros2_ws/src/my_package/launch/turtlesim.launch.py
```

以下は、launchファイルの中身です。

> ROS2ではPythonのAPIを直接使用するのがメジャーですが、他にもxmlやyamlでも記述できます。

```python
#!/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = [
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),
        Node(
            package='my_package',
            executable='my_node',
            name='pub_node',
            remappings=[
                ('/cmd_vel', '/turtle1/cmd_vel'),
            ]
        ),
        Node(
            package='my_package',
            executable='sub_node',
            name='sub_node',
            remappings=[
                ('/pose', '/turtle1/pose'),
            ]
        )
    ]
    return LaunchDescription(nodes)
```

インストールのためにCMakeLists.txtにlaunchディレクトリを追記します。

```cmake
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
```

![](https://storage.googleapis.com/zenn-user-upload/3b212bf53b3a-20221123.png)

再びビルドします。

```bash
cd ~/ros2_ws
colcon build
```

### 実行

実行します。

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch my_package turtlesim.launch.py
```

移動しながら位置を出力しています。

![](https://storage.googleapis.com/zenn-user-upload/2ded3da159d5-20230121.png)

<br>