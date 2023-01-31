# Action（Client）

## 互換性

> ROS-ActionsのAPIはHumbleで変更されており、書き換えが必要です。

| Distro | チェック |
| --- | --- |
| Galactic | ✅ |
| Humble | ❌ |

<br>

## Action (Client)

次に、ROS Actionの実装を行ってみましょう。

Actionはゴール地点への操作とゴール到達の通知に至るまでの過程でフィードバックを送信し続ける処理に使うメッセージ形式です。ゴールを受け取りフィードバックおよび到達の通知を行う方がServer、操作を行う方をClientとして振る舞います。

Actionに似たシステムにServiceがありますが、Serviceは時刻同期を重視しています。

一方で、途中経過を送信できるActionは処理に時間がかかることを織り込み済みであるため、時間よりも送るコマンドと結果の整合性を重視する場合に役に立ちます。

また、Actionはゴールのキャンセル処理も行うことができます。


![](https://storage.googleapis.com/zenn-user-upload/54ea0884ef66-20221123.png)

<br>

## お試し

Actionを実例をもとに確認してみます。turtlesimには角度を操作するActionを用いた実装があります。

Actionのメッセージは以下のようになっています。

開き方（事前にROS環境を読み込んでください）

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash

cat /opt/ros/${ROS_DISTRO}/share/turtlesim/action/RotateAbsolute.action
```

次のような出力が出るとおもいます。

```c
# The desired heading in radians

float32 theta
---
# The angular displacement in radians to the starting position

float32 delta
---
# The remaining rotation in radians

float32 remaining
```

メッセージは`---`で区切られており、それぞれ入力・結果・経過（フィードバック）になっています。

> 結果にはbool型を入れるケースがあります。
> 経過に変数を入れないケースがあります。

<br>

RotateAbsolute.actionでは次のようなデータになっています。

- 入力 : 絶対角度 theta
- 結果 : 最終的な差分 delta
- 経過 : 残り角度 remaining

<br>

次は、コマンドを使用した操作を試してみましょう。
まずはターミナルを開いてturtlesimを立ち上げます。

```bash
ros2 run turtlesim turtlesim_node
```

次に角度を調整するコマンドを送ります。Actionの送信には`ros2 action send_goal`を使用します。
```bash
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: -1.57}" --feedback
```

成功すると、以下の画像のようにカメの角度がゆっくりと調整されていきます。目的の角度になったらResultの表示とともにコマンドが終了します。

![](https://storage.googleapis.com/zenn-user-upload/3278ecb03ff9-20230131.png)

<br>

## Clientの実装

先ほどCLIで操作したTurtlesimをC++プログラム上で動かしてみます。

```bash
touch ~/ros2_ws/src/my_package/src/action_client.cpp
# code ~/ros2_ws/src/my_package/src/action_client.cpp
```

使用したプログラムを以下に示します。

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <turtlesim/action/rotate_absolute.hpp>

class ActionClientClass : public rclcpp::Node
{
public:
    ActionClientClass() : Node("action_client")
    {
        using namespace std::placeholders;
        this->client_ptr_ = rclcpp_action::create_client<turtlesim::action::RotateAbsolute>(this, "rotate_absolute");
    }

    void send_goal(float target_rad)
    {
        using namespace std::placeholders;
        auto goal_msg = turtlesim::action::RotateAbsolute::Goal();
        goal_msg.theta = target_rad;
        auto send_goal_options = rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(&ActionClientClass::goal_response_callback, this, _1);
        send_goal_options.feedback_callback = std::bind(&ActionClientClass::feedback_callback, this, _1, _2);
        send_goal_options.result_callback = std::bind(&ActionClientClass::result_callback, this, _1);
        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::SharedPtr, const std::shared_ptr<const turtlesim::action::RotateAbsolute::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Feedback: %f", feedback->remaining);
    }

    void result_callback(const rclcpp_action::ClientGoalHandle<turtlesim::action::RotateAbsolute>::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal succeeded!");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                return;
        }
    }

    rclcpp_action::Client<turtlesim::action::RotateAbsolute>::SharedPtr client_ptr_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto action_client = std::make_shared<ActionClientClass>();
    action_client->send_goal(1.57);
    rclcpp::spin(action_client);
    rclcpp::shutdown();
    return 0;
}
```

通常のrclcpp::Nodeを継承した`action_client`内の`send_goal`メンバ関数を呼び出すことでactionサーバにゴールへの移動要求を出します。

以下のメンバ関数はactionの進捗などによって自動的に呼び出されます。
- goal_response_callback
- feedback_callback
- result_callback

<br>

### package.xmlの編集

```xml
<depend>rclcpp_action</depend>
```

<br>

### CMakeLists.txtの編集

12行目付近に以下を追加します。

```cmake
find_package(rclcpp_action REQUIRED)
```

次に42行目付近に以下を追加します。

```cmake

set(TARGET action_client)
add_executable(${TARGET} src/${TARGET}.cpp)
target_include_directories(${TARGET} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(${TARGET} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  ${TARGET}
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "turtlesim"
)
```

`install`のターゲットにも追加します。

```cmake
install(TARGETS my_node sub_node action_client
  DESTINATION lib/${PROJECT_NAME})
```

<br>


## ビルド・実行

ビルドします。

```bash
colcon build
```

実行してみましょう。実行に使用したコマンドを以下に示します。

actionはremapできないので、namespaceを追加することで対応しました。

`--ros-args -r __ns:=/<ネームスペース>`で付加することができます。

```bash
ros2 run my_package action_client --ros-args -r __ns:=/turtle1
```


<br>

## 参考資料

https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

https://docs.ros2.org/foxy/api/turtlesim/action/RotateAbsolute.html