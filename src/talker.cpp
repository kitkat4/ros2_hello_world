#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <iostream>

using namespace std::chrono_literals;


class Talker : public rclcpp::Node{
public:
    explicit Talker(const std::string & topic_name)
        : Node("talker"){
        
        msg_ = std::make_shared<std_msgs::msg::String>();
        msg_->data = "Hello world!";

        // ラムダ式
        auto publish_message = [this]() -> void {
            RCLCPP_INFO(this->get_logger(), "%s", msg_->data.c_str());
            pub_->publish(*msg_);
        };

        // 10 が無難と https://index.ros.org//doc/ros2/Releases/Release-Dashing-Diademata/ に書いてある
        pub_ = create_publisher<std_msgs::msg::String>(topic_name, 10); 

        timer_ = create_wall_timer(100ms, publish_message);
    }

private:

    std::shared_ptr<std_msgs::msg::String> msg_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
};

int main(int argc, char* argv[]){

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Talker>("chatter");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

