#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <iostream>


class Listener : public rclcpp::Node{
    
public:
    
    explicit Listener(const std::string& topic_name)
        : Node("listener"){

        auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void {
            RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
        };

        // 1 は queue size. rmw_qos_profile_t 型オブジェクトを渡すこともできるようだ．
        sub_ = create_subscription<std_msgs::msg::String>(topic_name, 1, callback);
    }

private:

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};


int main(int argc, char* argv[]){

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Listener>("chatter");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}


