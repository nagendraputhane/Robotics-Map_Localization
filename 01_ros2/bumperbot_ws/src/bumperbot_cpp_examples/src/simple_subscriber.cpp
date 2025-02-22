#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>

using std::placeholders::_1; //for std::bind

class SimpleSubscriber : public rclcpp::Node {
    public:
        SimpleSubscriber() : Node("simple_subscriber") {
            sub_ = create_subscription<std_msgs::msg::String>("chatter", 10, std::bind(&SimpleSubscriber::msgCallback, this, _1)); //ROS2 uses an executor with create_subscription() to notify the message to the callback function.
        }
    
    private:
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_; //Subscription template class takes a message type (std::msg::String) as a template parameter.
        void msgCallback(const std_msgs::msg::String &msg) const { // parameter const = function will not modify the contents of msg. // msgCallback is a constant member function. This means that the function does not modify any member variables of the class. //&msg so it need not make a copy of msg again.
            RCLCPP_INFO_STREAM(get_logger(), "I heard: " << msg.data.c_str());

        }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}