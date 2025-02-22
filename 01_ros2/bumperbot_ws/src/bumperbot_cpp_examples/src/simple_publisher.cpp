#include<rclcpp/rclcpp.hpp>
#include<std_msgs/msg/string.hpp>

using namespace std::chrono_literals; //for 1s

class SimplePublisher : public rclcpp::Node { //class = node in rclcpp
    public:
        SimplePublisher() : Node("simple_publisher"), counter_(0) { //initialize the base class's constructor and member variables of this class
        pub_ = create_publisher<std_msgs::msg::String>("chatter", 10); //10 = queue size (buffer). //functions that create or manage objects (like Publisher, Subscription, etc.) often return shared pointers.
        //informing the middleware that this publisher will only send messages of that type (std_msgs::msg::String) on the topic "chatter".
        timer_ = create_wall_timer(1s, std::bind(&SimplePublisher::timer_callback, this)); //bind timer_callback of &SimplePublisher to the parameters of create_wall_timer and pass the object ('this') to indicate which object's timer_callback to call.
        //when 1s passes, create_wall_timer timerCallback function is called. 

        RCLCPP_INFO(get_logger(), "publishing at 1Hz");
        }
    private:
        unsigned int counter_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_; //Publisher is a template class.
        rclcpp::TimerBase::SharedPtr timer_;

        void timer_callback() {
            auto message = std_msgs::msg::String();
            message.data = "Hello ROS 2 - counter: " + std::to_string(counter_++);
            pub_->publish(message); //publish is a member function of Publisher class. as pub_ is a shared pointer, use -> to access its member functions.
        }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv); //without calling rclcpp:init(), the node will not be able to communicate with the ROS 2 middleware.
    auto node = std::make_shared<SimplePublisher>();
    rclcpp::spin(node); //spin() is a blocking call that will keep the node alive until a SIGINT signal is received.
    rclcpp::shutdown(); //shutdown the node.
    return 0;
}

