//
// Created by itr-wh on 23-2-27.
//
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "aemc_msgs/msg/marker_array.hpp"   // CHANGE
#include "vector"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
    MinimalSubscriber()
            : Node("minimal_subscriber") {
        subscription_ = this->create_subscription<aemc_msgs::msg::MarkerArray>(          // CHANGE
                "markers", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const aemc_msgs::msg::MarkerArray::SharedPtr msg) const       // CHANGE
    {
//        RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->markers);              // CHANGE
        std::cout << msg->markers[0].pose.position.x << std::endl;
    }

    rclcpp::Subscription<aemc_msgs::msg::MarkerArray>::SharedPtr subscription_;       // CHANGE
};

int main(int argc, char *argv[]) {
//    rclcpp::init(argc, argv);
//    rclcpp::spin(std::make_shared<MinimalSubscriber>());
//    rclcpp::shutdown();
    std::vector<std::vector<int>> vec;
    for (int i = 0; i < 10; ++i) {
        std::vector<int> x{i, i + 1};
        vec.push_back(x);
    }
    for (auto v: vec) {
        v.erase(v.begin());
        std::cout << v.size()<<std::endl;
    }
    return 0;
}