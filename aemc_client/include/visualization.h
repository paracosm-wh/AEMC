////
//// Created by itr-wh on 23-2-13.
////
//
//#ifndef AEMC_CLIENT_VISUALIZATION_H
//#define AEMC_CLIENT_VISUALIZATION_H
//#include <chrono>
//#include <functional>
//#include <memory>
//#include <string>
//#include <fstream>
//#include "rclcpp/rclcpp.hpp"
//#include "visualization_msgs/msg/marker.hpp"
//
//#include "geometry_msgs/msg/vector3.hpp"
//#include "std_msgs/msg/string.hpp"
//#include "aemc_client/SeekerSDK.h"
//#include "Eigen/Core"
//#include "Aemc.h"
//
//using namespace std::chrono_literals;
//
//class AemcRos {
//public:
//    AemcRos(rclcpp::Node::SharedPtr &node, std::string &path) : node(node), clock(node->get_clock()) {
//        path_ = path;
//        pointsPublisher_ = node->create_publisher<visualization_msgs::msg::Marker>("visual_points", 1);
//    }
//
//    void visualPointsMsg(visualization_msgs::msg::Marker &markerPoints, RawMarker &rawMarker) {
//        markerPoints.header.frame_id = "/data";
//        markerPoints.header.stamp = clock->now();
//        markerPoints.ns = "points";
//        markerPoints.id = 0;
//        markerPoints.type = visualization_msgs::msg::Marker::POINTS;
//        markerPoints.action = visualization_msgs::msg::Marker::ADD;
//
//        // Set the scale of the markerPoints -- 1x1x1 here means 1m on a side
//        markerPoints.scale.x = 0.01;
//        markerPoints.scale.y = 0.01;
//        markerPoints.scale.z = 0.01;
//
//        // Set the color -- be sure to set alpha to something non-zero!
//        markerPoints.color.r = 255;
//        markerPoints.color.g = 0;
//        markerPoints.color.b = 0;
//        markerPoints.color.a = 1;
//
//        markerPoints.points.clear();
//        geometry_msgs::msg::Point point;
//        for (int i = 0; i < static_cast<int>(rawMarker.Raw_data_.rows()); ++i) {
//            point.x = rawMarker.Raw_data_(i, 0) / 1000;
//            point.y = rawMarker.Raw_data_(i, 1) / 1000;
//            point.z = rawMarker.Raw_data_(i, 2) / 1000;
//            markerPoints.points.push_back(point);
//        }
//    }
//
//    void visualTextMsg(visualization_msgs::msg::Marker &markerText, CodedCluster &codedCluster) {
//        markerText.header.frame_id = "/data";
//        markerText.header.stamp = clock->now();
//        markerText.ns = "text";
//        markerText.id = 1;
//        markerText.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
//        markerText.action = visualization_msgs::msg::Marker::ADD;
//
//        markerText.pose.orientation.w = 1.0;
//
//        markerText.scale.z = 0.1;
//        markerText.color.r = 0;
//        markerText.color.g = 0;
//        markerText.color.b = 255;
//        markerText.color.a = 1;
//
//        markerText.pose.position.x = codedCluster.Position[0] / 1000 + 0.1;
//        markerText.pose.position.x = codedCluster.Position[1] / 1000 + 0.1;
//        markerText.pose.position.x = codedCluster.Position[2] / 1000 + 0.1;
//
//        std::ostringstream str;
//        str << codedCluster.Codeword_;
//        markerText.text = str.str();
//    }
//
//    void visualPoseMsg() {
//
//    }
//
//    void run_csv() {
//        visualization_msgs::msg::Marker markerPoints;
//        markerPoints.header.frame_id = "/raw_data";
//        markerPoints.header.stamp = clock->now();
//        markerPoints.ns = "points";
//        markerPoints.id = 0;
//        markerPoints.type = visualization_msgs::msg::Marker::POINTS;
//        markerPoints.action = visualization_msgs::msg::Marker::ADD;
//
//        // Set the scale of the markerPoints -- 1x1x1 here means 1m on a side
//        markerPoints.scale.x = 0.01;
//        markerPoints.scale.y = 0.01;
//        markerPoints.scale.z = 0.01;
//
//        // Set the color -- be sure to set alpha to something non-zero!
//        markerPoints.color.r = 1.0f;
//        markerPoints.color.g = 0.0f;
//        markerPoints.color.b = 0.0f;
//        markerPoints.color.a = 1.0;
//
//        // read-csv
//        std::ifstream csv_data(path_, std::ios::in);
//        std::string line;
//        if (!csv_data.is_open()) {
//            std::cout << "Error: opening file fail" << std::endl;
//            std::exit(1);
//        }
//        std::istringstream sin;
//        std::string s;
//
//        std::vector<float> fPoints;
//
//        while (rclcpp::ok() && std::getline(csv_data, line)) {
//
//            markerPoints.points.clear();
//            sin.clear();
//            sin.str(line);
//            fPoints.clear();
//
//            int col_num = 0;
//            int64_t frame;
//            while (std::getline(sin, s, ',')) //将字符串流sin中的字符读到field字符串中，以逗号为分隔符
//            {
//                if (!s.empty()) {
//                    if (col_num == 0)
//                        frame = static_cast<int64_t>(std::stod(s));
//                    else if (col_num > 1)
//                        fPoints.push_back(std::stof(s));
//                }
//                col_num++;
//            }
//            geometry_msgs::msg::Point point;
//            for (int i = 0; i < static_cast<int>(fPoints.size()); i = i + 3) {
//                point.x = fPoints[i] / 1000;
//                point.y = fPoints[i + 1] / 1000;
//                point.z = fPoints[i + 2] / 1000;
//                markerPoints.points.push_back(point);
//            }
//            pointsPublisher_->publish(markerPoints);
//            usleep(16666);
//            std::cout << frame << std::endl;
//        }
//    }
//
//private:
//    rclcpp::Node::SharedPtr node;
//    rclcpp::Clock::SharedPtr clock;
//    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pointsPublisher_;
//    std::string path_;
//};
//#endif //AEMC_CLIENT_VISUALIZATION_H
