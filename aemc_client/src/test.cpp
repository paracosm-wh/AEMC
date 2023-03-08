////
//// Created by itr-wh on 23-1-4.
////
//# include "iostream"
//# include "Eigen/Dense"
//# include "vector"
//# include "algorithm"
//# include "set"
////# include "Aemc.h"
//# include "bitset"
//# include "list"
//
////struct Circle {
////    float radius;
////    Eigen::Vector3d center;
////};
////
////const std::vector<float> classArr{16.0, 20.0, 24.0, 28.0};
////const float ERR = 3.0;
////const int MinClusterNum = 3;
////const int CodedLENGTH = 7;
////
////double getCrossAngle(Eigen::Vector3d &vertex, Eigen::Vector3d &P1, Eigen::Vector3d &P2) {
////    Eigen::Vector3d v1 = P1 - vertex;
////    Eigen::Vector3d v2 = P2 - vertex;
////    double cosVal = v1.dot(v2) / (v1.norm() * v2.norm());
////    double angle = acos(cosVal);
////    return angle;
////}
////
////
////void GenBase() {
////    float radius = 28.0;
////    Eigen::Matrix<float, 7, 7> BaseMatrix;
////    Eigen::Vector<float, 6> BaseAngel;
//////    BaseMatrix.setZero();
////    int m = 0;
////    for (int i = 0; i < 7 / 2; ++i) {
////        BaseMatrix(m, m + 1) = 14.0 + 5.0 * i;
////        BaseMatrix(m + 1, m + 2) = 14.0 + 5.0 * i;
////        BaseAngel(m) = 2 * asin((14.0 + 5.0 * i) / (2 * radius));
////        BaseAngel(m + 1) = 2 * asin((14.0 + 5.0 * i) / (2 * radius));
////        m += 2;
////    }
////    for (int i = 0; i < 5; ++i) {
////        for (int j = 2; j < 7; ++j) {
////            float alpha = 0;
////            for (int k = 0; k < j - i; ++k) {
////                alpha += BaseAngel[i + k];
////            }
////            if (alpha > M_PI)
////                alpha = 2 * M_PI - alpha;
////            BaseMatrix(i, j) = 2 * radius * sin(alpha / 2);
////        }
////    }
////    std::cout << BaseMatrix << "\n";
////}
////
////std::set<int> sets_intersection(std::set<int> v1, std::set<int> v2) {
////    std::set<int> v;
////    set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), inserter(v, v.begin()));
////    return v;
////}
////
////std::vector<int> vectors_diff(std::vector<int> v1, std::vector<int> v2) {
////    std::vector<int> v;
////    std::set_difference(v1.begin(), v1.end(), v2.begin(), v2.end(), std::inserter(v, v.begin()));
////    return v;
////}
////
////std::set<int> sets_union(std::set<int> v1, std::set<int> v2) {
////    std::set<int> v;
////    std::set_union(v1.begin(), v1.end(), v2.begin(), v2.end(), inserter(v, v.begin()));//求交集
////    return v;
////}
////
////
////void FitCircle(Eigen::MatrixX3d &pts) {
////    auto num = pts.rows();
////    Eigen::MatrixXd L1 = Eigen::MatrixXd::Ones(num, 1);
////    Eigen::Vector3d A = (pts.transpose() * pts).inverse() * pts.transpose() * L1;
////
////    auto A1 = A;
////    A1.normalize();
////    printf("plane normal:%f,%f,%f\n", A1(0), A1(1), A1(2));
////
////    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num - 1, 3);
////
////    for (int i = 0; i < num - 1; i++) {
////        B.row(i) = pts.row(i + 1) - pts.row(i);
////    }
////
////    Eigen::MatrixXd L2 = Eigen::MatrixXd::Zero(num - 1, 1);
////    for (int i = 0; i < num - 1; i++) {
////        L2(i) = (pts(i + 1, 0) * pts(i + 1, 0) + pts(i + 1, 1) * pts(i + 1, 1) + pts(i + 1, 2) * pts(i + 1, 2)
////                 - (pts(i, 0) * pts(i, 0) + pts(i, 1) * pts(i, 1) + pts(i, 2) * pts(i, 2))) / 2.0;
////    }
////
////    Eigen::Matrix4d D;
////    D.setZero();
////    D.block<3, 3>(0, 0) = B.transpose() * B;
////    D.block<3, 1>(0, 3) = A;
////    D.block<1, 3>(3, 0) = A.transpose();
////
////    Eigen::Vector4d L3((B.transpose() * L2)(0), (B.transpose() * L2)(1), (B.transpose() * L2)(2), 1);
////    Eigen::Vector4d C = D.inverse() * L3;
////
////    float radius = 0;
////    for (int i = 0; i < num; i++) {
////        Eigen::Vector3f tmp(pts.row(i)(0) - C(0), pts.row(i)(1) - C(1), pts.row(i)(2) - C(2));
////        radius = radius + sqrt(tmp(0) * tmp(0) + tmp(1) * tmp(1) + tmp(2) * tmp(2));
////    }
////
////    radius = radius / num;
////    printf("radius:%f\n", radius);
////    printf("circle center:%f,%f,%f\n", C(0), C(1), C(2));
////    printf("lamda:%f\n", C(3));
////}
////int HammingDistance(int x, int y) {
////    int temp = x ^ y;
////    int count = 0;
////    while (temp != 0) {
////        ++count;
////        temp &= temp - 1;
////    }
////    return count;
////}
//void combine(int N, int M, std::vector<std::vector<int>> &com) {
//    std::vector<bool> v(N);
//    std::vector<int> temp;
//    for (int i = 0; i < N; ++i) {
//        v[i] = (i >= (N - M));
//    }
//    do {
//        for (int i = 0; i < N; ++i) {
//            if (v[i]) {
//                temp.push_back(i);
//            }
//        }
//        com.push_back(temp);
//        temp.clear();
//    } while (std::next_permutation(v.begin(), v.end()));
//}
//
//std::vector<int> vectors_intersection(std::vector<int> &v1, std::vector<int> &v2) {
//    std::sort(v1.begin(), v1.begin());
//    std::sort(v2.begin(), v2.begin());
//    std::vector<int> v;
//    set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));
//    return v;
//}
//
//
//int main() {
//    // 圆心计算
//    Eigen::MatrixX3d pts{
//            {956.986, -240.915, 18.4875},
//            {1001.56, -271.25,  20.042},
//            {982.258, -222.847, 17.0615},
//            {998.992, -227.577, 19.0445},
//            {955.953, -94.8195, 15.3322},
//            {989.091, -97.8391, 16.5985},
//            {974.247, -74.9731, 15.5335},
//            {963.884, -105.169, 14.4019}
//    };
////    Eigen::MatrixX3d pts2{{956.986, -240.915, 18.4875},
////                          {1001.56, -271.25, 20.042},
////                          {982.258, -222.847, 17.0615},
////                          {998.992, -227.577, 19.0445}};
////
////    RawMarker rm1(pts);
////    std::vector<std::vector<int>> potentialCluster_indices;
////    rm1.EuclideanCluster(potentialCluster_indices);
////    rm1.Fitting(potentialCluster_indices);
//
////    int a = 0;
////    std::vector<int> A = {0, 2, 3};
////    for (int &i: A) {
////        a += pow(2, i);
////    }
////    std::list<int> Codewords = {60, 90, 102, 23, 127, 113, 77, 43};
////    int true_codeword = 0;
////    for (auto c: Codewords) {
////        std::cout << HammingDistance(a, c) << std::endl;
////        if (HammingDistance(a, c) < 2)
////            true_codeword = c;
////    }
////    std::cout << std::bitset<7>(true_codeword) << "de" << true_codeword << std::endl;
////    std::cout << std::bitset<7>(a) << "de" << a << std::endl;
////    auto a = P(0, 1);
////    Circle c1 = Points3Fitting(c1, P1);
////    std::cout << c1.center << "\n";
////    std::cout << c1.radius;
//// 组合测试
////    std::vector<std::vector<int>> combination;
////    combine(5, 2, combination);
////    std::cout << combination.size();
//
////    std::vector<int> s = {1, 2};
////    std::vector<std::pair<float, std::vector<int>>> a;
////    a.emplace_back(2.3, s);
////    std::cout << a[0].first << a[0].second[0];
//
////    Eigen::Matrix<float, 7, 7> BaseMatrix;
////    GenBase();
////    std::cout << (BaseMatrix.array() - 20).array().abs();
//
//    std::vector<int> a{2, 1, 3};
//    std::vector<int> b{5, 1, 2};
//    std::sort(a.begin(), a.end());
//    std::sort(b.begin(), b.end());
////    std::cout << a.size();
//    auto inter = vectors_intersection(a, b);
////    auto un1 = sets_union(a, b);
////    std::set<std::pair<int, int>> c;
////    std::pair<int, int> s{1, 2};
////    c.insert(s);
////    c.insert({1, 2});
////    auto dif = vectors_diff(a, b);
////    std::vector<int> A{1, 2};
////    std::vector<int> B{1, 2};
////    if (A == B) {
////        std::cout << "true";
////    }
//// TODO 测试AEMC_I的编解码
//    return 0;
//}


//#include <chrono>
//#include <functional>
//#include <memory>
//#include <string>
//#include <fstream>
//#include <utility>
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
///* This example creates a subclass of Node and uses std::bind() to register a
//* member function as a callback from the timer. */
//
//class AemcRos {
//public:
//    AemcRos(rclcpp::Node::SharedPtr &node, std::string &path) : node(node), clock(node->get_clock()) {
//        path_ = path;
//        pointsPublisher_ = node->create_publisher<visualization_msgs::msg::Marker>("visual_points", 1);
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
////        markerPoints.lifetime = rclcpp::Duration();
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
////            point.x = 1.0f;
////            point.y = 1.0f;
////            point.z = 1.0f;
////            markerPoints.points.push_back(point);
//            for (int i = 0; i < static_cast<int>(fPoints.size()); i = i + 3) {
//                point.x = fPoints[i] / 1000;
//                point.y = fPoints[i + 1] / 1000;
//                point.z = fPoints[i + 2] / 1000;
////                point.x = 0.002f + i * 0.002f * frame;
////                point.y = 0.0002f + i * 0.002f * frame;
////                point.z = 0.0002f + i * 0.002f;
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
//
//class AemcPublisher : public rclcpp::Node {
//public:
//    AemcPublisher()
//            : Node("aemc_publisher"), count_(0) {
//        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 1);
////        timer_ = this->create_wall_timer(
////                1000ms, std::bind(&AemcPublisher::visual_callback, this));
//    }
//
//private:
//    void visual_callback(std::vector<float> &fPoints) {
//        visualization_msgs::msg::Marker markerPoints;
//        markerPoints.header.frame_id = "/raw_data";
//        markerPoints.header.stamp = now();
//        markerPoints.ns = "points";
//        markerPoints.id = 0;
//        markerPoints.type = visualization_msgs::msg::Marker::POINTS;
//        markerPoints.action = visualization_msgs::msg::Marker::ADD;
//
//        // Set the scale of the markerPoints -- 1x1x1 here means 1m on a side
//        markerPoints.scale.x = 0.1;
//        markerPoints.scale.y = 0.1;
//        markerPoints.scale.z = 0.1;
//
//        // Set the color -- be sure to set alpha to something non-zero!
//        markerPoints.color.r = 1.0f;
//        markerPoints.color.g = 0.0f;
//        markerPoints.color.b = 0.0f;
//        markerPoints.color.a = 1.0;
//
//        markerPoints.points.clear();
//        geometry_msgs::msg::Point p;
//        for (int i = 0; i < static_cast<int>(fPoints.size()); i = i + 3) {
//            p.x = fPoints[i];
//            p.y = fPoints[i + 1];
//            p.z = fPoints[i + 2];
//            markerPoints.points.push_back(p);
//        }
//        publisher_->publish(markerPoints);
//    }
//
//    rclcpp::TimerBase::SharedPtr timer_;
//    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
//    size_t count_;
//    std::vector<float> points_;
//};
//
//int main(int argc, char *argv[]) {
//    rclcpp::init(argc, argv);
//    std::string path = R"(/home/itr-wh/Work/Ros2_ws/src/data/rode1.csv)";
//    auto visual_node = std::make_shared<AemcPublisher>();
//    auto visual_node = std::make_shared<rclcpp::Node>("mocap_node");
//    AemcRos visualNode(visual_node, path);
//    visualNode.run_csv();

//    return 0;
//}



#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs//msg/path.hpp"
#include "aemc_msgs/msg/marker_array.hpp"
//#include "aemc_client/SeekerSDK.h"
#include "Aemc.h"

using namespace std::chrono_literals;

class AemcRos {
public:
    AemcRos(rclcpp::Node::SharedPtr &node) : node(node), clock(node->get_clock()) {
        visualPublisher_ = node->create_publisher<visualization_msgs::msg::MarkerArray>("visual", 1);
        markerPublisher_ = node->create_publisher<aemc_msgs::msg::MarkerArray>("markers", 10);
        pathPublisher_ = node->create_publisher<nav_msgs::msg::Path>("paths", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node);
    }

    void visualPointsMsg(visualization_msgs::msg::MarkerArray &marker_array, Eigen::MatrixX3d &raw_data) {
        visualization_msgs::msg::Marker markerPoints;
        markerPoints.header.frame_id = "/data";
        markerPoints.header.stamp = clock->now();
        markerPoints.ns = "points";
        markerPoints.id = 0;
        markerPoints.type = visualization_msgs::msg::Marker::POINTS;
        markerPoints.action = visualization_msgs::msg::Marker::ADD;

        // Set the scale of the markerPoints -- 1x1x1 here means 1m on a side
        markerPoints.scale.x = 0.01;
        markerPoints.scale.y = 0.01;
        markerPoints.scale.z = 0.01;

        // Set the color -- be sure to set alpha to something non-zero!
        markerPoints.color.r = 255;
        markerPoints.color.g = 0;
        markerPoints.color.b = 0;
        markerPoints.color.a = 1;

        markerPoints.points.clear();
        geometry_msgs::msg::Point point;
        for (int i = 0; i < static_cast<int>(raw_data.rows()); ++i) {
            point.x = raw_data(i, 0) / 1000;
            point.y = raw_data(i, 1) / 1000;
            point.z = raw_data(i, 2) / 1000;
            markerPoints.points.push_back(point);
        }
        marker_array.markers.push_back(markerPoints);


    }

    void
    visualClusterMsg(visualization_msgs::msg::MarkerArray &marker_array, std::vector<CodedCluster> &codedClusterList) {
        // points
        visualization_msgs::msg::Marker markerPoints;
        markerPoints.header.frame_id = "/data";
        markerPoints.header.stamp = clock->now();
        markerPoints.ns = "points";
        markerPoints.id = 1;
        markerPoints.type = visualization_msgs::msg::Marker::POINTS;
        markerPoints.action = visualization_msgs::msg::Marker::ADD;
        // Set the scale of the markerPoints -- 1x1x1 here means 1m on a side
        markerPoints.scale.x = 0.01;
        markerPoints.scale.y = 0.01;
        markerPoints.scale.z = 0.01;

        // Set the color -- be sure to set alpha to something non-zero!
        markerPoints.color.r = 255;
        markerPoints.color.g = 0;
        markerPoints.color.b = 0;
        markerPoints.color.a = 1;

        // text
        visualization_msgs::msg::Marker markerText;
        markerText.header.frame_id = "/data";
        markerText.header.stamp = clock->now();
        markerText.ns = "text";
        markerText.id = 2;
        markerText.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        markerText.action = visualization_msgs::msg::Marker::ADD;

        markerText.pose.orientation.w = 1.0;

        markerText.scale.z = 0.1;
        markerText.color.r = 255;
        markerText.color.g = 255;
        markerText.color.b = 255;
        markerText.color.a = 1;

        // Pose
        visualization_msgs::msg::Marker markerPose;
        markerPose.header.frame_id = "/data";
        markerPose.header.stamp = clock->now();
        markerPose.ns = "pose";
        markerPose.id = 3;
        markerPose.type = visualization_msgs::msg::Marker::ARROW;
        markerPose.action = visualization_msgs::msg::Marker::ADD;
        // Set the scale of the arrow
        markerPose.scale.x = 0.1;
        markerPose.scale.y = 0.005;
        markerPose.scale.z = 0.005;

        // Set the color -- be sure to set alpha to something non-zero!
        markerPose.color.r = 0;
        markerPose.color.g = 0;
        markerPose.color.b = 255;
        markerPose.color.a = 1;
        markerPoints.points.clear();

        aemc_msgs::msg::MarkerArray markerArray;
        markerArray.header.frame_id = "/aemc";
        markerArray.header.stamp = clock->now();
        for (auto codedCluster: codedClusterList) {
            if (codedCluster.Coded_flag) {
                // points
                geometry_msgs::msg::Point point;
                for (int i = 0; i < static_cast<int>(codedCluster.Points_.rows()); ++i) {
                    point.x = codedCluster.Points_(i, 0) / 1000;
                    point.y = codedCluster.Points_(i, 1) / 1000;
                    point.z = codedCluster.Points_(i, 2) / 1000;
                    markerPoints.points.push_back(point);
                }
                // code
                marker_array.markers.push_back(markerPoints);
                markerText.pose.position.x = codedCluster.Position[0] / 1000 + 0.1;
                markerText.pose.position.y = codedCluster.Position[1] / 1000 + 0.1;
                markerText.pose.position.z = codedCluster.Position[2] / 1000 + 0.1;
                markerText.text = std::to_string(codedCluster.Codeword_);
                marker_array.markers.push_back(markerText);
                // pose-arrow
                markerPose.pose.position.x = codedCluster.Position[0] / 1000;
                markerPose.pose.position.y = codedCluster.Position[1] / 1000;
                markerPose.pose.position.z = codedCluster.Position[2] / 1000;
                markerPose.pose.orientation.w = codedCluster.Pose.w();
                markerPose.pose.orientation.x = codedCluster.Pose.x();
                markerPose.pose.orientation.y = codedCluster.Pose.y();
                markerPose.pose.orientation.z = codedCluster.Pose.z();
                marker_array.markers.push_back(markerPose);
                // TF
                geometry_msgs::msg::TransformStamped t;
                t.header.stamp = clock->now();
                t.header.frame_id = "/data";
                t.child_frame_id = codedCluster.id;
                t.transform.translation.x = codedCluster.Position[0] / 1000;
                t.transform.translation.y = codedCluster.Position[1] / 1000;
                t.transform.translation.z = codedCluster.Position[2] / 1000;
                t.transform.rotation.w = codedCluster.Pose.w();
                t.transform.rotation.x = codedCluster.Pose.x();
                t.transform.rotation.y = codedCluster.Pose.y();
                t.transform.rotation.z = codedCluster.Pose.z();
                tf_broadcaster_->sendTransform(t);
                // marker
                aemc_msgs::msg::Marker marker;
                marker.header.frame_id = "/aemc";
                marker.header.stamp = clock->now();
                marker.id = codedCluster.id;
                marker.pose.position.x = codedCluster.Position[0] / 1000;
                marker.pose.position.y = codedCluster.Position[1] / 1000;
                marker.pose.position.z = codedCluster.Position[2] / 1000;
                marker.pose.orientation.w = codedCluster.Pose.w();
                marker.pose.orientation.x = codedCluster.Pose.x();
                marker.pose.orientation.y = codedCluster.Pose.y();
                marker.pose.orientation.z = codedCluster.Pose.z();
                marker.confidence = 1.0;
                markerArray.markers.push_back(marker);
            }

        }
        markerPublisher_->publish(markerArray);
    }

    void
    visualPathMsg(nav_msgs::msg::Path &path, std::vector<CodedCluster> &codedClusterList) {
        for (auto codedCluster: codedClusterList) {

            geometry_msgs::msg::PoseStamped points;
            // pose-arrow
            points.pose.position.x = codedCluster.Position[0] / 1000;
            points.pose.position.y = codedCluster.Position[1] / 1000;
            points.pose.position.z = codedCluster.Position[2] / 1000;
            points.pose.orientation.w = codedCluster.Pose.w();
            points.pose.orientation.x = codedCluster.Pose.x();
            points.pose.orientation.y = codedCluster.Pose.y();
            points.pose.orientation.z = codedCluster.Pose.z();
            points.header.stamp = clock->now();
            points.header.frame_id = "/data";
            path.poses.push_back(points);
        }
        pathPublisher_->publish(path);
    }

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualPublisher_;
    rclcpp::Publisher<aemc_msgs::msg::MarkerArray>::SharedPtr markerPublisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pathPublisher_;
private:
    rclcpp::Node::SharedPtr node;
    rclcpp::Clock::SharedPtr clock;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};


// Test :43, 90, 77 ---
//    Eigen::MatrixX3d point{
//            {956.986, -240.915, 18.4875},
//            {1001.56, -271.25,  20.042},
//            {982.258, -222.847, 17.0615},
//            {998.992, -227.577, 19.0445},
//            {955.953, -94.8195, 15.3322},
//            {989.091, -97.8391, 16.5985},
//            {974.247, -74.9731, 15.5335},
//            {963.884, -105.169, 14.4019}
//    };
//    // TODO :
//    //  1.读取trc/csv格式文件中的数据作为输入
//    //  2.可视化位姿
//    //  3.是否建立有效的跟踪：解码的结果与跟踪结果相融合
//
////     csv文件读取


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::string path = R"(/home/itr-wh/Work/Ros2_ws/src/data/track-1.csv)";
    auto visual_node = std::make_shared<rclcpp::Node>("mocap_node");
    AemcRos visualNode(visual_node);
    std::ifstream csv_data(path, std::ios::in);
    std::string line;
    if (!csv_data.is_open()) {
        std::cout << "Error: opening file fail" << std::endl;
        std::exit(1);
    }

    std::istringstream sin;
    std::string s;

    std::vector<double> points;
    std::vector<PreClusters> PreList;
    std::vector<PreTrace> PreTraceList;
    while (std::getline(csv_data, line)) {
        sin.clear();
        sin.str(line);
        points.clear();

        int col_num = 0;
        int64_t frame;
        while (std::getline(sin, s, ',')) //将字符串流sin中的字符读到field字符串中，以逗号为分隔符
        {
            if (!s.empty()) {
                if (col_num == 0)
                    frame = static_cast<int64_t>(std::stod(s));
                else if (col_num > 1)
                    points.push_back(std::stod(s));
            }
            col_num++;
        }
        if (!points.empty()) {
            Eigen::MatrixX3d rawData;
            int nPoints = static_cast<int>(points.size());
            rawData.resize(nPoints / 3, 3);
            for (int i = 0; i < nPoints; ++i) {
                rawData(i / 3, i % 3) = points[i];
            }

            RawMarker raw1(frame, rawData);
            // 数据处理
            visualization_msgs::msg::MarkerArray markerArray;
            visualNode.visualPointsMsg(markerArray, raw1.Raw_data_);
            std::vector<std::vector<int>> PotentialCluster_indices;
            raw1.EuclideanCluster(PotentialCluster_indices);
            std::vector<CodedCluster> codedClusterList;
            raw1.Fitting(PotentialCluster_indices, codedClusterList);
//            raw1.Track(PreList, codedClusterList);
            raw1.TraceTrack(PreTraceList, codedClusterList);
            visualNode.visualClusterMsg(markerArray, codedClusterList);
            visualNode.visualPublisher_->publish(markerArray);
//                visualNode.visualPathMsg(path1, codedClusterList);
            codedClusterList.clear();
        }
        std::cout << "The" << frame << "end" << std::endl;
    }
//    csv_data.close();
//    Eigen::Vector3d x{1, 0, 0};
//    Eigen::Vector3d y{0, 1, 0};
//    auto cross = x.cross(y);
//        auto sl = Z.dot(half.transpose());
//    Eigen::Quaternion<double> p = {x.dot(y.transpose()), cross[0], cross[1], cross[2]};
//    std::vector<std::string> a = {"1-2", "2-1"};
//    std::string x = "2-1";
//    std::cout << (std::find(a.begin(), a.end(), x) - a.begin());
    return 0;
}
