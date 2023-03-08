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
#include "aemc_client/SeekerSDK.h"
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

void writeResult(std::string path, CodedCluster &codedCluster) {

}

void MyErrorMsgHandler(int iLevel, char *szMsg) {
    std::string szLevel;

    if (iLevel == VL_Debug) {
        szLevel = "Debug";
    } else if (iLevel == VL_Info) {
        szLevel = "Info";
    } else if (iLevel == VL_Warning) {
        szLevel = "Warning";
    } else if (iLevel == VL_Error) {
        szLevel = "Error";
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    std::string path = R"(/home/itr-wh/Work/Ros2_ws/src/data/human_upper.csv)";
    auto visual_node = std::make_shared<rclcpp::Node>("mocap_node");
    AemcRos visualNode(visual_node);

    sHostInfo Seeker_HostInfo;
    Seeker_SetErrorMsgHandlerFunc(MyErrorMsgHandler);
    int ret = Seeker_Initialize("10.1.1.101", "10.1.1.198");
    if (ret != RC_Okay) {
        setlocale(LC_ALL, "");
        RCLCPP_INFO(rclcpp::get_logger("connect"), "Failed to initialize the network.\n\n");
    } else {
        setlocale(LC_ALL, "");
        RCLCPP_INFO(rclcpp::get_logger("connect"), "Successfully initialized the network.\n\n");
    }

    int ret1 = Seeker_GetHostInfo(&Seeker_HostInfo);   // Get the information of server.
    if (ret1 != RC_Okay || !Seeker_HostInfo.bFoundHost) {
        RCLCPP_INFO(rclcpp::get_logger("connect"), "Failed to connect to the server.\n\n");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("connect"), "Successfully connected to the server.\n\n");
    }

    int preFrmNo = 0;
    int curFrmNo = 0;
    int no_get_data = 0;
    sFrameOfData *pFrameOfData = nullptr;
    sFrameOfData CapFrameOfData;       //motion data

    Eigen::MatrixX3d points;
    nav_msgs::msg::Path path1;
    std::vector<PreClusters> PreList;
    while (true) {
        memset(&CapFrameOfData, 0, sizeof(sFrameOfData));   //clear
        pFrameOfData = Seeker_GetCurrentFrame();

        if (pFrameOfData != nullptr) {
            ret = Seeker_CopyFrame(pFrameOfData, &CapFrameOfData);
            std::chrono::milliseconds rec_t = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now().time_since_epoch());
            curFrmNo = CapFrameOfData.iFrame;
            //-------------------- remove duplicate frames
            if (curFrmNo != preFrmNo) {
                if (curFrmNo != preFrmNo + 1) {
                    no_get_data += 1;
//                    printf("已丢失%i. Frame %i \n", no_get_data, curFrmNo);
                }
                preFrmNo = curFrmNo;

                for (int i = 0; i < CapFrameOfData.nUnidentifiedMarkers; i++) //输出未识别的Marker点坐标
                {
                    points.resize(CapFrameOfData.nUnidentifiedMarkers, 3);
                    points(i, 0) = CapFrameOfData.UnidentifiedMarkers[i][0];
                    points(i, 1) = CapFrameOfData.UnidentifiedMarkers[i][1];
                    points(i, 2) = CapFrameOfData.UnidentifiedMarkers[i][2];
                }
                RawMarker raw1(curFrmNo, points);
                // CodedCluster visualization
                visualization_msgs::msg::MarkerArray markerArray;
                visualNode.visualPointsMsg(markerArray, raw1.Raw_data_);
                std::vector<std::vector<int>> PotentialCluster_indices;
                raw1.EuclideanCluster(PotentialCluster_indices);
                std::vector<CodedCluster> codedClusterList;
                raw1.Fitting(PotentialCluster_indices, codedClusterList);

                raw1.Track(PreList, codedClusterList);
                visualNode.visualClusterMsg(markerArray, codedClusterList);
                visualNode.visualPublisher_->publish(markerArray);
//                visualNode.visualPathMsg(path1, codedClusterList);
                codedClusterList.clear();
                std::chrono::milliseconds send_t = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::system_clock::now().time_since_epoch());
                long delta = (send_t.count() - rec_t.count());
                std::cout << "----" << curFrmNo << "-delta:" << delta << "us" << std::endl;
            }
        } else {
            printf("未获取到当前帧动捕数据.\n");
        }
        Seeker_FreeFrame(&CapFrameOfData);
    }
}
