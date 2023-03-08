//
// Created by itr-wh on 22-8-13.
//

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "aemc_msgs/msg/marker_array.hpp"
#include "aemc_client/SeekerSDK.h"
#include "Eigen/Core"
#include "Aemc.h"

using namespace std::chrono_literals;

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

char TimeCodeMode[5][20] = {"None", "SMPTE", "Film", "EBU", "SystemClock"};

void Connect() {
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
}

int main() {
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
    while (true) {
        memset(&CapFrameOfData, 0, sizeof(sFrameOfData));   //clear
        pFrameOfData = Seeker_GetCurrentFrame();

        if (pFrameOfData != nullptr) {
            ret = Seeker_CopyFrame(pFrameOfData, &CapFrameOfData);
            curFrmNo = CapFrameOfData.iFrame;
            //-------------------- remove duplicate frames
            if (curFrmNo != preFrmNo) {
                if (curFrmNo != preFrmNo + 1) {
                    no_get_data += 1;
                    printf("已丢失%i. Frame %i \n", no_get_data, curFrmNo);
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
                std::vector<std::vector<int>> PotentialCluster_indices;
                raw1.EuclideanCluster(PotentialCluster_indices);
                std::vector<CodedCluster> codedClusterList;
                raw1.Fitting(PotentialCluster_indices, codedClusterList);


                std::cout << "----" << curFrmNo << std::endl;
            }
        } else {
            printf("未获取到当前帧动捕数据.\n");
        }
        Seeker_FreeFrame(&CapFrameOfData);
    }
}