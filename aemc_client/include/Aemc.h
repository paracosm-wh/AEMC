//
// Created by itr-wh on 22-12-29.
//

#ifndef AEMC_CLIENT_AEMC_H
#define AEMC_CLIENT_AEMC_H

//# include "Eigen/Core"
# include "Eigen/Dense"

# include "vector"
# include "list"
# include "map"


//const std::vector<float> classArr{17.29, 42.0, 50.0, 28.0};
const std::vector<float> classArr{0.0, 42.0, 50.0, 28.0};
const float ERR = 4.0;
const int MinClusterNum = 3;
const int CodedLENGTH = 7;

struct Circle {
    float radius;
    Eigen::Vector3d center;
};

struct CodedCluster {
    Eigen::MatrixX3d Points_;
    bool Coded_flag = false;
    int Codeword_;
    std::string id;
    Eigen::Vector3d Position;
    Eigen::Quaternion<double> Pose;
};

struct PreClusters {
    std::string id;
    std::vector<int64_t> frames;
    std::vector<Eigen::Vector3d> Positions;
};

struct PreTrace {
    std::vector<std::string> ids;
    std::vector<int64_t> frames;
    std::vector<Eigen::Vector3d> Positions;
};

//将每个点赋予位置，无帧头赋予顺序，有帧头赋予位置
//Explicit and Implicit headers

class AEMC_E {
/* Explicit clusters
 * Properties:
 * * CodeLength
 * * Radius
 * * Codewords
 * * Chord_matrix
 * Functions:
 * * GenCodewords
 * * GenChordMatrix
 *
 */
private:
    int CodeLength_;
    float Radius_;
    std::vector<int> Codewords;
    Eigen::Matrix<float, 7, 7> Chord_matrix;

    void GenCodewords();

    void GenChordMatrix();

public:
    AEMC_E(int CodeLength, int classBit);

    Eigen::MatrixXf GetChordMatrix();

    std::vector<int> GetCodewords();

};

/* Implicit clusters
 * Properties:
 * * CodeLength
 * * Radius
 * * Codewords
 * *
 * Functions:
 * * GenCodewords
 * * GetRadius
 * * Decoding
 */
class AEMC_I {
private:
    int CodeLength_;   // Length of codewords
    float Radius_;
    std::vector<int> Codewords;
    std::vector<float> Base_chords;

    void GenCodewords();

    void GenBase();

public:
    AEMC_I(int CodeLength, int classIndices);

    std::vector<float> &GetBaseChords();

    std::vector<int> GetCodewords();
};


class RawMarker {
private:
    int raw_num; //原始点数量
//    int code_num; //CodedMarker数量
    Eigen::VectorXf Uncoded_marker;
    std::vector<CodedCluster> Coded_marker;
public:
    int64_t frame_;
    Eigen::MatrixX3d Raw_data_;

    // Constructor
    RawMarker(int64_t frame, Eigen::MatrixX3d &Raw_data);

    /* 聚类方法
     * input --> All of points
     * output -->
     */
    void EuclideanCluster(std::vector<std::vector<int>> &PotentialCluster_indices);

    bool RadiusSearch(int sq, float radius, std::vector<bool> &process, std::vector<int> &k_indices) const;

    /* Decoding
     * input -->
     * output -->
     */
    void AEMC_I_Decoding(int class_indices, std::vector<int> &cluster_indices, Circle &c, CodedCluster &codedCluster);

    void AEMC_E_Decoding(int class_indices, std::vector<int> &cluster_indices, CodedCluster &codedCluster);

    void areInMatrix(const std::vector<int> &, const Eigen::MatrixXf &,
                     std::vector<std::pair<std::vector<int>, std::vector<int>>> &);

    int bestFit(std::vector<std::pair<std::vector<int>, std::vector<int>>> &ordered_indices,
                const Eigen::MatrixXf &chordsMatrix);


    /* Fitting
     * input -->
     * output -->
     * 基于RANSAC的圆形滤波
     * 可继续使用处理标志列表
     * 先判断是否处理
     * 1. 采样三个点判断半径是否在设定范围内，
     * 2. 如果在，判断其他点到这个圆的距离是否在范围内，生成一个可解码的聚类，
     * 若不在，更换采样点，重复1，2
     */
    void
    Fitting(const std::vector<std::vector<int>> &PotentialCluster_indices, std::vector<CodedCluster> &CodedClusterList);

    void Track(std::vector<PreClusters> &preClusterList, std::vector<CodedCluster> &curCodedClusterList) const;

    void TraceTrack(std::vector<PreTrace> &preTrace, std::vector<CodedCluster> &curCodedClusterList);
};

bool filter(std::vector<Eigen::Vector3d> &pre, Eigen::Vector3d &cur);

bool filter2(PreClusters &pre, Eigen::Vector3d &cur, int64_t frame);

void GenPose(Eigen::MatrixX3d &pts, Eigen::Quaternion<double> &Pose);

void GenTransform(Eigen::MatrixX3d &pts, Eigen::Quaternion<double> &Pose, Eigen::Vector3d &position);

const Circle &Points3Fitting(Circle &cir1, Eigen::Matrix<double, 3, 3> &points);

const Circle &LeastSquareFitting(Eigen::MatrixX3d);

void combine(int N, int M, std::vector<std::vector<int>> &com);

bool areRanges(float value, const std::vector<float> &targets, float error, int &class_idx);

double getCrossAngle(Eigen::Vector3d &vertex, Eigen::Vector3d &P1, Eigen::Vector3d &P2);

bool cmp(const std::pair<float, std::vector<int>> &, const std::pair<float, std::vector<int>> &);

bool cmp1(const std::pair<double, int> &a, const std::pair<double, int> &b);

std::vector<int> vectors_intersection(std::vector<int> v1, std::vector<int> v2);

std::vector<int> vectors_diff(std::vector<int> v1, std::vector<int> v2);

int cyclicShift(int value, int d, int N);

int HammingDistance(int x, int y);

Circle FitCircle(Eigen::MatrixX3d &pts);

#endif //AEMC_CLIENT_AEMC_H
