//
// Created by itr-wh on 22-12-29.
//
# include "iostream"
# include "cmath"
# include "Aemc.h"
#include <cmath>
# include "list"


AEMC_E::AEMC_E(int CodeLength, int classBit) {
    Radius_ = classArr[classBit];
    CodeLength_ = CodeLength;
    GenChordMatrix();
    GenCodewords();
}

void AEMC_E::GenCodewords() {
    Codewords = {30, 106, 89, 45, 51, 71, 116};
//    Codewords = {60, 90, 102, 23, 127, 113, 77, 43};
}

//TD 如何生成参考矩阵
void AEMC_E::GenChordMatrix() {
    Chord_matrix.setZero();
    Eigen::Vector<float, 6> baseAngel;

    int m = 0;
    for (int i = 0; i < 7 / 2; ++i) {
        Chord_matrix(m, m + 1) = 14.0 + 5.0 * i;
        Chord_matrix(m + 1, m + 2) = 14.0 + 5.0 * i;
        baseAngel(m) = 2 * asin((14.0 + 5.0 * i) / (2 * Radius_));
        baseAngel(m + 1) = 2 * asin((14.0 + 5.0 * i) / (2 * Radius_));
        m += 2;
    }
    for (int i = 0; i < 5; ++i) {
        for (int j = 2; j < 7; ++j) {
            float alpha = 0;
            for (int k = 0; k < j - i; ++k) {
                alpha += baseAngel[i + k];
            }
            if (alpha > M_PI)
                alpha = 2 * M_PI - alpha;
            Chord_matrix(i, j) = 2 * Radius_ * sin(alpha / 2);
        }
    }
}

std::vector<int> AEMC_E::GetCodewords() {
    return Codewords;
}

Eigen::MatrixXf AEMC_E::GetChordMatrix() {
    return Chord_matrix;
}

AEMC_I::AEMC_I(int CodeLength, int classBit) {
    //TODO:添加异常处理
    Radius_ = classArr[classBit];
    CodeLength_ = CodeLength;
    GenBase();
    GenCodewords();
}

// TODO 生成codewords
void AEMC_I::GenCodewords() {
    Codewords = {45, 51, 116};
}

void AEMC_I::GenBase() {
    for (int i = 1; i < ((static_cast<float>(CodeLength_) + 1) / 2); ++i)
        Base_chords.push_back(static_cast<float>(2 * Radius_ * sin(i * M_PI / CodeLength_)));
    std::sort(Base_chords.begin(), Base_chords.end());
}


std::vector<float> &AEMC_I::GetBaseChords() {
    return Base_chords;
}

std::vector<int> AEMC_I::GetCodewords() {
    return Codewords;
}

RawMarker::RawMarker(int64_t frame, Eigen::MatrixX3d &Raw_data) {
    frame_ = frame;
    Raw_data_ = Raw_data;
    raw_num = static_cast<int>(Raw_data.rows());
}


/* Method: Euclidean Cluster
 * Para: cluster_indices: Storing indices for each cluster
 */
void RawMarker::EuclideanCluster(std::vector<std::vector<int>> &PotentialCluster_indices) {
    float radius_err = 2 * classArr.back() + ERR;
    std::vector<bool> processed(Raw_data_.rows(), false);   // Flags for clustering processing

    // Process all points in the indices vector
    for (int i = 0; i < raw_num; ++i) {
        if (processed[i])
            continue;

        std::vector<int> seed_queue;    // Storing indices for a cluster
        int sq_idx = 0;
        seed_queue.push_back(i);
        processed[i] = true;

        while (sq_idx < static_cast<int>(seed_queue.size())) {
            if (!RadiusSearch(seed_queue[sq_idx], radius_err, processed, seed_queue)) {
                sq_idx++;
                continue;
            }
        }
        if (seed_queue.size() >= MinClusterNum) {
            PotentialCluster_indices.push_back(seed_queue);
        }
    }
}

bool RawMarker::RadiusSearch(int seed_idx, float radius, std::vector<bool> &process,
                             std::vector<int> &k_indices) const {
    int flag_num = 0;
    for (int i = 0; i < raw_num; ++i) {
        if (!process[i]) {
            auto dis = (Raw_data_.row(i) - Raw_data_.row(seed_idx)).norm();
            if (static_cast<float>(dis) < radius) {
                k_indices.push_back(i);
                process[i] = true;
                flag_num++;
            }
        }
    }
    if (flag_num == 0)
        return false;
    else
        return true;
}

void RawMarker::AEMC_I_Decoding(int class_indices, std::vector<int> &cluster_indices, Circle &c,
                                CodedCluster &codedCluster) {
    AEMC_I I_cluster(CodedLENGTH, class_indices);
    auto base_chords = I_cluster.GetBaseChords();
    auto codewords = I_cluster.GetCodewords();
    // Calculate all chords
    std::vector<std::pair<float, std::vector<int>>> chords;
    int cluster_num = static_cast<int>(cluster_indices.size());
    for (int i = 0; i < cluster_num; ++i) {
        for (int j = (i + 1); j < cluster_num; ++j) {
            std::vector<int> a = {cluster_indices[i], cluster_indices[j]};
            chords.emplace_back(static_cast<float> ((Raw_data_.row(cluster_indices[j]) -
                                                     Raw_data_.row(cluster_indices[i])).norm()), a);
        }
    }
    std::sort(chords.begin(), chords.end(), cmp);
    // Determine the coding position of the points
    bool first_flag = true;
    int base_flag;
    int target_flag;
    std::pair<std::vector<int>, std::vector<int>> cluster_order;
    for (auto &chord: chords) {
        if (first_flag) {
            if (areRanges(chord.first, base_chords, ERR, base_flag)) {
                //Determining clockwise and counterclockwise
                std::vector<double> c_0 = {Raw_data_(chord.second[0], 0) - c.center[0],
                                           Raw_data_(chord.second[0], 1) - c.center[1]};
                std::vector<double> c_1 = {Raw_data_(chord.second[1], 0) - c.center[0],
                                           Raw_data_(chord.second[1], 1) - c.center[1]};
                auto cross_c = c_0[0] * c_1[1] - c_1[0] * c_0[1];

                if (cross_c < 0) {
                    cluster_order.first.push_back(chord.second[0]);
                    cluster_order.first.push_back(chord.second[1]);
                } else {
                    cluster_order.first.push_back(chord.second[1]);
                    cluster_order.first.push_back(chord.second[0]);
                }
                cluster_order.second.push_back(0);
                base_flag = base_flag + 1;
                cluster_order.second.push_back(base_flag);
                first_flag = false;
            }
        } else {
            auto vertex = vectors_intersection(chord.second, cluster_order.first);
            if (static_cast<int>(vertex.size()) == 1) {
                if (areRanges(chord.first, base_chords, ERR, target_flag)) {
                    Eigen::Vector3d p0 = Raw_data_.row(cluster_order.first[0]).transpose();
                    Eigen::Vector3d p1 = Raw_data_.row(cluster_order.first[1]).transpose();
                    auto third = vectors_diff(chord.second, vertex);
                    Eigen::Vector3d p2 = Raw_data_.row(third[0]).transpose();
                    double angel = getCrossAngle(p0, p1, p2);
                    double base_angel = M_PI / CodedLENGTH;
                    int x = static_cast<int>(round(angel / base_angel));
                    cluster_order.first.push_back(third[0]);
//                    std::vector<double> t_0 = {p0[0] - c.center[0], p0[1] - c.center[1]};
//                    std::vector<double> t_1 = {p2[0] - c.center[0], p2[1] - c.center[1]};
//                    auto cross_t = t_0[0] * t_1[1] - t_1[0] * t_0[1];
//                    if (cross_t > 0) {
                    cluster_order.second.push_back(base_flag + x);
//                    } else {
//                        cluster_order.second.push_back(CodedLENGTH - 1 - x);
//                    }
                }
            }
//            if (chord.second[0] == cluster_order[0].first || chord.second[1] == cluster_order[0].first) {
//                if (areRanges(chord.first, base_chords, ERR, target_flag)) {
//                    Eigen::Vector3d p0 = Raw_data_.row(cluster_order[0].first).transpose();
//                    Eigen::Vector3d p1 = Raw_data_.row(cluster_order[1].first).transpose();
//                    Eigen::Vector3d p2 = Raw_data_.row(cluster_indices[chord.second[1]]).transpose();
//                    double angel = getCrossAngle(p0, p1, p2);
//                    double base_angel = M_PI / CodedLENGTH;
//                    int x = static_cast<int>(round(angel / base_angel));
//                    cluster_order.emplace_back(cluster_indices[chord.second[1]], base_flag + x);
//                }
//            }
        }
    }
    int decode_flag = 0;
    int codeword = 0;
    for (int &p: cluster_order.second) {
        codeword += pow(2, CodedLENGTH - 1 - p);
    }

    for (int i = 0; i < CodedLENGTH; ++i) {
        int codeword_c = cyclicShift(codeword, i, CodedLENGTH);
        for (auto t: codewords) {
            if (HammingDistance(codeword_c, t) < 2) {
                codedCluster.Codeword_ = t;
                decode_flag = 1;
                break;
            }
        }
        if (decode_flag)
            break;
    }
    if (decode_flag) {
        codedCluster.Points_.resize(cluster_order.first.size(), 3);
        int row_i = 0;
        for (auto p: cluster_order.first) {
            codedCluster.Points_.row(row_i) = Raw_data_.row(p);
            row_i++;
        }
        codedCluster.Coded_flag = true;
        auto codeword_indices =
                std::find(codewords.begin(), codewords.end(), codedCluster.Codeword_) - codewords.begin();
        codedCluster.id = std::to_string(class_indices) + '-' + std::to_string(codeword_indices);
        codedCluster.Position = FitCircle(codedCluster.Points_).center;
        GenTransform(codedCluster.Points_, codedCluster.Pose, codedCluster.Position);
//        GenPose(codedCluster.Points_, codedCluster.Pose);
        std::cout << "I:" << codedCluster.Codeword_ << "/";
    } else {
        codedCluster.Coded_flag = false;
        int row_i = 0;
        codedCluster.Points_.resize(cluster_indices.size(), 3);
        for (auto p: cluster_indices) {
            codedCluster.Points_.row(row_i) = Raw_data_.row(p);
            row_i++;
        }
        codedCluster.Position = FitCircle(codedCluster.Points_).center;
        codedCluster.id = "fail";
    }
}

void RawMarker::AEMC_E_Decoding(int class_indices, std::vector<int> &cluster_indices, CodedCluster &codedCluster) {
    // Generate the cluster details corresponding to the class indices
    AEMC_E E_cluster(CodedLENGTH, class_indices);
    auto base_chords = E_cluster.GetChordMatrix();
    auto codewords = E_cluster.GetCodewords();
    //
    std::vector<std::vector<std::pair<std::vector<int>, std::vector<int>>>> temp_result;
    std::vector<std::pair<std::vector<int>, std::vector<int>>> ordered_indices;   //rawData & codeword 储存所有的聚类和对应的码字位置
    int c_num = static_cast<int> (cluster_indices.size());
    // tri_1 and tri_2
    std::vector<std::vector<int>> tri_list;
    if (c_num > 3) {
        std::vector<std::pair<float, std::vector<int>>> tr_chords;
        for (int i = 0; i < c_num; ++i) {
            for (int j = i + 1; j < c_num; ++j) {
                float ch = (float) (Raw_data_.row(cluster_indices[j]) - Raw_data_.row(cluster_indices[i])).norm();
                tr_chords.emplace_back(ch, std::vector<int>{cluster_indices[i], cluster_indices[j]});
            }
        }
        std::sort(tr_chords.begin(), tr_chords.end(), cmp);
        std::vector<std::vector<int>> traversal_order{{0, 1},
                                                      {0, 2},
                                                      {1, 2},
                                                      {1, 3},
                                                      {2, 3},
                                                      {2, 4},
                                                      {3, 4}};

        for (auto o: traversal_order) {
            if (tri_list.size() < (cluster_indices.size() - 2)) {
                if (tr_chords[o[0]].first > 5.0f && tr_chords[o[1]].first > 5.0f) {
                    auto vertex = vectors_intersection(tr_chords[o[0]].second, tr_chords[o[1]].second);
                    if (vertex.size() == 1) {
                        auto third = vectors_diff(tr_chords[o[1]].second, vertex);
                        std::vector<int> tri = tr_chords[o[0]].second;
                        tri.push_back(third[0]);
                        if (!tri_list.empty()) {
                            for (const auto &l: tri_list) {
                                if (vectors_intersection(tri, l).size() < 3)
                                    tri_list.emplace_back(tri);
                            }
                        } else
                            tri_list.emplace_back(tri);
                    }
                }
            }
        }
        for (const auto &t: tri_list) {
            areInMatrix(t, base_chords, ordered_indices);
            if (!ordered_indices.empty()) {
                temp_result.emplace_back(ordered_indices);
                ordered_indices.clear();
            }
        }
//        std::vector<std::vector<int>> sample;    //Combination
//        combine(static_cast<int> (cluster_indices.size()), 3, sample);
//        for (int i = 0; i < (static_cast<int>(sample.size())); ++i) {
//            std::vector<int> c1;
//            c1.push_back(cluster_indices[sample[i][0]]);
//            c1.push_back(cluster_indices[sample[i][1]]);
//            c1.push_back(cluster_indices[sample[i][2]]);
//            areInMatrix(c1, base_chords, ordered_indices);
//            temp_result.emplace_back(ordered_indices);
//            ordered_indices.clear();
//            c1.clear();
//        }
    } else if (c_num == 3) {
        areInMatrix(cluster_indices, base_chords, ordered_indices);
        if (!ordered_indices.empty()) {
            temp_result.emplace_back(ordered_indices);
            ordered_indices.clear();
        }
    }

    int n = static_cast<int>(temp_result.size());
    int codeword = 0;
//    std::vector<std::pair<int, int>> temp_cluster;
    if (n == 1) {
        int id = bestFit(temp_result[0], base_chords);
        for (int &p: temp_result[0][id].second) {
            codeword += pow(2, p);
        }
        for (auto c: codewords) {
            if (HammingDistance(codeword, c) < 2) {
                codedCluster.Codeword_ = c;
                codedCluster.Coded_flag = true;
            }
        }
        int row_i = 0;
        codedCluster.Points_.resize(temp_result[0][id].second.size(), 3);
        for (auto p: temp_result[0][id].first) {
            codedCluster.Points_.row(row_i) = Raw_data_.row(p);
            row_i++;
        }
        auto codeword_indices =
                std::find(codewords.begin(), codewords.end(), codedCluster.Codeword_) - codewords.begin();
        codedCluster.id = std::to_string(class_indices) + '-' + std::to_string(codeword_indices);
    } else if (n > 1) {
        std::vector<std::pair<std::vector<int>, std::vector<int>>> new_ordered;
        std::vector<std::vector<int>> com;

//        for (int i = 0; i < static_cast<int>(temp_result[0].size()); ++i) {
//            for (int j = 0; j < static_cast<int>(temp_result[1].size()); ++j) {
//                auto r = vectors_intersection(temp_result[0][i].first, temp_result[1][j].first);
//                auto c = vectors_intersection(temp_result[0][i].second, temp_result[1][j].second);
//                if (r.size() == 2 && c.size() == 2) {
//                    auto f_p = vectors_diff(temp_result[1][j].second, c);
//                    auto f_ind = vectors_diff(temp_result[1][j].first, r);
//                    std::vector<int> n_1 = temp_result[0][i].first;
//                    std::vector<int> n_2 = temp_result[0][i].second;
//                    n_1.push_back(f_ind[0]);
//                    n_2.push_back(f_p[0]);
//                    for (int &p: n_2) {
//                        codeword += pow(2, p);
//                    }
//                    if (std::find(codewords.begin(), codewords.end(), codeword) != codewords.end())
//                    new_ordered.emplace_back(n_1, n_2);
//                }
//            }
//        }
        combine(static_cast<int>(temp_result.size()), 2, com);
        for (auto com_: com) {
            int m_0 = com_[0];
            int m_1 = com_[1];
            for (int i = 0; i < static_cast<int>(temp_result[m_0].size()); ++i) {
                for (int j = 0; j < static_cast<int>(temp_result[m_1].size()); ++j) {
                    auto r = vectors_intersection(temp_result[m_0][i].first, temp_result[m_1][j].first);
                    auto c = vectors_intersection(temp_result[m_0][i].second, temp_result[m_1][j].second);
                    if (r.size() == 2 && c.size() == 2) {
                        auto f_p = vectors_diff(temp_result[m_1][j].second, c);
                        auto f_ind = vectors_diff(temp_result[m_1][j].first, r);
                        std::vector<int> n_1 = temp_result[m_0][i].first;
                        std::vector<int> n_2 = temp_result[m_0][i].second;
                        n_1.push_back(f_ind[0]);
                        n_2.push_back(f_p[0]);
                        new_ordered.emplace_back(n_1, n_2);
                    }
                }
            }
        }
        temp_result.clear();
        std::vector<int> codewordsList;
        std::vector<int> del_list;
        for (int i = 0; i < static_cast<int>(new_ordered.size()); ++i) {
            codeword = 0;
            for (int &p: new_ordered[i].second) {
                codeword += pow(2, CodedLENGTH - 1 - p);
            }
            int decode_flag = 0;

            for (auto c: codewords) {
                if (HammingDistance(codeword, c) < 2) {
//                if (codeword == c) {
                    codewordsList.push_back(c);
                    decode_flag = 1;
                    break;
                }
            }
            if (decode_flag == 0) {
                del_list.push_back(i);
//                new_ordered.erase(new_ordered.begin() + i);
            }
        }
        std::sort(del_list.begin(), del_list.end());
        for (int i = 0; i < static_cast<int>(del_list.size()); ++i) {
            new_ordered.erase(new_ordered.begin() + del_list[i] - i);
        }
        if (!new_ordered.empty()) {
            std::vector<int> cluster_points;
            if (new_ordered.size() == 1) {
                codedCluster.Codeword_ = codewordsList[0];
                cluster_points = new_ordered[0].first;
            } else {
                int result = bestFit(new_ordered, base_chords);
                codedCluster.Codeword_ = codewordsList[result];
                cluster_points = new_ordered[result].first;
            }
            codedCluster.Points_.resize(cluster_points.size(), 3);
            int row_i = 0;
            for (auto p: cluster_points) {
                codedCluster.Points_.row(row_i) = Raw_data_.row(p);
                row_i++;
            }
            codedCluster.Coded_flag = true;
        }
        new_ordered.clear();
    }
    if (frame_ > 1207) {
        std::cout << std::endl;
    }
    if (codedCluster.Coded_flag) {
        auto codeword_indices =
                std::find(codewords.begin(), codewords.end(), codedCluster.Codeword_) - codewords.begin();
        codedCluster.id = std::to_string(class_indices) + '-' + std::to_string(codeword_indices);
        codedCluster.Position = FitCircle(codedCluster.Points_).center;
        GenTransform(codedCluster.Points_, codedCluster.Pose, codedCluster.Position);
//            GenPose(codedCluster.Points_, codedCluster.Pose);
        std::cout << "E:" << codedCluster.id << "/";
    } else {
//        std::cout << "failed" << "/";
        int row_i = 0;
        codedCluster.Points_.resize(cluster_indices.size(), 3);
        for (auto p: cluster_indices) {
            codedCluster.Points_.row(row_i) = Raw_data_.row(p);
            row_i++;
        }
        codedCluster.Position = FitCircle(codedCluster.Points_).center;
        codedCluster.id = "fail";
    }
}

int RawMarker::bestFit(std::vector<std::pair<std::vector<int>, std::vector<int>>> &ordered_indices,
                       const Eigen::MatrixXf &chordsMatrix) {
    std::vector<std::pair<double, int>> errList;
    int ind = 0;
    for (const auto &ord_i: ordered_indices) {
        double err = 0;
        for (int i = 0; i < static_cast<int>(ord_i.first.size()); ++i) {
            for (int j = i; j < static_cast<int>(ord_i.first.size()); ++j) {
                double measured = (Raw_data_.row(ord_i.first[i]) - Raw_data_.row(ord_i.first[0])).norm();
                double truth = chordsMatrix(ord_i.second[i], ord_i.second[j]);
                err += fabs(measured - truth);
            }
        }
        errList.emplace_back(err, ind);
        ind++;
    }
    std::sort(errList.begin(), errList.end(), cmp1);
    return errList[0].second;
}

void RawMarker::areInMatrix(const std::vector<int> &subCluster_indices, const Eigen::MatrixXf &chordsMatrix,
                            std::vector<std::pair<std::vector<int>, std::vector<int>>> &ordered_indices) {
    std::vector<std::pair<float, std::vector<int>>> chords;
    chords.emplace_back(static_cast<float> ((Raw_data_.row(subCluster_indices[1]) -
                                             Raw_data_.row(subCluster_indices[0])).norm()), std::vector<int>{0, 1});
    chords.emplace_back(static_cast<float> ((Raw_data_.row(subCluster_indices[2]) -
                                             Raw_data_.row(subCluster_indices[0])).norm()), std::vector<int>{0, 2});
    chords.emplace_back(static_cast<float> ((Raw_data_.row(subCluster_indices[2]) -
                                             Raw_data_.row(subCluster_indices[1])).norm()), std::vector<int>{1, 2});
    std::sort(chords.begin(), chords.end(), cmp);
    auto e_1 = (chordsMatrix.array() - chords[0].first).abs();
    auto e_2 = (chordsMatrix.array() - chords[1].first).abs();
    std::vector<std::vector<int>> position_1;
    std::vector<std::vector<int>> position_2;
    for (int i = 0; i < chordsMatrix.rows(); ++i) {
        for (int j = i + 1; j < chordsMatrix.cols(); ++j) {
            if (e_1(i, j) < ERR)
                position_1.push_back(std::vector<int>{i, j});
            if (e_2(i, j) < ERR)
                position_2.push_back(std::vector<int>{i, j});
        }
    }
    for (const auto &pos1: position_1) {
        for (const auto &pos2: position_2) {
            std::vector<int> r_indices;   //Raw_data index
            std::vector<int> code_position;   // Codeword index
            auto t_inter = vectors_intersection(pos1, pos2);
            if (t_inter.size() == 1) {
                auto pos3_x = vectors_diff(pos1, t_inter);
                auto pos3_y = vectors_diff(pos2, t_inter);
                float base_chord3;
                if (pos3_x[0] < pos3_y[0])
                    base_chord3 = chordsMatrix(pos3_x[0], pos3_y[0]);
                else
                    base_chord3 = chordsMatrix(pos3_y[0], pos3_x[0]);
                if (std::fabs(chords[2].first - base_chord3) < ERR) {
                    auto r_inter = vectors_intersection(chords[0].second, chords[1].second);
                    int ind_1 = r_inter[0];
                    int ind_2 = vectors_diff(chords[0].second, r_inter)[0];
                    int ind_3 = vectors_diff(chords[1].second, r_inter)[0];
                    r_indices = {subCluster_indices[ind_1], subCluster_indices[ind_2], subCluster_indices[ind_3]};
                    code_position = {t_inter[0], pos3_x[0], pos3_y[0]};
                    ordered_indices.emplace_back(r_indices, code_position);
                }
            }
            r_indices.clear();
            code_position.clear();
        }
    }
}

void RawMarker::Fitting(const std::vector<std::vector<int>> &PotentialCluster_indices,
                        std::vector<CodedCluster> &CodedClusterList) {
    // Processing of each potential cluster
    for (const auto &PotentialCluster: PotentialCluster_indices) {
        int n = static_cast<int>(PotentialCluster.size());
        std::vector<bool> processed(n, false);   //Storing processed status
        std::vector<std::vector<int>> sample;    //Combination set
        combine(n, 3, sample);
        Eigen::Matrix<double, 3, 3> points3;
        CodedCluster codedCluster1;
        //  Circle filter:
        for (const auto &s: sample) {
            if (processed[s[0]] || processed[s[1]] || processed[s[2]])
                continue;
            else {
                points3.row(0) = Raw_data_.row(PotentialCluster[s[0]]);
                points3.row(1) = Raw_data_.row(PotentialCluster[s[1]]);
                points3.row(2) = Raw_data_.row(PotentialCluster[s[2]]);
                Circle cir = Points3Fitting(cir, points3);
                int class_indices = 255;
                std::vector<int> temp_cluster;
                //过滤
                if (areRanges(cir.radius, classArr, ERR / 2, class_indices)) {
                    temp_cluster.push_back(PotentialCluster[s[0]]);
                    temp_cluster.push_back(PotentialCluster[s[1]]);
                    temp_cluster.push_back(PotentialCluster[s[2]]);
                    processed[s[0]] = true;
                    processed[s[1]] = true;
                    processed[s[2]] = true;
                    //计算其他内点
                    for (int i = 0; i < n; ++i) {
                        if (processed[i])
                            continue;
                        else {
                            auto dis = (Raw_data_.row(PotentialCluster[i]).transpose() - cir.center).norm();
                            if (std::fabs(dis - cir.radius) < ERR) {
                                temp_cluster.push_back(PotentialCluster[i]);
                                processed[i] = true;
                            }
                        }
                    }

                    // AEMC_I decoding
                    if (class_indices <= 2) {
                        AEMC_I_Decoding(class_indices, temp_cluster, cir, codedCluster1);

                    }
                        // AEMC_E decoding
                    else if (class_indices > 2 && class_indices < static_cast<int>(classArr.size())) {
                        AEMC_E_Decoding(class_indices, temp_cluster, codedCluster1);
                    }
                    //若Decode_flag为true，标记为已处理
                    if (!codedCluster1.Coded_flag) {
                        for (auto p: processed) {
                            p = false;
                        }
                    }
                    temp_cluster.clear();
                }
            }
        }
        CodedClusterList.push_back(codedCluster1);
        processed.clear();
    }
}

// Outlier 与已知的序列做对比
void RawMarker::Track(std::vector<PreClusters> &preClusterList, std::vector<CodedCluster> &curCodedClusterList) const {
    for (auto &c: curCodedClusterList) {
        bool tracked = false;
        if (preClusterList.empty()) {
            PreClusters preClusters;
            preClusters.id = c.id;
            preClusters.frames.push_back(frame_);
            preClusters.Positions.push_back(c.Position);
            preClusterList.push_back(preClusters);
        } else {
            for (auto &p: preClusterList) {
                if (p.frames.size() < 10) {
                    if (p.id == c.id) {
                        tracked = true;
                        p.frames.push_back(frame_);
                        p.Positions.push_back(c.Position);
//                        p.frames.erase(p.frames.begin());
//                        p.Positions.erase(p.Positions.begin());
                        std::cout << "add>>" << c.id << "  ";
                    }
                } else {
                    if (c.id == p.id) {
//                        if (filter(p.Positions, c.Position)) {
                        if (filter2(p, c.Position, frame_)) {
                            tracked = true;
                            p.frames.push_back(frame_);
                            p.Positions.push_back(c.Position);
                            std::cout << "true>>" << c.id << "  ";
                            break;
                        }
                    } else
                        continue;
                }
            }
        }
        if (!tracked) {
            bool new_cluster = true;
            for (auto &p: preClusterList) {
//                if (filter(p.Positions, c.Position)) {
                if (filter2(p, c.Position, frame_)) {
                    new_cluster = false;
                    c.id = p.id;
                    p.frames.push_back(frame_);
                    p.Positions.push_back(c.Position);
                    std::cout << "mod>>" << c.id << "  ";
                    break;
                }
            }
            if (new_cluster && c.Coded_flag) {
                PreClusters preClusters;
                preClusters.id = c.id;
                preClusters.frames.push_back(frame_);
                preClusters.Positions.push_back(c.Position);
                preClusterList.push_back(preClusters);
                std::cout << "new>>" << c.id << "  ";
            }
        }
    }
    for (auto p: preClusterList) {
        if (p.Positions.size() > 9) {
            p.frames.erase(p.frames.begin());
            p.Positions.erase(p.Positions.begin());
        }
        if (p.Positions.size() > 1) {
            int n = p.frames.size();
            if ((p.frames.at(n - 1) - p.frames.at(n - 2)) > 10) {
                for (int i = 0; i < n - 1; ++i) {
                    p.frames.erase(p.frames.begin());
                    p.Positions.erase(p.Positions.begin()
                    );
                }
            }
        }
        std::cout << "ids:" << p.id << "--" << p.Positions.size() << "   ";
    }
}

// store 10 frames data
//    if (!curCodedClusterList.empty()) {
//        if (preClusterList.frames.size() < 20) {
//            for (auto c: curCodedClusterList) {
//                if (c.Coded_flag) {
//                    if (preClusterList.ids.empty()) {
//                        preClusterList.ids.push_back(c.id);
//                        preClusterList.Positions.push_back({c.Position});
//                    } else {
//                        if (std::find(preClusterList.ids.begin(), preClusterList.ids.end(), c.id) ==
//                            preClusterList.ids.end()) {
//                            preClusterList.ids.push_back(c.id);
//                            preClusterList.Positions.push_back({c.Position});
//                        } else {
//                            int pre_id = std::find(preClusterList.ids.begin(), preClusterList.ids.end(), c.id) -
//                                         preClusterList.ids.begin();
//                            preClusterList.Positions[pre_id].push_back(c.Position);
//                        }
//                    }
//                }
//            }
//            std::cout << "\n p" << preClusterList.frames.size();
//            preClusterList.frames.push_back(frame_);
//            std::cout << "--" << std::endl;
//        } else {
//            // pop data
//            preClusterList.frames.erase(preClusterList.frames.begin());
//            preClusterList.frames.push_back(frame_);
//            for (auto &c: curCodedClusterList) {
//                if (std::find(preClusterList.ids.begin(), preClusterList.ids.end(), c.id) !=
//                    preClusterList.ids.end()) {
//                    int pre_id = std::find(preClusterList.ids.begin(), preClusterList.ids.end(), c.id) -
//                                 preClusterList.ids.begin();
//                    // 滤波位置
//                    if (filter(preClusterList.Positions[pre_id], c.Position)) {
//                        preClusterList.Positions[pre_id].push_back(c.Position);
//                    } else {
//                        //判断距离Position最近的点的id并修改cur,并添加到pre list
//                        for (int i = 0; i < static_cast<int>(preClusterList.Positions.size()); ++i) {
//                            if (filter(preClusterList.Positions[i], c.Position)) {
//                                c.id = preClusterList.ids[i];
//                                std::cout << "track";
//                                preClusterList.Positions[i].push_back(c.Position);
//                                break;
//                            }
//                        }
//                    }
//                } else {
//                    if (c.id == "f") {
//                        //判断距离Position最近的点的id并修改cur,并添加到pre list
//                        for (int i = 0; i < static_cast<int>(preClusterList.Positions.size()); ++i) {
//                            if (filter(preClusterList.Positions[i], c.Position)) {
//                                c.id = preClusterList.ids[i];
//                                preClusterList.Positions[i].push_back(c.Position);
//                            }
//                        }
//                    } else {
//                        preClusterList.ids.push_back(c.id);
//                        preClusterList.Positions.push_back({c.Position});
//                    }
//                }
//            }
//
//            for (auto &c: curCodedClusterList) {
//                bool tracked = false;
//                for (int i = 0; i < static_cast<int>(preClusterList.Positions.size()); ++i) {
//                    if (!tracked) {
//                        if (filter(preClusterList.Positions[i], c.Position)) {
//                            if (c.id == preClusterList.ids[i]) {
//                                preClusterList.Positions[i].push_back(c.Position);
//                                tracked = true;
//                                std::cout << "true>>" << c.id << ";";
//                            } else {
//                                if (preClusterList.Positions[i].size() > 3) {
//                                    c.id = preClusterList.ids[i];
//                                    tracked = true;
//                                    std::cout << "mod-tracker>>" << c.id << ";";
//                                }
//                            }
//                        }
//                    }
//                }
//                if (!tracked) {
//                    if (std::find(preClusterList.ids.begin(), preClusterList.ids.end(), c.id) ==
//                        preClusterList.ids.end()) {
//                        preClusterList.ids.push_back(c.id);
//                        preClusterList.Positions.push_back({c.Position});
//                        std::cout << "new>>" << c.id << ";";
//                    } else {}
//                }
//            }
//            for (auto &p: preClusterList.Positions) {
//                if (!p.empty()) {
//                    p.erase(p.begin());
//                    std::cout << "test" << p.size() << std::endl;
//                }
//            }
//        }
//    }

void RawMarker::TraceTrack(std::vector<PreTrace> &preTrace, std::vector<CodedCluster> &curCodedClusterList) {
    int t_num = (int) preTrace.size();
    int c_num = (int) curCodedClusterList.size();
    if (c_num > 0) {
        // initial
        if (t_num == 0) {
            for (auto &c: curCodedClusterList) {
                if (c.Coded_flag) {
                    PreTrace t;
                    t.ids.push_back(c.id);
                    t.frames.push_back(frame_);
                    t.Positions.push_back(c.Position);
                    preTrace.emplace_back(t);
                }
            }
        } else {
            std::vector<bool> t_process(t_num, false);
            std::vector<double> dis_list(preTrace.size(), 0.0);
            std::vector<Eigen::Vector3d> vel;
            std::vector<Eigen::Vector3d> accel;
            std::pair<std::vector<int>, std::vector<double>> first_track;
            for (auto &c: curCodedClusterList) {
//                std::pair<int, Eigen::Vector3d> disAndId{0, {500.0, 500.0, 500.0}};
                std::pair<int, double> disAndId{t_num + 1, 0.0};
                for (int i = 0; i < t_num; ++i) {
                    //使用frame判断是否进行距离判断

                    if ((frame_ - preTrace[i].frames.back()) < 2) {
                        double f_dis = (c.Position - preTrace[i].Positions.back()).norm();
                        if (f_dis > 100)
                            continue;
                    }
                    int n_range = 0;
//                    double dis = 0.0;
                    Eigen::Vector3d dis{0.0, 0.0, 0.0};
                    double dis_p = 0.0;
                    for (int j = 0; j < static_cast<int> (preTrace[i].frames.size()); ++j) {
                        if ((frame_ - preTrace[i].frames[j]) < 5) {
//                            dis += (c.Position.array() - preTrace[i].Positions[j].array()).abs().matrix();
//                                dis += (c.Position - preTrace[i].Positions[j]).cwiseAbs();  // 距离差
                            dis_p += (c.Position - preTrace[i].Positions[j]).norm();
                            n_range += 1;
                        }
                    }
                    // 最短距离及其索引
                    Eigen::Vector3d avg_dis = dis / n_range;
                    double avg_dis_p = dis_p / n_range;
//                        if (avg_dis[0] < disAndId.second[0] && avg_dis[1] < disAndId.second[1] &&
//                            avg_dis[2] < disAndId.second[2]) {
//                            disAndId.first = i;
//                            disAndId.second = avg_dis;
                    if (disAndId.first > t_num) {
                        disAndId.first = i;
                        disAndId.second = avg_dis_p;
                    } else {
                        if (avg_dis_p < disAndId.second) {
                            disAndId.first = i;
                            disAndId.second = avg_dis_p;
                        }
                    }
                }
                first_track.first.push_back(disAndId.first);
                first_track.second.push_back(disAndId.second);
            }
            std::vector<int> unmatched;
            bool single_flag = true;
            for (int i = 0; i < c_num; ++i) {
                for (int j = i + 1; j < c_num; ++j) {
                    if (first_track.first[i] == first_track.first[j]) {
                        single_flag = false;
                        if (std::find(unmatched.begin(), unmatched.end(), i) == unmatched.end())
                            unmatched.push_back(i);
                        if (std::find(unmatched.begin(), unmatched.end(), j) == unmatched.end())
                            unmatched.push_back(j);
                    }
                }
            }
            if (single_flag) {
                for (int i = 0; i < static_cast<int> (first_track.first.size()); ++i) {
                    if (first_track.first[i] > t_num) {
                        PreTrace t;
                        t.ids.push_back(curCodedClusterList[i].id);
                        t.frames.push_back(frame_);
                        t.Positions.push_back(curCodedClusterList[i].Position);
                        preTrace.emplace_back(t);
                        std::cout << "new--" << curCodedClusterList[i].id << "  ";
                    } else {
                        if (preTrace[first_track.first[i]].frames.size() > 7) {

                            std::string pre_id;
                            std::map<std::string, int> m;
                            for (auto id: preTrace[first_track.first[i]].ids) {
                                if (id != "fail")
                                    m[id]++;
                            }
                            int max_count = 0;
                            for (auto item: m) {
                                if (item.second > max_count) {
                                    max_count = item.second;
                                    pre_id = item.first;
                                }
                            }
                            if (curCodedClusterList[i].id == pre_id) {
                                std::cout << "true--" << curCodedClusterList[i].id << "  ";
                            } else {
                                curCodedClusterList[i].id = pre_id;
                                std::cout << "mod--" << curCodedClusterList[i].id << "  ";
                            }
                        } else {
                            std::cout << "add--" << curCodedClusterList[i].id << "  ";
                        }
                        curCodedClusterList[i].Coded_flag = true;
                        preTrace[first_track.first[i]].frames.push_back(frame_);
                        preTrace[first_track.first[i]].ids.push_back(curCodedClusterList[i].id);
                        preTrace[first_track.first[i]].Positions.emplace_back(curCodedClusterList[i].Position);
                    }
                }
            } else {
                double err = 200.0;
                for (auto m: unmatched) {

                }
            }
        }


//                std::cout << disAndId.second[0] << ",  " << disAndId.second[1] << ",  " << disAndId.second[2] << ",  ";
//            if (((preTrace[disAndId.first].Positions.back() - c.Position).norm() /
//                 (frame_ - preTrace[disAndId.first].frames.back())) < 40.0) {
//                if (preTrace[disAndId.first].frames.size() > 7) {
//                    std::string pre_id;
//                    std::map<std::string, int> m;
//                    for (auto id: preTrace[disAndId.first].ids) {
//                        if (id != "fail")
//                            m[id]++;
//                    }
//                    int max_count = 0;
//                    for (auto item: m) {
//                        if (item.second > max_count) {
//                            max_count = item.second;
//                            pre_id = item.first;
//                        }
//                    }
//                    if (c.id == pre_id) {
//                        std::cout << "true--" << c.id << "  ";
//                    } else {
//                        c.id = pre_id;
//                        std::cout << "mod--" << c.id << "  ";
//                    }
//                } else {
//                    std::cout << "add--" << c.id << "  ";
//                }
//                t_process[disAndId.first] = true;
//                c.Coded_flag = true;
//                preTrace[disAndId.first].frames.push_back(frame_);
//                preTrace[disAndId.first].ids.push_back(c.id);
//                preTrace[disAndId.first].Positions.emplace_back(c.Position);
//            } else {
//                bool new_flag = true;
//                for (auto p: preTrace) {
//                    std::string pre_id;
//                    std::map<std::string, int> m;
//                    for (auto id: p.ids) {
//                        if (id != "fail")
//                            m[id]++;
//                    }
//                    int max_count = 0;
//                    for (auto item: m) {
//                        if (item.second > max_count) {
//                            max_count = item.second;
//                            pre_id = item.first;
//                        }
//                    }
//                    if (c.id == pre_id)
//                        new_flag = false;
//                }
//                if (new_flag) {
//                    PreTrace t;
//                    t.ids.push_back(c.id);
//                    t.frames.push_back(frame_);
//                    t.Positions.push_back(c.Position);
//                    preTrace.emplace_back(t);
//                    std::cout << "new--" << c.id << "  ";
//                } else {
//                    c.Coded_flag = false;
//                }
//            }


    }

// Limit the number of trace points
    for (auto p = preTrace.begin(); p != preTrace.end();) {
        if ((frame_ - p->frames.back() > 10)) {
            p = preTrace.erase(p);
        } else {
            if (p->frames.size() > 10) {
                p->frames.erase(p->frames.begin());
                p->ids.erase(p->ids.begin());
                p->Positions.erase(p->Positions.begin());
            }
            std::cout << " num " << p->ids.size() << "is" << p->ids.back() << "   ";
            ++p;
        }
    }
    std::cout << " trace " << preTrace.size();
}

bool filter(std::vector<Eigen::Vector3d> &pre, Eigen::Vector3d &cur) {
    bool flag = false;
    int n = (int) pre.size();
    // 平均欧氏距离滤波
//    if (n > 1) {
//        double sum_dis;
//        for (int i = 0; i < (n - 1); ++i) {
//            sum_dis += (pre[i + 1] - pre[i]).norm();
//        }
//        sum_dis /= n;
//        if ((cur - pre[-1]).norm() < sum_dis)
//            flag = true;
//    }
    if (n > 1) {
        Eigen::Vector3d sum_delta{0.0, 0.0, 0.0};
        for (int i = 0; i < n - 1; ++i) {
            sum_delta += (pre[i + 1] - pre[i]).cwiseAbs();
        }
//        sum_delta /= n;
        sum_delta = (pre[n - 1] - pre[n - 2]).cwiseAbs();
        Eigen::Vector3d cur_delta = (cur - pre[n - 1]).cwiseAbs();
//        auto d = (cur_delta - sum_delta).cwiseAbs();
        if (cur_delta[0] < 2 * sum_delta[0] && cur_delta[1] < 2 * sum_delta[1] && cur_delta[2] < 2 * sum_delta[2])
//        if (cur_delta[0] <  && d[1] < 0.5 && d[2] < 0.5)
            flag = true;
    }
    return flag;
}

bool filter2(PreClusters &pre, Eigen::Vector3d &cur, int64_t frame) {
    bool flag = false;
    int n = (int) pre.frames.size();
    if (n > 1) {
        Eigen::Vector3d sum_delta{0.0, 0.0, 0.0};
//        sum_delta /= n;
        sum_delta =
                (pre.Positions[n - 1] - pre.Positions[n - 2]).cwiseAbs() / (pre.frames[n - 1] - pre.frames[n - 2]);
        Eigen::Vector3d cur_delta = (cur - pre.Positions[n - 1]).cwiseAbs() / (frame - pre.frames[n - 1]);
//        auto d = (cur_delta - sum_delta).cwiseAbs();
        if (cur_delta[0] < 3 * sum_delta[0] && cur_delta[1] < 3 * sum_delta[1] && cur_delta[2] < 3 * sum_delta[2])
//        if (cur_delta[0] <  && d[1] < 0.5 && d[2] < 0.5)
            flag = true;
    }
    return flag;
}


void GenPose(Eigen::MatrixX3d &pts, Eigen::Quaternion<double> &Pose) {
    Eigen::MatrixXd L1 = Eigen::MatrixXd::Ones(pts.rows(), 1);
    Eigen::Vector3d A = (pts.transpose() * pts).inverse() * pts.transpose() * L1;
    A.normalize();
    if (A[2] < 0)
        A = -A;
    Eigen::Vector3d Z{0, 0, 1};
    if ((Z + A).norm() == 0)
        Pose = {0, 1, 0, 0};
    else {
        Eigen::Vector3d half = (Z + A) / (Z + A).norm();
        Eigen::Vector3d cross = Z.cross(half);
        Pose = {Z.dot(half.transpose()), cross[0], cross[1], cross[2]};
    }
}

void GenTransform(Eigen::MatrixX3d &pts, Eigen::Quaternion<double> &Pose, Eigen::Vector3d &position) {
    Eigen::MatrixXd L1 = Eigen::MatrixXd::Ones(pts.rows(), 1);
    Eigen::Vector3d w = (pts.transpose() * pts).inverse() * pts.transpose() * L1;
    w.normalize();
    if (w[2] < 0)
        w = -w;
    Eigen::Vector3d u = (pts.row(0).transpose() - position).normalized();
    Eigen::Vector3d v = w.cross(u);
    Eigen::Matrix<double, 3, 3> transition;
    Eigen::Vector3d x{1, 0, 0};
    Eigen::Vector3d y{0, 1, 0};
    Eigen::Vector3d z{0, 0, 1};
    transition << u.dot(x), v.dot(x), w.dot(x),
            u.dot(y), v.dot(y), w.dot(y),
            u.dot(z), v.dot(z), w.dot(z);
//    transition = transition.inverse();
    Pose = Eigen::Quaterniond(transition);
    Pose.normalize();
}

// Calculating the center of a circle with three points
const Circle &Points3Fitting(Circle &cir1, Eigen::Matrix<double, 3, 3> &points) {
//    std::cout << points.rows();
    Eigen::Vector3d A = points.row(0).transpose();
    Eigen::Vector3d B = points.row(1).transpose();
    Eigen::Vector3d C = points.row(2).transpose();
    double a = (C - B).norm();
    double b = (C - A).norm();
    double c = (B - A).norm();
    double s = (a + b + c) / 2;
    cir1.radius = static_cast<float>(a * b * c / 4 / sqrt(s * (s - a) * (s - b) * (s - c)));
    double b1 = a * a * (b * b + c * c - a * a);
    double b2 = b * b * (a * a + c * c - b * b);
    double b3 = c * c * (a * a + b * b - c * c);
    Eigen::Vector3d barcyntric;
    barcyntric << b1, b2, b3;
    cir1.center = points.transpose() * barcyntric / barcyntric.sum();
    return cir1;
}

// Optimal circle fitting by the method of least-squares
//const Circle &LeastSquareFitting(Eigen::MatrixX3d &points) {
//    Circle *cir1;
//
//    return *cir1;
//}

//void sort2Vector(std::vector<int> &a, std::vector<int> &b) {
//    assert(a.size() == b.size());
//    std::vector<std::pair<int, int>> a_b;
//    for (int i = 0; i < static_cast<int>(a.size()); ++i) {
//        a_b.emplace_back(a[i], b[i]);
//    }
//
//}

void combine(int N, int M, std::vector<std::vector<int>> &com) {
    std::vector<bool> v(N);
    std::vector<int> temp;
    for (int i = 0; i < N; ++i) {
        v[i] = (i >= (N - M));
    }
    do {
        for (int i = 0; i < N; ++i) {
            if (v[i]) {
                temp.push_back(i);
            }
        }
        com.push_back(temp);
        temp.clear();
    } while (std::next_permutation(v.begin(), v.end()));
}

bool areRanges(float value, const std::vector<float> &targets, float error, int &class_idx) {
    bool flag = false;
    std::vector<float> err;
    for (int i = 0; i < static_cast<int>(targets.size()); ++i) {
        float e = std::fabs(value - targets[i]);
        if (e < error) {
            flag = true;
            if (err.empty()) {
                err.push_back(e);
                class_idx = i;
            } else {
                if (e < err[0]) {
                    err[0] = e;
                    class_idx = i;
                }
            }

        }
    }
    return flag;
}

double getCrossAngle(Eigen::Vector3d &vertex, Eigen::Vector3d &P1, Eigen::Vector3d &P2) {
    Eigen::Vector3d v1 = P1 - vertex;
    Eigen::Vector3d v2 = P2 - vertex;
    double cosVal = v1.dot(v2) / (v1.norm() * v2.norm());
    double angle = acos(cosVal);
    return angle;
}

bool cmp(const std::pair<float, std::vector<int>> &a, const std::pair<float, std::vector<int>> &b) {
    return a.first < b.first;
}

bool cmp1(const std::pair<double, int> &a, const std::pair<double, int> &b) {
    return a.first < b.first;
}


std::vector<int> vectors_intersection(std::vector<int> v1, std::vector<int> v2) {
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    std::vector<int> v;
    set_intersection(v1.begin(), v1.end(), v2.begin(), v2.end(), back_inserter(v));
    return v;
}

//v1-v2
std::vector<int> vectors_diff(std::vector<int> v1, std::vector<int> v2) {
    std::sort(v1.begin(), v1.end());
    std::sort(v2.begin(), v2.end());
    std::vector<int> v;
    std::set_difference(v1.begin(), v1.end(), v2.begin(), v2.end(), std::inserter(v, v.begin()));
    return v;
}

int cyclicShift(int value, int d, int N) {
    return ((value << d) % (1 << N)) | (value >> (N - d));
//    return (value << d) | (value >> (N - d));
}

int HammingDistance(int x, int y) {
    int temp = x ^ y;
    int count = 0;
    while (temp != 0) {
        ++count;
        temp &= temp - 1;
    }
    return count;
}
// TODO 最小二乘法拟合圆心
//void fitPlaneSVD(Eigen::MatrixX3d &points) {
//    Eigen::RowVector3d centroid = points.rowwise().mean();
//    Eigen::MatrixXd P_mean = points;
//    P_mean.rowwise() -= centroid;
//    Eigen::JacobiSVD<Eigen::MatrixXd> svd(P_mean, Eigen::ComputeFullU | Eigen::ComputeFullV);
//    Eigen::Matrix3d V = svd.matrixV();
//    Eigen::MatrixXd U = svd.matrixU();
//    Eigen::Matrix3d S = U.inverse() * P_mean * V.transpose().inverse();
//    // 5、平面的法向量a,b,c
//    Eigen::RowVector3d normal;
//    normal << V(0, 2), V(1, 2), V(2, 2);
//    // 6、原点到平面的距离d
//    double d = -normal * centroid.transpose();
//    // 7、获取拟合平面的参数a,b,c,d和质心x,y,z。
//    m_planeparameters << normal, d, centroid;
//}

Circle FitCircle(Eigen::MatrixX3d &pts) {
    Circle c1;
    auto num = pts.rows();
    Eigen::MatrixXd L1 = Eigen::MatrixXd::Ones(num, 1);
    Eigen::Vector3d A = (pts.transpose() * pts).inverse() * pts.transpose() * L1;

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(num - 1, 3);
    for (int i = 0; i < num - 1; i++) {
        B.row(i) = pts.row(i + 1) - pts.row(i);
    }
    Eigen::MatrixXd L2 = Eigen::MatrixXd::Zero(num - 1, 1);
    for (int i = 0; i < num - 1; i++) {
        L2(i) = (pts(i + 1, 0) * pts(i + 1, 0) + pts(i + 1, 1) * pts(i + 1, 1) + pts(i + 1, 2) * pts(i + 1, 2)
                 - (pts(i, 0) * pts(i, 0) + pts(i, 1) * pts(i, 1) + pts(i, 2) * pts(i, 2))) / 2.0;
    }
    Eigen::Matrix4d D;
    D.setZero();
    D.block<3, 3>(0, 0) = B.transpose() * B;
    D.block<3, 1>(0, 3) = A;
    D.block<1, 3>(3, 0) = A.transpose();
    Eigen::Vector4d L3((B.transpose() * L2)(0), (B.transpose() * L2)(1), (B.transpose() * L2)(2), 1);
    Eigen::Vector4d C = D.inverse() * L3;
    float radius = 0;
    for (int i = 0; i < num; i++) {
        Eigen::Vector3f tmp(pts.row(i)(0) - C(0), pts.row(i)(1) - C(1), pts.row(i)(2) - C(2));
        radius = radius + sqrt(tmp(0) * tmp(0) + tmp(1) * tmp(1) + tmp(2) * tmp(2));
    }

    c1.radius = static_cast<float> (radius / num);
    c1.center = C.segment(0, 3);
    return c1;
}
