//
// Created by aska on 19-2-1.
//

#pragma once
#include <Eigen>

#include <string>
#include <vector>
#include <fstream>

namespace Utils {
void SavePointsAsPly(std::string save_path, const std::vector<Eigen::Vector3d> &points);
}