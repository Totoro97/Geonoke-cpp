//
// Created by aska on 2019/3/10.
//

#include "Curve.h"
#include <Eigen>
#include <iostream>
#include <vector>

void Check(bool x, std::string err_info = "") {
  if (!x) {
    std::cout << "Fuck: " << err_info << std::endl;
    exit(0);
  }
}

void TestCurve2d() {
  using namespace Noke;
  std::vector<Eigen::Vector2d> points;
  for (int i = 0; i < 11; i++)
    points.emplace_back(i, 0.0);

  // Test resample.
  Curve2d curve_0(points);
  curve_0.Resample(0.5);
  Check(curve_0.points_.size() == 21, "size error");
  for (int i = 0; i < 21; i++) {
    Check(curve_0.points_[i](0) == (double) i * 0.5, "step error");
  }

  Curve2d curve_1(points);
  curve_0.Resample(2.0);
  Check(curve_0.points_.size() == 6, "size error");
  for (int i = 0; i < 6; i++) {
    Check(curve_0.points_[i](0) == (double) i * 2.0, "step error");
  }

  // Test Mean Conv.
  points = {
    Eigen::Vector2d(0.0, 0.0),
    Eigen::Vector2d(1.0, 0.0),
    Eigen::Vector2d(2.0, 0.0),
  };
  Curve2d curve_2(points);
  curve_2.MeanConv(1);
  Check(curve_2.points_[0](0) == 0.5, "Conv Error");
  Check(curve_2.points_[1](0) == 1.0, "Conv Error");
  Check(curve_2.points_[2](0) == 1.5, "Conv Error");
}

int main() {
  TestCurve2d();
  return 0;
}