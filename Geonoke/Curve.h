#pragma once
// Eigen
#include "Vector.h"
// std
#include <vector>

class Curve2d {
public:
  // methods
  Curve2d(const std::vector<EVector3d> points);
  ~Curve2d();
  double LinkScore(const Curve2d &ano_curve, bool is_begin = false, bool ano_is_begin = true);

  // data
  std::vector<EVector3d> points_;
};

class Curve3d {
public:
  Curve3d(Eigen::Vector3d pt_begin, Eigen::Vector3d pt_end, int num_points);
  
  // data
  std::vector<Eigen::Vector3d> points_;
  int num_points_;
};