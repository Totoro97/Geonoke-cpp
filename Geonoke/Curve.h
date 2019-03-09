#pragma once
// Eigen
#include "Vector.h"
// std
#include <vector>

namespace Noke {

class Curve2d {
public:
  // methods
  Curve2d(const std::vector<EVector2d> &points);
  Curve2d(const Curve2d &ano_curve_2d);
  ~Curve2d();

  double LinkScore(const Curve2d &ano_curve, bool is_begin = false, bool ano_is_begin = true);
  void Link(const Curve2d &ano_curve);
  void Reverse();
  // data
  std::vector<EVector2d> points_;
};

class Curve3d {
public:
  Curve3d(Eigen::Vector3d pt_begin, Eigen::Vector3d pt_end, int num_points);
  
  // data
  std::vector<Eigen::Vector3d> points_;
  int num_points_;
};

}