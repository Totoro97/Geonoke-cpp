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

  void UpdateInfoFromPoints();
  double E2EDistance(const Curve2d &ano_curve, bool is_begin, bool ano_is_begin);
  double LinkScore(const Curve2d &ano_curve, bool is_begin = false, bool ano_is_begin = true);
  void Link(const Curve2d &ano_curve);
  void Reverse();
  void Resample(double hope_dis);
  void MeanConv(int n);

  Eigen::Vector2d At(double s);
  double Curvature(double s);
  double CurvatureIndex(int idx);
  double Length();
  // Calc Pyramid Arclength Descriptor.
  Eigen::VectorXd CalcPAD(double s, double r, int n);
  
  // ---- data ----
  std::vector<EVector2d> points_;
  std::vector<double> s_;
};

class Curve3d {
public:
  Curve3d(Eigen::Vector3d pt_begin, Eigen::Vector3d pt_end, int num_points);
  
  // data
  std::vector<Eigen::Vector3d> points_;
  int num_points_;
};

}