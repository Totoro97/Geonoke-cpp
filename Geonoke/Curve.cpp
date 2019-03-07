#include "Curve.h"
#include <cmath>

// ------------------------------------ Curve2d -----------------------------------------

Curve2d::Curve2d(const std::vector<EVector3d> points): points_(points) {
}

Curve2d::~Curve2d() {
}

double Curve2d::LinkScore(const Curve2d &ano_curve, bool is_begin, bool ano_is_begin) {
  EVector3d my_pt, my_heading, ano_pt, ano_heading;
  if (is_begin) {
    my_pt = points_.front();
    my_heading = points_[0] - points_[1];
  }
  else {
    my_pt = points_.back();
    my_heading = points_[points_.size() - 1] - points_[points_.size() - 2];
  }
  if (ano_is_begin) {
    ano_pt = ano_curve.points_.front();
    ano_heading = ano_curve.points_[0] - ano_curve.points_[1];
  }
  else {
    ano_pt = ano_curve.points_.back();
    int ano_pt_size = ano_curve.points_.size();
    ano_heading = ano_curve.points_[ano_pt_size - 1] - ano_curve.points_[ano_pt_size - 2];
  }
  double distance_score = std::exp(-(my_pt - ano_pt).norm());
  my_heading /= my_heading.norm();
  ano_heading /= ano_heading.norm();

  double heading_score = (my_heading.dot(-ano_heading) + 1.0) / 2.0;
  // TODO: Hard code here.
  double weight = 0.5;
  return distance_score * weight + heading_score * (1.0 - weight);
}