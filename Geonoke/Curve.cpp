#include "Curve.h"

#include <algorithm>
#include <cmath>

namespace Noke {

// ------------------------------------ Curve2d -----------------------------------------

Curve2d::Curve2d(const std::vector<EVector2d> &points): points_(points) {
  UpdateInfoFromPoints();
}

Curve2d::Curve2d(const Curve2d &ano_curve_2d): points_(ano_curve_2d.points_) {
  UpdateInfoFromPoints();
}

Curve2d::~Curve2d() {
}

void Curve2d::UpdateInfoFromPoints() {
  s_.resize(points_.size());
  s_[0] = 0.0;
  for (int i = 1; i < (int) points_.size(); i++) {
    s_[i] = s_[i - 1] + (points_[i] - points_[i - 1]).norm();
  }
}

double Curve2d::E2EDistance(const Curve2d &ano_curve, bool is_begin, bool ano_is_begin) {
  EVector2d my_pt, ano_pt;
  if (is_begin)
    my_pt = points_.front();
  else
    my_pt = points_.back();
  if (ano_is_begin)
    ano_pt = ano_curve.points_.front();
  else
    ano_pt = ano_curve.points_.back();
  return (my_pt - ano_pt).norm();
}

double Curve2d::LinkScore(const Curve2d &ano_curve, bool is_begin, bool ano_is_begin) {
  EVector2d my_pt, my_heading, ano_pt, ano_heading;
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
  double weight = 0.0;
  return distance_score * weight + heading_score * (1.0 - weight);
}

void Curve2d::Link(const Curve2d &ano_curve) {
  for (const auto &pt : ano_curve.points_) {
    points_.push_back(pt);
  }
}

void Curve2d::Reverse() {
  std::reverse(points_.begin(), points_.end());
}

void Curve2d::Resample(double hope_dis) {
  std::vector<double> dis;
  double sum_dis = 0;
  for (auto iter = points_.begin(); std::next(iter) != points_.end(); iter++) {
    auto next_iter = std::next(iter);
    dis.push_back((*iter - *next_iter).norm());
    sum_dis += dis.back();
  }
  double new_size = std::ceil(sum_dis / hope_dis);
  double new_dis = sum_dis / new_size;
  std::vector<Eigen::Vector2d> new_points;
  double res_dis = 0.0;
  for (int i = 0; i + 1 < points_.size(); i++) {
    // TODO: Hard code here.
    const double eps = 1e-9;
    while (res_dis > -eps && res_dis < dis[i] - eps) {
      double bias = res_dis / dis[i];
      new_points.emplace_back(bias * points_[i + 1] + (1.0 - bias) * points_[i]);
      res_dis += new_dis;
    }
    res_dis -= dis[i];
  }
  new_points.push_back(points_.back());
  points_ = std::move(new_points);
  UpdateInfoFromPoints();
}

double Curve2d::Length() {
  return s_.back();
}

void Curve2d::MeanConv(int n) {
  // TODO: Faster Conv.
  std::vector<Eigen::Vector2d> new_points;
  for (int i = 0; i < (int) points_.size(); i++) {
    int cnt = 0;
    Eigen::Vector2d pt(0.0, 0.0);
    for (int j = std::max(0, i - n); j < i + n + 1 && j < (int) points_.size(); j++) {
      cnt++;
      pt += points_[j];
    }
    new_points.emplace_back(pt / (double) cnt);
  }
  points_ = std::move(new_points);
  UpdateInfoFromPoints();
}

Eigen::VectorXd CalcPAD(double s, double r, int n) {
  // TODO.
  return Eigen::Vector2d(0, 0);
}

}