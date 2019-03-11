#include "Curve.h"

#include <algorithm>
#include <cmath>
#include <iostream>

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
    if ((pt - points_.back()).norm() < 1e-9)
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

double Curve2d::Curvature(double s) {
  auto ptr = std::lower_bound(s_.data(), s_.data() + (int) s_.size(), s - 1e-9);
  int idx = (int)((ptr - s_.data())) - 1;
  if (idx + 1 == points_.size()) {
    idx--;
  }
  if (idx == -1) {
    idx++;
  }
  double k0 = CurvatureIndex(idx);
  double k1 = CurvatureIndex(idx + 1);
  double weight = (s - s_[idx]) / (s_[idx + 1] - s_[idx]);
  // For debug.
  // std::cout << "idx = " << idx << std::endl;
  // std::cout << "Length = " << s_.back() << std::endl;
  // std::cout << s_[idx] << " " << s << " " << s_[idx + 1] << std::endl;
  // std::cout << "weight = " << weight << std::endl;
  return k1 * weight + k0 * (1.0 - weight);
}


double Curve2d::CurvatureIndex(int idx) {
  if (points_.size() == 2) {
    return 0.0;
  }
  if (idx == 0) {
    idx = 1;
  }
  else if (idx + 1 == points_.size() - 1) {
    idx--;
  }
  // Curvature of 3 points:
  // 4*triangleArea/(sideLength1*sideLength2*sideLength3)
  // https://stackoverflow.com/questions/41144224/calculate-curvature-for-3-points-x-y
  auto v0 = points_[idx] - points_[idx - 1];
  auto v1 = points_[idx + 1] - points_[idx];
  return 2.0 * (v0(0) * v1(1) - v0(1) * v1(0)) / (v0.norm() * v1.norm() * (v0 + v1).norm());
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

Eigen::VectorXd Curve2d::CalcPAD(double s, double r, int n) {
  // TODO: Hard code here.
  const double eps = 1e-9;
  const double inf = 1e9;
  const double s_step = 0.125;
  Eigen::VectorXd pad;
  pad = Eigen::VectorXd::Zero(n * 2);
  {
    double expect_k = r;
    double current_k = 0.0;
    double current_pos = s - 0.5 * s_step;
    for (int t = 0; t < n; t++) {
      while (current_pos > eps && current_k + eps < expect_k) {
        current_k += s_step * std::abs(Curvature(current_pos));
        current_pos -= s_step;
      }
      if (current_k + eps < expect_k) {
        pad(n - t - 1) = inf;
      }
      else {
        pad(n - t - 1) = s - current_pos;
      }
      expect_k *= 2.0;
    }
    double weight = 1.0;
    for (int t = 0; t < n; t++) {
      pad(n - t - 1) = pad(n - t - 1) * weight / pad(0);
      weight *= 0.5;
    }
  }
  {
    double expect_k = r;
    double current_k = 0.0;
    double current_pos = s + 0.5 * s_step;
    for (int t = 0; t < n; t++) {
      while (current_pos < s_.back() - eps && current_k + eps < expect_k) {
        current_k += s_step * std::abs(Curvature(current_pos));
        current_pos += s_step;
      }
      if (current_k + eps < expect_k) {
        pad[n + t] = inf;
      }
      else {
        pad[n + t] = current_pos - s;
      }
      expect_k *= 2.0;
    }
    double weight = 1.0;
    for (int t = 0; t < n; t++) {
      pad(n + t) = pad(n + t) * weight / pad(n * 2 - 1);
      weight *= 0.5;
    }
  }
  return pad;
}

}