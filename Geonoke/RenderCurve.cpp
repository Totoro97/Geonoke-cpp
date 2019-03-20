#include <algorithm>
#include <cmath>
#include <iostream>
#include <Eigen>
#include <opencv2/opencv.hpp>

double Random() {
  return (double) std::rand() / (double) RAND_MAX;
}

double RandomLR(double l, double r) {
  return l + (r - l) * Random();
}

Eigen::Matrix3d CalcEssential(const Eigen::Vector3d &pas_pos,
                               const Eigen::Vector3d &pas_dir_x,
                               const Eigen::Vector3d &pas_dir_y,
                               const Eigen::Vector3d &pos,
                               const Eigen::Vector3d &dir_x,
                               const Eigen::Vector3d &dir_y) {
  auto pas_dir_z = pas_dir_x.cross(pas_dir_y);
  auto dir_z = dir_x.cross(dir_y);
  Eigen::Vector3d t;
  t(0) = (pas_pos - pos).dot(dir_x);
  t(1) = (pas_pos - pos).dot(dir_y);
  t(2) = (pas_pos - pos).dot(dir_z);
  Eigen::Matrix3d R;
  R(0, 0) = pas_dir_x.dot(dir_x); R(0, 1) = pas_dir_x.dot(dir_y); R(0, 2) = pas_dir_x.dot(dir_z);
  R(1, 0) = pas_dir_y.dot(dir_x); R(1, 1) = pas_dir_y.dot(dir_y); R(1, 2) = pas_dir_y.dot(dir_z);
  R(2, 0) = pas_dir_z.dot(dir_x); R(2, 1) = pas_dir_z.dot(dir_y); R(2, 2) = pas_dir_z.dot(dir_z);
  Eigen::Matrix3d T;
  T(0, 0) = 0; T(0, 1) = -t(2); T(0, 2) = t(1);
  T(1, 0) = t(2); T(1, 1) = 0; T(1, 2) = -t(0);
  T(2, 0) = -t(1); T(2, 1) = t(0); T(2, 2) = 0;
  return R * T;
}

int main() {
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> dirs;
  int n = 10;
  for (int i = 0; i < n; i++) {
    points.emplace_back(RandomLR(-1.0, 1.0), RandomLR(-1.0, 1.0), RandomLR(-1.0, 1.0));
  }
  for (int i = 0; i < n; i++) {
    dirs.emplace_back(RandomLR(-1.0, 1.0), RandomLR(-1.0, 1.0), RandomLR(-1.0, 1.0));
  }
  // Cam parameters.
  Eigen::Vector3d pos(7.0, 0.0, 0.0);
  Eigen::Vector3d dir_x(0.0, 1.0, 0.0);
  Eigen::Vector3d dir_y(0.0, 0.0, -1.0);
  double focal_length = 500.0;

  // Motion.
  Eigen::Vector3d rot(0.0, 0.0, 1.0);
  int n_views = 64;
  // VGG?
  int height = 224;
  int width = 224;
  for (int cnt = 0; cnt < n_views; cnt++) {
    auto dir_z = dir_x.cross(dir_y);
    cv::Mat img(height, width, CV_8UC1);
    std::memset(img.data, 0, height * width);
    for (int i = 0; i + 1 < n; i++) {
      auto p0 = points[i];
      auto p1 = dirs[i];
      auto p2 = points[i + 1] * 2.0 - dirs[i + 1];
      auto p3 = points[i + 1];
      for (double t = 0; t < 1; t += 0.001) {
        double s = 1.0 - t;
        auto pt = s * s * s * p0 + 3 * s * s * t * p1 + 3 * s * t * t * p2 + t * t * t * p3;
        double x = (pt - pos).dot(dir_x);
        double y = (pt - pos).dot(dir_y);
        double z = (pt - pos).dot(dir_z);
        x = x / z * focal_length + width * 0.5;
        y = y / z * focal_length + height * 0.5;
        int a = std::round(y);
        int b = std::round(x);
        if (a < height && a >= 0 && b < width && b >= 0) {
          img.data[a * width + b] = 255;
        }
      }
    }
    cv::imwrite("tmp_" + std::to_string(cnt) + ".png", img);
    std::cout << "pos " << cnt << ": " << pos.transpose() << " " << pos.norm() << std::endl;

    auto pas_pos = pos;
    auto pas_dir_x = dir_x;
    auto pas_dir_y = dir_y;
    auto pas_dir_z = dir_z;

    Eigen::Matrix3d T;
    T = Eigen::AngleAxisd(0.1, rot);
    double step_len = 0.2;
    rot += Eigen::Vector3d(RandomLR(-step_len, step_len), RandomLR(-step_len, step_len), RandomLR(-step_len, step_len));
    rot /= rot.norm();
    pos = T * pos;
    dir_z = -pos / pos.norm();
    if (std::abs(dir_z(2)) > 1e-9) {
      dir_y = Eigen::Vector3d(-dir_z(0), -dir_z(1), (dir_z(0) * dir_z(0) + dir_z(1) * dir_z(1)) / dir_z(2));
      if (dir_y(2) > 0.0) {
        dir_y = -dir_y;
      }
    }
    else {
      dir_y = Eigen::Vector3d(0.0, 0.0, -1.0);
    }
    dir_y /= dir_y.norm();
    dir_x = dir_y.cross(dir_z);
    // std::cout << dir_y.dot(dir_z) << " " << dir_x.norm() << " " << dir_y.norm() << " " << dir_z.norm() << std::endl;
    // Essential matrix.
    Eigen::Matrix3d E;
    E = CalcEssential(pas_pos, pas_dir_x, pas_dir_y, pos, dir_x, dir_y);
    // Check.
    for (int i = 0; i + 1 < n; i++) {
      auto p0 = points[i];
      auto p1 = dirs[i];
      auto p2 = points[i + 1] * 2.0 - dirs[i + 1];
      auto p3 = points[i + 1];
      for (double t = 0; t < 1; t += 0.001) {
        double s = 1.0 - t;
        auto pt = s * s * s * p0 + 3 * s * s * t * p1 + 3 * s * t * t * p2 + t * t * t * p3;
        Eigen::Vector3d p0, p1;
        {
          double x = (pt - pas_pos).dot(pas_dir_x);
          double y = (pt - pas_pos).dot(pas_dir_y);
          double z = (pt - pas_pos).dot(pas_dir_z);
          p0 = Eigen::Vector3d(x, y, z);
        }
        {
          double x = (pt - pos).dot(dir_x);
          double y = (pt - pos).dot(dir_y);
          double z = (pt - pos).dot(dir_z);
          p1 = Eigen::Vector3d(x, y, z);
        }
        if (std::abs(p0.transpose() * E * p1) > 1e-7) {
          std::cout << "fuck essential." << std::endl;
          std::exit(0);
        }
      }
    }
    std::cout << E << std::endl;
  }
  return 0;
}