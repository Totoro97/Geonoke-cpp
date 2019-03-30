#include <algorithm>
#include <cmath>
#include <cnpy.h>
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
  auto ret = R * T;
  if (ret.norm() > 1e-9) {
    return ret / ret.norm();
  }
  else {
    return ret;
  }
}

Eigen::Matrix3d CalcR(const Eigen::Vector3d &pas_pos,
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
  return R;
}

/*
 *  std::ofstream tmp_file;
  tmp_file.open(map_path.c_str(), std::ios::binary | std::ios::ate);
  tmp_file.write((const char *) dist_map_, width_ * height_ * sizeof(double));
  tmp_file.close();
 */
int main(int argc, char *argv[]) {
  std::map<std::string, int> args;
  args["height"] = 224;
  args["width"] = 224;
  args["n_views"] = 200;
  args["srand"] = 0;
  args["n_anchors"] = 5;
  args["out_info"] = 1;
  args["show_gif"] = 1;
  args["write_data"] = 1;
  for (int i = 1; i < argc; i++) {
    if (argv[i][0] != '-')
      continue;
    int p = 1;
    for (; argv[i][p] && argv[i][p] != '='; p++);
    if (!argv[i][p]) {
      continue;
    }
    std::string arg_name(argv[i] + 1, argv[i] + p);
    int arg_val = 0;
    for (p++; argv[i][p]; p++)
      arg_val = arg_val * 10 + (argv[i][p] - '0');
    args[arg_name] = arg_val;
  }
  int n_views = args["n_views"];
  int height = args["height"];
  int width = args["width"];
  std::cout << height << " " << width << std::endl;
  int n = args["n_anchors"];
  int out_info = args["out_info"];
  int show_gif = args["show_gif"];
  int write_data = args["write_data"];
  std::srand(args["srand"]);
  // std::ofstream out_file;
  // out_file.open("/home/aska/Data/train_curve/train_curve.bin", std::ios::binary | std::ios::ate);
  // out_file.open("/home/aska/Data/train_curve/val_curve.bin", std::ios::binary | std::ios::ate);
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector3d> dirs;
  // out_file.write("CURV", 4);
  // out_file.write((const char *) &n_views, 4);
  // out_file.write((const char *) &height, 4);
  // out_file.write((const char *) &width, 4);
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
  double focal_length = 500.0 * height / 224.0;

  // Motion.
  Eigen::Vector3d rot(RandomLR(-1.0, 1.0), RandomLR(-1.0, 1.0), RandomLR(-1.0, 1.0));
  rot /= rot.norm();
  cv::Mat past_img(height, width, CV_8UC1);
  Eigen::Vector3d pas_pos = pos;
  Eigen::Vector3d pas_dir_x = dir_x;
  Eigen::Vector3d pas_dir_y = dir_y;

  auto out_image_data = new uint8_t[n_views * height * width * 2];
  auto out_mat_data = new float[n_views * 9];

  auto current_image_ptr = out_image_data;
  auto current_mat_ptr = out_mat_data;

  for (int cnt = 0; cnt < n_views; cnt++) {
    if (out_info) {
     std::cout << "------------------------------------------" << std::endl;
    }
    auto dir_z = dir_x.cross(dir_y);
    auto pas_dir_z = pas_dir_x.cross(pas_dir_y);
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
    if (write_data) {
      cv::imwrite("/home/aska/Data/images/" + std::to_string(cnt) + ".png", img);
    }
    if (show_gif) {
      cv::imshow("tmp", img);
      cv::waitKey(50);
    }
    if (!cnt) {
      std::memcpy(past_img.data, img.data, height * width);
    }
    // std::cout << "pos " << cnt << ": " << pos.transpose() << " " << pos.norm() << std::endl;

    // Essential matrix.
    Eigen::Matrix3d E;
    E = CalcEssential(pas_pos, pas_dir_x, pas_dir_y, pos, dir_x, dir_y);
    Eigen::Matrix3d R;
    R = CalcR(pas_pos, pas_dir_x, pas_dir_y, pos, dir_x, dir_y);
    // std::cout << E.norm() << std::endl;
    // E *= 100;
    // std::cout << "E:" << std::endl << E << std::endl;

    // Check.
    std::vector<cv::Point2d> in_points_0, in_points_1;
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
          if (int(t * 1000) % 1 == 0)
           in_points_0.emplace_back(x / z, y / z);
        }
        {
          double x = (pt - pos).dot(dir_x);
          double y = (pt - pos).dot(dir_y);
          double z = (pt - pos).dot(dir_z);
          p1 = Eigen::Vector3d(x, y, z);
          if (int(t * 1000) % 1 == 0)
            in_points_1.emplace_back(x / z, y / z);
        }
        if (std::abs(p0.transpose() * E * p1) > 1e-7) {
          std::cout << "fuck essential." << std::endl;
          std::exit(0);
        }
      }
    }
    /*auto Es = cv::findFundamentalMat(
      in_points_1, in_points_0, cv::FM_LMEDS, 3.0, 0.999);
    Eigen::Matrix3d E_;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        E_(i, j) = Es.at<double>(i, j);
        // Es.at<double>(i, j) = E(i, j);
      }
    }
    E_ /= E_.norm();*/
    /*for (int i = 0; i + 1 < n; i++) {
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
        std::cout << std::abs(p0.transpose() * E_ * p1) << std::endl;
      }
    }*/
    /*cv::Mat r1, r2, t;
    cv::decomposeEssentialMat(Es, r1, r2, t);
    Eigen::Matrix3d R1, R2;
    if (r1.type() != CV_64FC1) {
      std::cout << "fuck" << std::endl;
      exit(0);
    }
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        R1(i, j) = r1.at<double>(i, j);
        R2(i, j) = r1.at<double>(i, j);
      }
    }*/
    // std::cout << "E_" << std::endl << E_ << std::endl;
    // std::cout << "R1: " << R1.determinant() << std::endl << R1 << std::endl
    // << "R2: " << R2.determinant() << std::endl << R2 << std::endl
    // << "R:  " << R.determinant()  << std::endl << R << std::endl;
    // out_file.write((const char *) past_img.data, height * width);
    // out_file.write((const char *) img.data, height * width);
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        *current_image_ptr = past_img.at<uint8_t>(i, j);
        current_image_ptr++;
      }
    }
    for (int i = 0; i < height; i++) {
      for (int j = 0; j < width; j++) {
        *current_image_ptr = img.at<uint8_t>(i, j);
        current_image_ptr++;
      }
    }
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        float tmp = (float) R(i, j);
        // out_file.write((const char *) &tmp, 4);
        *current_mat_ptr = tmp;
        current_mat_ptr++;
      }
    }
    std::memcpy(past_img.data, img.data, height * width);
    pas_pos = pos;
    pas_dir_x = dir_x;
    pas_dir_y = dir_y;

    // change_pose.
    Eigen::Matrix3d T;
    T = Eigen::AngleAxisd(0.05, rot);
    double step_len = 0.2;
    rot += Eigen::Vector3d(RandomLR(-step_len, step_len), RandomLR(-step_len, step_len), RandomLR(-step_len, step_len));
    rot /= rot.norm();
    pos = T * pos;
    // pos *= RandomLR(0.99, 1.01);
    dir_z = -pos / pos.norm();
    // Eigen::Vector3d rt(RandomLR(-0.002, 0.002), RandomLR(-0.002, 0.002), RandomLR(-0.002, 0.002));
    // dir_z = Eigen::AngleAxisd(rt.norm(), rt / rt.norm()) * dir_z;
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
  }
  // out_file.close();
  if (write_data) {
    cnpy::npy_save("/home/aska/Data/train_curve/img.npy",
                   out_image_data, {uint32_t(n_views), 2, uint32_t(height), uint32_t(width)}, "w");
    cnpy::npy_save("/home/aska/Data/train_curve/mat.npy",
                   out_mat_data, {uint32_t(n_views), 9}, "w");
  }
  delete[](out_image_data);
  delete[](out_mat_data);
  return 0;
}
