#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " <pcd_file>\n";
    return -1;
  }

  const std::string pcd_file(argv[1]);
  pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *point_cloud) == -1) {
    std::cout << "Couldn't read pcd file!\n";
    return -1;
  }

  constexpr int width = 2048;
  constexpr int height = 64;
  constexpr float fov_up = 3 * M_PI / 180.0;
  constexpr float fov_down = -25 * M_PI / 180.0;
  constexpr float fov = std::abs(fov_up) + std::abs(fov_down);
  const std::vector<float> image_means{12.12, 10.88, 0.23, -1.04, 0.21};
  const std::vector<float> image_stds{12.32, 11.47, 6.91, 0.86, 0.16};
  float* range_images = new float[5 * width * height]();

  for (const auto& point : point_cloud->points) {
    const auto& x = point.x;
    const auto& y = point.y;
    const auto& z = point.z;
    const auto& intensity = point.intensity;
    const float range = std::sqrt(x * x + y * y + z * z);
    const float yaw = -std::atan2(y, x);
    const float pitch = std::asin(z / range);

    float proj_x = 0.5f * (yaw / M_PI + 1.0f) * width;
    float proj_y = (1.0f - (pitch + std::abs(fov_down)) / fov) * height;
    proj_x = std::floor(proj_x);
    proj_y = std::floor(proj_y);

    const int u = std::clamp<int>(static_cast<int>(proj_x), 0, width - 1);
    const int v = std::clamp<int>(static_cast<int>(proj_y), 0, height - 1);

    range_images[0 * width * height + v * width + u] = (range - image_means.at(0)) / image_stds.at(0);
    range_images[1 * width * height + v * width + u] = (x - image_means.at(1)) / image_stds.at(1);
    range_images[2 * width * height + v * width + u] = (y - image_means.at(2)) / image_stds.at(2);
    range_images[3 * width * height + v * width + u] = (z - image_means.at(3)) / image_stds.at(3);
    range_images[4 * width * height + v * width + u] = (intensity - image_means.at(4)) / image_stds.at(4);
  }

  // 对range通道进行可视化
  cv::Mat range = cv::Mat(height, width, CV_32FC1, static_cast<void*>(range_images));
  cv::Mat normalized_range, u8_range, color_map;
  cv::normalize(range, normalized_range, 255, 0, cv::NORM_MINMAX);
  normalized_range.convertTo(u8_range, CV_8UC1);
  cv::applyColorMap(u8_range, color_map, cv::COLORMAP_JET);
  cv::imwrite("range_color_map.jpg", color_map);
  cv::imshow("Range Image", color_map);
  cv::waitKey(0);

  delete[] range_images;

  return 0;
}
