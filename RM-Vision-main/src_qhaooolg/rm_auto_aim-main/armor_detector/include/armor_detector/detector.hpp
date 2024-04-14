// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__DETECTOR_HPP_
#define ARMOR_DETECTOR__DETECTOR_HPP_

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>

// STD
#include <cmath>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"
#include "auto_aim_interfaces/msg/debug_armors.hpp"
#include "auto_aim_interfaces/msg/debug_lights.hpp"

namespace rm_auto_aim
{
class Detector  //这里更像是声明Detector，只给出其中包含的变量和函数而不给出定义(实现代码)
{
public:
  struct LightParams  //灯条参数 -> 用于判断是否为灯条
  {
    // width / height -> 用于根据灯条长宽比判断灯条，给出匹配范围
    double min_ratio; 
    double max_ratio;
    // vertical angle
    double max_angle; //垂直方向最大倾斜角
  };

  struct ArmorParams  //装甲板参数
  {
    double min_light_ratio; 
    // light pairs distance -> 作为灯条中点距离的阈值，筛选灯条
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle; 
  };

  Detector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a);

  std::vector<Armor> detect(const cv::Mat & input);

  cv::Mat preprocessImage(const cv::Mat & input);
  std::vector<Light> findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img);
  std::vector<Armor> matchLights(const std::vector<Light> & lights);

  // For debug usage
  cv::Mat getAllNumbersImage();
  void drawResults(cv::Mat & img);

  int binary_thres; //图片二值化参数 用于图片预处理时作为二值化的参数传入
  int detect_color;
  LightParams l;
  ArmorParams a;

  std::unique_ptr<NumberClassifier> classifier;

  // Debug msgs
  cv::Mat binary_img;
  auto_aim_interfaces::msg::DebugLights debug_lights;
  auto_aim_interfaces::msg::DebugArmors debug_armors;

private:
  bool isLight(const Light & possible_light);
  bool containLight(
    const Light & light_1, const Light & light_2, const std::vector<Light> & lights); //
  ArmorType isArmor(const Light & light_1, const Light & light_2);

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__DETECTOR_HPP_
