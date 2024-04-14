// Copyright 2022 Chen Jun

#ifndef ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_

// OpenCV
#include <opencv2/opencv.hpp>

// STL
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"

namespace rm_auto_aim
{
class NumberClassifier  //分类器？
{
public:
  NumberClassifier(
    const std::string & model_path, //
    const std::string & label_path, //
    const double threshold, 
    const std::vector<std::string> & ignore_classes = {});  

  void extractNumbers(const cv::Mat & src, std::vector<Armor> & armors);  //得到每个装甲板对应的透视变换后的二值化图像

  void classify(std::vector<Armor> & armors);

  double threshold; //识别度阈值

private:
  cv::dnn::Net net_;  //定义神经网络训练模型 -> 用于深度学习 匹配装甲板
  std::vector<std::string> class_names_;  //定义string类型的vector用于存储读取的ONNX模型中的数据 装甲板数字？
  std::vector<std::string> ignore_classes_; //手动写出需要排除的装甲板数字？
};
}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__NUMBER_CLASSIFIER_HPP_
