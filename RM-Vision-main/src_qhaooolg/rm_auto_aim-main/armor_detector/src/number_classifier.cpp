// Copyright 2022 Chen Jun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "armor_detector/armor.hpp"
#include "armor_detector/number_classifier.hpp"

namespace rm_auto_aim
{
NumberClassifier::NumberClassifier(
  const std::string & model_path, const std::string & label_path, const double thre,
  const std::vector<std::string> & ignore_classes)
: threshold(thre), ignore_classes_(ignore_classes)  //让threshold继承构造函数NumberClassifier创建时传入的thre常参？
{
  net_ = cv::dnn::readNetFromONNX(model_path);  //定义一个神经网络模型，使其加载已有的ONNX模型

  std::ifstream label_file(label_path); //输入流 对某个文件只执行输入 ifstream->input file stream
  std::string line; 
  while (std::getline(label_file, line)) { 
    class_names_.push_back(line); //将line中数据添加到vector<string>类型的class_names_中 -> 这里指读取label_file文件中的每一行，若有值则将其存到class_names_中
  }
}

//得到每个装甲板对应的透视变换后的二值化图像
void NumberClassifier::extractNumbers(const cv::Mat & src, std::vector<Armor> & armors)
{
  // Light length in image
  const int light_length = 12;
  // Image size after warp
  const int warp_height = 28;   //图像扭曲后的height (给定值，width由此等比例得到)
  const int small_armor_width = 32; //图像扭曲后装甲板小 大宽度 -> 考虑透视变换
  const int large_armor_width = 54;
  // Number ROI size
  const cv::Size roi_size(20, 28);  //定义感兴趣区域面积(识别面积)

  for (auto & armor : armors) { //遍历装甲板集
    // Warp perspective transform
    cv::Point2f lights_vertices[4] = {
      armor.left_light.bottom, armor.left_light.top, armor.right_light.top,
      armor.right_light.bottom};  //定义通过左右两个灯条的四个边角点构建矩形的四个点

    const int top_light_y = (warp_height - light_length) / 2 - 1; 
    const int bottom_light_y = top_light_y + light_length;  //通过计算得出灯条部分在y轴上的两个端点
    const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
    //判断装甲板类型：若为小板则将其扭曲后的width赋值为小宽度
    cv::Point2f target_vertices[4] = {  
      cv::Point(0, bottom_light_y),
      cv::Point(0, top_light_y),
      cv::Point(warp_width - 1, top_light_y),
      cv::Point(warp_width - 1, bottom_light_y),
    };  //得到识别区域
    cv::Mat number_image;
    auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices); //根据实际灯条在现实中的透视点集和所要识别的正矩形点集得到透视变换矩阵
    cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height)); //将函数传入的实际图像根据得到的透视变换矩阵进行透视变换，

    // Get ROI
    number_image =
      number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

    // Binarize
    cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
    cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    armor.number_img = number_image;  //返回经过透视变换后的含装甲板数字的二值化图像，并赋给armor中的number_img
  } //对装甲板集中的每个装甲板都进行以上操作
}

void NumberClassifier::classify(std::vector<Armor> & armors)  //分类？
{
  for (auto & armor : armors) {
    cv::Mat image = armor.number_img.clone();

    // Normalize
    image = image / 255.0;  //使图像进行归一化，得到的数值范围为[0, 1]，彩色图片会变成灰图

    // Create blob from image
    cv::Mat blob;
    cv::dnn::blobFromImage(image, blob);  //图片预处理 -> 使图片参数满足进行神经网络训练的要求

    // Set the input blob for the neural network
    net_.setInput(blob);  //将预处理后的图片传入神经网络进行训练
    // Forward pass the image blob through the model
    cv::Mat outputs = net_.forward();   //将深度学习后的图片传出到outputs中

    // Do softmax
    float max_prob = *std::max_element(outputs.begin<float>(), outputs.end<float>()); //找出深度学习后图像中的最大像素点
    cv::Mat softmax_prob;
    cv::exp(outputs - max_prob, softmax_prob);  //双边滤波？  滤波后将结果赋给softmax_prob
    float sum = static_cast<float>(cv::sum(softmax_prob)[0]); //计算出softmax_prob矩阵中第一行元素的和，强制静态转化为float类型并把结果赋给sum
    softmax_prob /= sum;  //矩阵中每个元素都除以其第一行元素之和

    double confidence;  //识别度 原理？咋算的？
    cv::Point class_id_point;
    minMaxLoc(softmax_prob.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);  //reshape将softmax_prob转为单通道单行多列矩阵，然后得到转换后矩阵中的最大值赋给confidence，该元素位置赋给class_id_point
    int label_id = class_id_point.x;  

    armor.confidence = confidence;
    armor.number = class_names_[label_id];  //给装甲板数字赋值为class_names_中label_id这个下标对应的数据 

    std::stringstream result_ss;  //定义字符串流
    result_ss << armor.number << ": " << std::fixed << std::setprecision(1)
              << armor.confidence * 100.0 << "%"; //向result_ss中输入内容 fixed用于消除浮点数的科学计数法 再转成百分比格式的识别度(string类型)
    armor.classfication_result = result_ss.str();
  }

  armors.erase( //重写erase函数 使其可以自主遍历装甲板集，将其中不符合标准的装甲板对象移至vector末尾以变相达到删除不合理数据的需求
    std::remove_if( //重写std::remove_if函数 重写该函数时，删除操作需要通过容器的成员函数来进行 故一般与erase配套使用
      armors.begin(), armors.end(), //遍历装甲板集
      
      [this](const Armor & armor) { //重写回调函数  传入参数为Armor类型
        if (armor.confidence < threshold) {   //第一次筛选：当装甲板识别度低于阈值时
          return true;  //返回true -> 即将正在遍历的装甲板对象移动至vector末尾
        }

        for (const auto & ignore_class : ignore_classes_) {   //遍历ignore_classes_，当装甲板数字与其遍历值相等时，返回true并将该装甲板对象移至末尾
          if (armor.number == ignore_class) {
            return true;
          }
        }

        bool mismatch_armor_type = false;
        if (armor.type == ArmorType::LARGE) { //对大装甲板类型进行判断
          mismatch_armor_type =
            armor.number == "outpost" || armor.number == "2" || armor.number == "guard";  //满足任一条件就会使函数返回true，让该装甲板对象移至末尾
        } else if (armor.type == ArmorType::SMALL) {  //小装甲板
          mismatch_armor_type = armor.number == "1" || armor.number == "base";
        }
        return mismatch_armor_type; //true则移至末尾
      }),
    armors.end());
}

}  // namespace rm_auto_aim
