// Copyright (c) 2022 ChenJun
// Licensed under the MIT License.

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

// STD
#include <algorithm>
#include <cmath>
#include <vector>

#include "armor_detector/detector.hpp"
#include "auto_aim_interfaces/msg/debug_armor.hpp"
#include "auto_aim_interfaces/msg/debug_light.hpp"

namespace rm_auto_aim
{
Detector::Detector(
  const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a)
: binary_thres(bin_thres), detect_color(color), l(l), a(a)  //构造函数
{
}

std::vector<Armor> Detector::detect(const cv::Mat & input)  //装甲板识别完整调用流程  -> 主函数
{
  binary_img = preprocessImage(input);  //图像预处理
  lights_ = findLights(input, binary_img);  //找灯条
  armors_ = matchLights(lights_); //将灯条与装甲板匹配

  if (!armors_.empty()) { 
    classifier->extractNumbers(input, armors_); 
    classifier->classify(armors_);
  }

  return armors_;
}

cv::Mat Detector::preprocessImage(const cv::Mat & rgb_img)  //图像预处理
{
  cv::Mat gray_img;
  cv::cvtColor(rgb_img, gray_img, cv::COLOR_RGB2GRAY);

  cv::Mat binary_img;
  cv::threshold(gray_img, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  return binary_img;
}

std::vector<Light> Detector::findLights(const cv::Mat & rbg_img, const cv::Mat & binary_img)  //找灯条 传入原始图像rbg_img和二值化图像binary_img  返回值为构造好的灯条集 可判断灯条颜色
{
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);  //只找最高层级的轮廓

  vector<Light> lights;
  this->debug_lights.data.clear();  //处理前清空Detector中DebugLights类型的debug_lights数组中的数据

  for (const auto & contour : contours) { //遍历轮廓集
    if (contour.size() < 5) continue; //确保轮廓为四边形

    auto r_rect = cv::minAreaRect(contour); //获取最小外接矩形，返回值为RotatedRect(center(x,y),(width,height),rotatedAngle)
    auto light = Light(r_rect); //以获取的最小外接矩形构造灯条

    if (isLight(light)) {
      auto rect = light.boundingRect(); //计算并返回指定点集或灰度图像非零像素的最小上边界矩形  -> 这里返回二值化图像中灯条的最小外接矩形
      //返回矩形左上角顶点的x y以及矩形的width height
      if (  // Avoid assertion failed
        0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= rbg_img.cols && 0 <= rect.y &&
        0 <= rect.height && rect.y + rect.height <= rbg_img.rows) { //筛选，使最小外接矩形在二维平面上满足一定条件 -> 更合理？？？
        int sum_r = 0, sum_b = 0;
        auto roi = rbg_img(rect); //用rect创建感兴趣区域(识别区域)
        // Iterate through the ROI  ROI迭代
        for (int i = 0; i < roi.rows; i++) {  //遍历ROI区域
          for (int j = 0; j < roi.cols; j++) {
            if (cv::pointPolygonTest(contour, cv::Point2f(j + rect.x, i + rect.y), false) >= 0) { //检测识别到的灯条的左上角点叠加每次遍历的ROI区域后，是否在最高层级的轮廓内
              // if point is inside contour
              //若点在ROI区域内，则找出ROI区域内所有颜色为红/蓝的像素点，得到总数量
              sum_r += roi.at<cv::Vec3b>(i, j)[0];  //rgb顺序
              sum_b += roi.at<cv::Vec3b>(i, j)[2];
            }
          }
        }
        // Sum of red pixels > sum of blue pixels ?
        light.color = sum_r > sum_b ? RED : BLUE; //依据红色蓝色像素点的数目多少来判断当前识别灯条的颜色，并将该判断赋给Light类型对象中的color属性中
        lights.emplace_back(light); //将每轮循环构造的灯条添加到灯条集中
      }
    }
  }

  return lights;
}

bool Detector::isLight(const Light & light) //判断是否为灯条 返回值类型为bool
{
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length; //灯条长宽比
  bool ratio_ok = l.min_ratio < ratio && ratio < l.max_ratio; //根据标准的长宽比进行第一次判断

  bool angle_ok = light.tilt_angle < l.max_angle;   //根据垂直方向夹角进行第二次判断

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  auto_aim_interfaces::msg::DebugLight light_data;  //这块是"auto_aim_interfaces/msg/DebugLight.msg"中自定义的消息类型
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data); //向debug_lights这个DebugLight数组中添加light_data这个元素

  return is_light;  
}

std::vector<Armor> Detector::matchLights(const std::vector<Light> & lights) //匹配灯条
{
  std::vector<Armor> armors;
  this->debug_armors.data.clear();

  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_1->color != detect_color || light_2->color != detect_color) continue; //左右灯条均与目标识别颜色相符，则进行后续操作，否则跳过进行下一轮循环

      if (containLight(*light_1, *light_2, lights)) {
        continue;
      }

      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) { //装甲板合法  
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(const Light & light_1, const Light & light_2, const std::vector<Light> & lights)  
{
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom}; //分别获取左右灯条顶边和底边的中点，四个点
  auto bounding_rect = cv::boundingRect(points);  //以获取的四个点构建一个双灯条矩形

  for (const auto & test_light : lights) {  //遍历灯条参数集
    if (test_light.center == light_1.center || test_light.center == light_2.center) continue; //如果灯条集中某灯条的中心点和传入的左右两根灯条中的一根的中心点位置相同，则继续遍历

    if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) || bounding_rect.contains(test_light.center)) {   //判断灯条集中灯条的顶边中点、底边中点、中心点是否在传入的左右两根灯条构成的矩形面积中
      return true;   //有则直接返回true 结束 -> 说明灯条集中的该灯条在构建矩形中
    }
  }

  return false; //说明灯条集中的灯条在构建矩形边界或不在矩形内
}

ArmorType Detector::isArmor(const Light & light_1, const Light & light_2) //判断是否符合装甲板标准 同时返回装甲板类型
{
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length; //如果 灯条1长度 < 灯条2长度，则ratio为 灯条1长度/灯条2长度 -> 该数值理应<1且>0
  bool light_ratio_ok = light_length_ratio > a.min_light_ratio; //由 两根灯条的长度比大于一个给定的最小值 来判断识别到的两根灯条是否合理，防止灯条误识别

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2; //灯条平均长度
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length; //算出两灯条中点距离后/灯条平均长度
  bool center_distance_ok = (a.min_small_center_distance <= center_distance &&
                             center_distance < a.max_small_center_distance) ||
                            (a.min_large_center_distance <= center_distance &&
                             center_distance < a.max_large_center_distance);  //当灯条中点距离满足两个区间时置对应状态位为true
                             //一个小装甲板间距 一个大装甲板间距

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center; //两灯条中点相减得到的矩阵，即中点连线
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180; //计算连线与水平方向的夹角
  bool angle_ok = angle < a.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = center_distance > a.min_large_center_distance ? ArmorType::LARGE : ArmorType::SMALL; //根据两灯条中点距离判断装甲板大小
  } else {
    type = ArmorType::INVALID;
  }

  // Fill in debug information -> 每次调用isArmor时都会先根据传入的两个灯条完善好DebugArmor类型的armor_data中的数据
  auto_aim_interfaces::msg::DebugArmor armor_data;
  armor_data.type = ARMOR_TYPE_STR[static_cast<int>(type)];
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data); //最后将完善好的armor_data添加到在detector.hpp中定义好的DebugArmors类型数据集(debug_armors)中

  return type;  //返回装甲板类型
}

cv::Mat Detector::getAllNumbersImage()  
{
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto & armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

void Detector::drawResults(cv::Mat & img) //画出装甲板中心点
{
  // Draw Lights
  for (const auto & light : lights_) {
    cv::circle(img, light.top, 3, cv::Scalar(255, 255, 255), 1);  //标出灯条顶边中点
    cv::circle(img, light.bottom, 3, cv::Scalar(255, 255, 255), 1); //标出灯条底边中点
    auto line_color = light.color == RED ? cv::Scalar(255, 255, 0) : cv::Scalar(255, 0, 255); //判断灯条颜色
    cv::line(img, light.top, light.bottom, line_color, 1);  //通过上下两边中点连线勾勒灯条
  }

  // Draw armors
  for (const auto & armor : armors_) {  //由左右两根灯条构建装甲板中心点(两根连线交叉得到)
    cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 2);
    cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 2);
  }

  // Show numbers and confidence
  for (const auto & armor : armors_) {
    cv::putText(
      img, armor.classfication_result, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8,
      cv::Scalar(0, 255, 255), 2);  //将识别到的数字放到处理后的图片上，位置为左灯条顶点
  }
}

}  // namespace rm_auto_aim
