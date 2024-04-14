// Copyright 2022 Chen Jun
// Licensed under the MIT License.

#ifndef ARMOR_DETECTOR__ARMOR_HPP_  //防止文件中的内容被重复引用/定义   ==if not def ARMOR_DETECTOR__ARMOR_HPP_ 即如果没有这个头文件就进行引用，若有则不引用
#define ARMOR_DETECTOR__ARMOR_HPP_  //与上面的#ifdef 配套使用

#include <opencv2/core.hpp> 

// STL
#include <algorithm>
#include <string>

namespace rm_auto_aim
{
const int RED = 0;
const int BLUE = 1;

enum class ArmorType { SMALL, LARGE, INVALID }; //装甲板分为三个类型：小板、大板、非装甲板(识别到的物体不匹配)
const std::string ARMOR_TYPE_STR[3] = {"small", "large", "invalid"};  

struct Light : public cv::RotatedRect //单个灯条的各项参数
{
  Light() = default;  
  explicit Light(cv::RotatedRect box) : cv::RotatedRect(box)  //自定义旋转矩形类型Light的构造方式，explicit显式构造在该命名空间内共享该构造方式
  {
    cv::Point2f p[4]; 
    box.points(p);  //将旋转矩形box四个定点的(x,y)值赋值给p[4]
    std::sort(p, p + 4, [](const cv::Point2f & a, const cv::Point2f & b) { return a.y < b.y; });
    //对p[4]中存储的四个顶点位置进行排序，按照每个点的y值从小到大依次排序，使得四个顶点从上到下依次为p[0],p[1],p[2],p[3]
    top = (p[0] + p[1]) / 2;  
    bottom = (p[2] + p[3]) / 2; //分别根据上下两组顶点得到各自的中点

    length = cv::norm(top - bottom);  //求解上下中点构成的矩阵的范数 -> 得到两点间距离，即灯条的长度
    width = cv::norm(p[0] - p[1]);

    tilt_angle = std::atan2(std::abs(top.x - bottom.x), std::abs(top.y - bottom.y));
    tilt_angle = tilt_angle / CV_PI * 180;
    //正常来说，atan2(double y,double x)是先传y再传x，返回值为该店与原点连线与x轴正方向的夹角 -> 这里先传x再传y应该是得到连线与y轴正方向的夹角，即垂直方向的夹角
  }

  int color;
  cv::Point2f top, bottom;
  double length;  //灯条长度
  double width; //灯条宽度
  float tilt_angle; //垂直方向夹角
};

struct Armor
{
  Armor() = default;  //默认构造函数
  Armor(const Light & l1, const Light & l2) //自定义构造方法：以只读方式传入识别到的两根灯条
  {
    if (l1.center.x < l2.center.x) {  //根据灯条的中心点x轴位置判断左右灯条
      left_light = l1, right_light = l2;
    } else {
      left_light = l2, right_light = l1;
    }
    center = (left_light.center + right_light.center) / 2;  //然后根据左右两个灯条的中心点获取装甲板的中心点
  }

  // Light pairs part
  Light left_light, right_light;
  cv::Point2f center;
  ArmorType type;

  // Number part
  cv::Mat number_img; //由摄像头传入的原始图像，经透视变换后的二值化图像
  std::string number;   //装甲板数字
  float confidence; //识别度 浮点数 经计算得到
  std::string classfication_result; //识别度结果 由confidence转成百分比格式得到
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR__ARMOR_HPP_
