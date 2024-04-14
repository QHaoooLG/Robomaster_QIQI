//#ifndef CLION_PERSONAL_DETECTOR_HPP
//#define CLION_PERSONAL_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace personal_aim{
    Detector::Dectector(const int & bin_thres, const int & color, const LightParams & l, const ArmorParams & a): binary_thres(bin_thres), detect_color(color), l(l), a(a)
    {
    }


}


//#endif //CLION_PERSONAL_DETECTOR_HPP