//
// Created by ASUS on 2023/12/10.
//

#ifndef CLION_PERSONAL_DETECTOR_HPP
#define CLION_PERSONAL_DETECTOR_HPP

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <cmath>
#include <string>
#include <vector>


namespace personal_aim{
    class Detector{
    public:
        struct LightParams
        {
            // width / height
            double min_ratio;
            double max_ratio;
            // vertical angle
            double max_angle;
        };

        struct ArmorParams
        {
            double min_light_ratio;
            // light pairs distance
            double min_small_center_distance;
            double max_small_center_distance;
            double min_large_center_distance;
            double max_large_center_distance;
            // horizontal angle
            double max_angle;
        };



    };
}

#endif //CLION_PERSONAL_DETECTOR_HPP
