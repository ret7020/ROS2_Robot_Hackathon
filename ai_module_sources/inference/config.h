#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#define MODEL_SCALE 0.0039216
#define MODEL_MEAN 0.0
#define MODEL_CLASS_CNT 3
#define MODEL_THRESH 0.6
#define MODEL_NMS_THRESH 0.6

#define CENTER_COLOR cv::Scalar(200, 100, 30)

// 0 - яблоко зелёное
// 1 - яблоко красное
// 2 - помидор
cv::Scalar color_map[3] = {cv::Scalar(0, 255, 0),
                           cv::Scalar(0, 0, 0),
                           cv::Scalar(0, 0, 255)};

// Calibrated via checkerboard
cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 727.97277723, 0, 308.83841529,
                         0, 723.12158831, 270.40274403,
                         0, 0, 1);
cv::Mat dist_coeffs = (cv::Mat_<double>(5, 1) << -0.71612235, 0.61866812, -0.03516522, 0.00746986, -0.35457296);