//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.h"
#include <Eigen>
#include <opencv2/opencv.hpp>
class Texture {
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        std::cout << name << std::endl;
        image_data = cv::imread(name);
        std::cout << image_data.cols <<" " <<image_data.rows<< std::endl;
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
        auto u_img = std::min(width - 1, int(u * width));
        auto v_img = std::min(height - 1, int((1 - v) * height));
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    Eigen::Vector3f getColorBilinnear(float u, float v)
    {
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) v = 0;
        if (v > 1) v = 1;
        u = u * width;
        v = (1 - v) * height;
        Eigen::Vector2i A(floor(u), ceil(v)), B(ceil(u), ceil(v)), C(floor(u), floor(v)), D(ceil(u), floor(v));
        A.x() = std::min(width - 1, A.x()); B.x() = std::min(width - 1, B.x()); C.x() = std::min(width - 1, C.x()); D.x() = std::min(width - 1, D.x());
        A.y() = std::min(height - 1, A.y()); B.y() = std::min(height - 1, B.y()); C.y() = std::min(height - 1, C.y()); D.y() = std::min(height - 1, D.y());
        float s = u - A.x(), t = v - C.y();
        auto ColorA = image_data.at<cv::Vec3b>(A.y(), A.x());
        auto ColorB = image_data.at<cv::Vec3b>(B.y(), B.x());
        auto ColorC = image_data.at<cv::Vec3b>(C.y(), C.x());
        auto ColorD = image_data.at<cv::Vec3b>(D.y(), D.x());
        auto ColorAB = (1 - s) * ColorA + s * ColorB;
        auto ColorCD = (1 - s) * ColorC + s * ColorD;
        auto color = (1 - t) * ColorCD + t * ColorAB;
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
