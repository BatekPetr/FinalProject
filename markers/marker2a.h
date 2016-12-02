#ifndef MARKER2A_H
#define MARKER2A_H

#include <opencv2/opencv.hpp>

    void marker2a(std::string filename);
    void houghCallback(int, void*);
    cv::Mat skeleton(cv::Mat imgIn);

#endif // MARKER2A_H
