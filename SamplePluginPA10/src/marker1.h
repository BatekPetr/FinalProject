
#ifndef MARKER1_H
#define MARKER1_H

#include <opencv2/opencv.hpp>
#include "rw/math.hpp"
#include <algorithm>
#include "helperFunctions.h"

void sortTriangle(std::vector<rw::math::Vector2D<int> >& vec);
std::vector<rw::math::Vector2D<int> > marker1(cv::Mat src, NoOfTargets No);
double euclDist(int x1, int y1, int x2, int y2);

#endif // MARKER1_H


/*
#ifndef MARKER1_H
#define MARKER1_H

#include <opencv2/opencv.hpp>
#include "rw/math.hpp"
#include "helperFunctions.h"

std::vector<rw::math::Vector2D<int> > marker1(cv::Mat src, NoOfTargets No);

#endif // MARKER1_H
*/