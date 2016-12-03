#ifndef ROVI1PLUGINPA10_HELPERFUNCTIONS_H
#define ROVI1PLUGINPA10_HELPERFUNCTIONS_H

#include <opencv2/opencv.hpp>
#include "rw/math.hpp"
#include <rw/kinematics.hpp>
#include "../build/ui_SamplePlugin.h"
//#include "SamplePlugin.hpp"
//#include "marker1.h"

enum NoOfTargets {single = 1, multiple = 3};

std::vector<rw::math::Vector3D<double>> hardcodedTextureTargetCoordinates(NoOfTargets No,
                                                                          rw::kinematics::Frame *textureFrame,
                                                                          rw::kinematics::Frame *cameraFrame,
                                                                          rw::kinematics::State state);

//std::vector<rw::math::Vector2D<int>> getTargetPixelCoordinates(cv::Mat im);


#endif //ROVI1PLUGINPA10_HELPERFUNCTIONS_H
