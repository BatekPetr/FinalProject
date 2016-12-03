#include "helperFunctions.h"

std::vector<rw::math::Vector3D<double>> hardcodedTextureTargetCoordinates(NoOfTargets No,
                                                                          rw::kinematics::Frame *textureFrame,
                                                                          rw::kinematics::Frame *cameraFrame,
                                                                          rw::kinematics::State state)
{
    // Create vector of 3 hardcoded target points on the texture
    std::vector<rw::math::Vector3D<double>> textureTargetCoordinates;
    rw::math::Vector3D<double> targetOnTexture(0, 0, 0);
    rw::math::Vector3D<double> targetOnTexture2(0.1, 0, 0);
    rw::math::Vector3D<double> targetOnTexture3(0, 0.1, 0);

    textureTargetCoordinates.push_back(targetOnTexture);
    textureTargetCoordinates.push_back(targetOnTexture2);
    textureTargetCoordinates.push_back(targetOnTexture3);


    // Return specified noOfTargs coordinates
    std::vector<rw::math::Vector3D<double>>::const_iterator first = textureTargetCoordinates.begin();
    std::vector<rw::math::Vector3D<double>>::const_iterator last = textureTargetCoordinates.begin() + No;

    return std::vector<rw::math::Vector3D<double>>(first, last);
}

/*
std::vector<rw::math::Vector2D<int>> getTargetPixelCoordinates(cv::Mat im)
{
    std::vector<rw::math::Vector2D<int>> ptsPixelCoordinates;
    cv::Mat imflip;
    cv::flip(im, imflip, -1);
    ptsPixelCoordinates = marker1(imflip);
    return ptsPixelCoordinates;
}
*/
