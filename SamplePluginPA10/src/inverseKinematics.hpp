#include <rw/rw.hpp>
#include <iostream>

#ifndef ROVI1PLUGINPA10_INVERSEKINEMATICS_H
#define ROVI1PLUGINPA10_INVERSEKINEMATICS_H

rw::math::VelocityScrew6D<double> calculateDeltaU(rw::math::Transform3D<double> baseTtool, rw::math::Transform3D<double> baseTtool_desired);
boost::numeric::ublas::vector<double> calculate_dUImage(std::vector<rw::math::Vector2D<int>> targetRealPixels, std::vector<rw::math::Vector2D<int>> targetPixels);
boost::numeric::ublas::matrix<double> createS(rw::math::Rotation3D<double> R);
rw::math::Jacobian calculateImageJ(std::vector<rw::math::Vector2D<int>> targetRealPixels, int focal_L, double z = -0.5);
rw::math::Q saturateDQ(rw::math::Q deltaQ, rw::math::Q velocityLimits, double deltaT);
double euclideanDist(boost::numeric::ublas::vector<double> dU_Image);
void writeErrsToFile(std::ofstream& writeStr, double deltaT, double maxEucD, boost::numeric::ublas::vector<double> dU_Image);

boost::numeric::ublas::matrix<double> calculate_Z_image(const rw::models::Device::Ptr device,
                                                        rw::kinematics::State state, rw::kinematics::Frame *cameraFrame,
                                                        rw::math::Jacobian J_image);

boost::numeric::ublas::vector<double> compute_dQ_LSM(boost::numeric::ublas::matrix<double> Z_image,
                                                     boost::numeric::ublas::vector<double> dU_Image);


#endif //ROVI1PLUGINPA10_INVERSEKINEMATICS_H
