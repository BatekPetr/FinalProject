
#include "inverseKinematics.hpp"


rw::math::VelocityScrew6D<double> calculateDeltaU(rw::math::Transform3D<double> baseTtool, rw::math::Transform3D<double> baseTtool_desired) {
    // Calculate dp
    rw::math::Vector3D<double> dp = baseTtool_desired.P() - baseTtool.P();

    // Calculate dw
    rw::math::EAA<double> dw(baseTtool_desired.R() * baseTtool.R().inverse());

    return rw::math::VelocityScrew6D<double>(dp, dw);
}

boost::numeric::ublas::vector<double> calculate_dUImage(std::vector<rw::math::Vector2D<int>> targetRealPixels, std::vector<rw::math::Vector2D<int>> targetPixels)
{
    boost::numeric::ublas::vector<double> dU(2*targetPixels.size());
    for (int i = 0; i < targetPixels.size(); ++i)
    {
        dU[2*i] = - ( targetRealPixels[i][0] - targetPixels[i][0] );
        dU[2*i + 1] = - ( targetRealPixels[i][1] - targetPixels[i][1] );
    }

    return dU;
}

boost::numeric::ublas::matrix<double> createS(rw::math::Rotation3D<double> R)
{
    boost::numeric::ublas::matrix<double> S(6,6,0);

    // fill S entries with transposed R on the diagonal
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
        {
            S(j, i) = R(i, j);
            S(j+3, i+3) = R(i, j);
        }

    return S;
}

rw::math::Jacobian calculateImageJ(std::vector<rw::math::Vector2D<int>> targetRealPixels, int focal_L, double z)
{
    int f = focal_L;
    rw::math::Jacobian J(2*targetRealPixels.size(),6);

    for (int i = 0; i< targetRealPixels.size(); ++i)
    {
        int u = targetRealPixels[i][0];
        int v = targetRealPixels[i][1];

        int tempI = 2*i;

        J(tempI, 0) = -f / z;
        J(tempI, 1) = 0;
        J(tempI, 2) = u / z;
        J(tempI, 3) = (u * v) / f;
        J(tempI, 4) = -(std::pow(f, 2) + std::pow(u, 2)) / f;
        J(tempI, 5) = v;

        J(tempI + 1, 0) = 0;
        J(tempI + 1, 1) = -f / z;
        J(tempI + 1, 2) = v / z;
        J(tempI + 1, 3) = (std::pow(f, 2) + std::pow(v, 2)) / f;
        J(tempI + 1, 4) = -(u * v) / f;
        J(tempI + 1, 5) = -u;
    }
    return J;
}

rw::math::Q saturateDQ(rw::math::Q deltaQ, rw::math::Q velocityLimits, double deltaT)
{
    int size = deltaQ.size();
    deltaT = deltaT/1000;
    for ( int i = 0; i < size; ++i)
    {
        if ( (std::abs(deltaQ[i])/deltaT) > (velocityLimits[i]) )
        {
            if( deltaQ[i] > 0)
                deltaQ[i] = velocityLimits[i] * deltaT;
            else
                deltaQ[i] = - velocityLimits[i] * deltaT;
        }
    }

    return deltaQ;
}

double euclideanDist(boost::numeric::ublas::vector<double> dU_Image)
{
    int eucD = 0;
    for (size_t i = 0; i < dU_Image.size(); ++i)
    {
        eucD += std::sqrt( std::pow(dU_Image[0+i], 2) + std::pow(dU_Image[1+i], 2) );
    }
    return eucD;
}

void writeErrsToFile(std::ofstream& writeStr, double deltaT, double maxEucD, boost::numeric::ublas::vector<double> max_dU_Image)
{

}

boost::numeric::ublas::matrix<double> calculate_Z_image(const rw::models::Device::Ptr device,
                                                        rw::kinematics::State state, rw::kinematics::Frame *cameraFrame,
                                                        rw::math::Jacobian J_image)
{
    // Calculate Manipulator Jacobian
    auto J = device->baseJframe(cameraFrame, state);

    // Create matrix S
    auto baseTcamera = device->baseTframe(cameraFrame, state);
    auto baseRcamera = baseTcamera.R(); //Retrieve rotational matrix from transformation of cameraFrame
    //log().info() << "baseRcamera: " << baseRcamera << "\n";
    //log().info() << "base -> camera RPY" << rw::math::RPY<double>(baseRcamera) << "\n";
    auto S = createS(baseRcamera);

    // Compute Z matrix for image
    boost::numeric::ublas::matrix<double> temp = boost::numeric::ublas::prod(S, J.m());

    boost::numeric::ublas::matrix<double> Z_image = boost::numeric::ublas::prod(J_image.m(), temp);

    return Z_image;
}

boost::numeric::ublas::vector<double> compute_dQ_LSM(boost::numeric::ublas::matrix<double> Z_image,
                                                     boost::numeric::ublas::vector<double> dU_Image)
{
    boost::numeric::ublas::matrix<double> Z_imageT = boost::numeric::ublas::trans(Z_image);


    //log().info() << Z_image << std::endl;
    // Compute matrix product (ZZ')Z
    boost::numeric::ublas::matrix<double> ZZTproduct = boost::numeric::ublas::prod(Z_image, Z_imageT);
    boost::numeric::ublas::matrix<double> ZZTproduct_inverse = rw::math::LinearAlgebra::pseudoInverse(ZZTproduct);
    //boost::numeric::ublas::matrix<double> Z3 = boost::numeric::ublas::prod(temp2, rw::math::LinearAlgebra::pseudoInverse(Z_imageT));
    boost::numeric::ublas::matrix<double> Z3inv = boost::numeric::ublas::prod(Z_imageT, ZZTproduct_inverse);

    // Find solution for dq
    // Create new boost vector for change in joints dQ
    boost::numeric::ublas::vector<double> dQ = boost::numeric::ublas::prod(Z3inv, dU_Image);

    return dQ;
}