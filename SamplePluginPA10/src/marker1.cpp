
#include "marker1.h"

using namespace cv;
using namespace std;
using namespace rw::math;

double euclDist(int x1, int y1, int x2, int y2) {
    return sqrt( pow( x1-x2, 2) + pow( y1-y2, 2) );
}

void sortTriangle(std::vector<rw::math::Vector2D<int> >& vec) {
    double maxDist = 0, curDist;
    size_t maxIndex, maxJndex;
    for (size_t i = 0; i < 3; ++i) {
        for (size_t j = i; j < 3; ++j) {
            curDist = euclDist(vec[i][0], vec[i][1], vec[j][0], vec[j][1]);
            if (curDist > maxDist) {
                maxDist = curDist;
                maxIndex = i;
                maxJndex = j;
            }
        }
    }
    size_t first = (3 - maxIndex - maxJndex) % 3;
    swap(vec[0], vec[first]);

    int xprod = (vec[1][0] - vec[0][0]) * (vec[2][1] - vec[0][1]) - (vec[2][0] - vec[0][0]) * (vec[1][1] - vec[0][1]);
    if (xprod < 0) swap(vec[1], vec[2]);
}

std::vector<rw::math::Vector2D<int> > marker1(Mat src, NoOfTargets No) {


    /// Extract hue information
    Mat hsvTemp, hsv[3];
    // Robwork uses RGB instead of BGR
    cvtColor(src, hsvTemp, CV_RGB2HSV); // convert image to HSV color space
    split(hsvTemp, hsv);    // split image into hue, saturation and value images
    Mat hue = hsv[0].clone();   // select hue channel

    /// Perform hue thresholding on each pixel
    int hueTol = .025*255,    // Set tolerance for hue thresholding
            red = 0,
            green = 60,
            blue = 120;
    Mat bin = hue.clone();
    for(int i = 0; i < src.rows; ++i) {
        for(int j = 0; j < src.cols; ++j) {

            uchar &pix = bin.at<uchar>(i,j);  // Extract hue value of pixel

            if(norm(pix-blue) <= hueTol) {    // Check whether pixel is blue
                pix = 255; // If blue, highlight
            } else {
                pix = 0;   // If not blue, remove
            }
        }
    }

    /// Find contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(bin, contours, hierarchy,
                 CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    /// Get the moments
    vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        mu[i] = moments(contours[i], false);
    }

    /// Find the 3 most circular contours in the binary image
    double maxRatio[3],
            curRatio;
    Point2f maxCenter[3];
    Point2f curCenter;
    for (size_t i = 0; i < 3; ++i) {
        maxRatio[i] = 0;
    }
    for (int i = 0; i < contours.size(); i++) {
        float radius;
        minEnclosingCircle(contours[i], curCenter, radius);
        curRatio = mu[i].m00 / radius;  // mu[i].m00 is the contour area
        for (size_t j = 0; j < 3; ++j) {    // compare to 3 highest values
            if ( curRatio > maxRatio[j] ) {
                // Add new maxima by shifting old elements accordingly
                rotate(maxRatio+j,maxRatio+3,maxRatio+4);
                maxRatio[j] = curRatio;
                // For each area/radius ratio, save corresponding center
                rotate(maxCenter+j, maxCenter+3, maxCenter+4);
                maxCenter[j] = curCenter;
                break;
            }
        }

    }



    vector<Vector2D<int> > tri;
    for (size_t i = 0; i < 3; ++i) {
        tri.push_back(rw::math::Vector2D<int>(maxCenter[i].x, maxCenter[i].y));
    }
    sortTriangle(tri);

    for(int i = 0; i < 3; ++i){
        maxCenter[i] = Point2f(tri[i][0], tri[i][1]);
    }

    vector<Scalar> color;
    color.push_back(Scalar(255, 0, 0));
    color.push_back(Scalar(0, 255, 0));
    color.push_back(Scalar(255, 255, 255));
    /// Draw marker positions onto original image and display
    for(int i = 0; i < 3; ++i){
        circle( src,
                maxCenter[i],
                3,
                color[i],
                5,
                8 );
    }
    //namedWindow("points", WINDOW_NORMAL);
    //resizeWindow("points", 640, 480);
    //imshow("points", src);

    int cols = src.cols;
    int rows = src.rows;
    for (size_t i = 0; i < 3; ++i) {
        tri[i][0]= tri[i][0] - cols/2;
        tri[i][1]= tri[i][1] - rows/2;
    }

    // Return specified noOfTargs coordinates
    std::vector<Vector2D<int>>::const_iterator first = tri.begin();
    std::vector<Vector2D<int>>::const_iterator last = tri.begin() + No;

    return std::vector<Vector2D<int>>(first, last);
}


