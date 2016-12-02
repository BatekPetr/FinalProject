#include "marker1.h"

using namespace cv;
using namespace std;

Point2f* marker1(string filename) {

    /// Read image from file
    Mat src = imread(filename, 1);

    /// Extract hue information
    Mat hsvTemp, hsv[3];
    cvtColor(src, hsvTemp, CV_BGR2HSV); // convert image to HSV color space
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
    Point2f* maxCenter;
    maxCenter = new Point2f[3];
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

    /// Draw marker positions onto original image and display
    for(int i = 0; i < 3; ++i){
        circle( src,
                 maxCenter[i],
                 3,
                 Scalar( 0, 255, 0 ),
                 5,
                 8 );
    }
    //namedWindow("points", WINDOW_NORMAL);
    //resizeWindow("points", 640, 480);
    //imshow("points", src);

    return maxCenter;
}
