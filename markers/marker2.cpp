#include "marker2.h"

using namespace cv;
using namespace std;

void marker2(Mat src) {

    /// Find edges
    Mat bin;
    Canny(src, bin, 50, 250);

    /// Detect lines using HoughLinesP
    vector<Vec4i> lines;
    HoughLinesP(bin, lines, 1, CV_PI/180, 76, 200, 150 );
    Mat drawing = Mat::zeros(bin.size(), CV_8UC3);
    for( size_t i = 0; i < lines.size(); i++ ) {
      Vec4i l = lines[i];
      line( drawing, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,255), 1, CV_AA);
    }
    Mat lin;
    cvtColor(drawing, lin, CV_BGR2GRAY);

    /// Combine(AND) line and edge images
    Mat out;
    dilate(lin, lin, Mat());
    dilate(bin, bin, Mat());
    bitwise_and(bin, lin, out);

    /// Find contours
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(out, contours, hierarchy,
            CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

    /// Get the moments
    vector<Moments> mu(contours.size());
    for (int i = 0; i < contours.size(); i++) {
            mu[i] = moments(contours[i], false);
    }

    /// Find the biggest remaining rectangle
    vector<RotatedRect> minRect( contours.size() );
    RotatedRect biggest;
    double maxArea = 0, area, diff;
    Size2f siz;
    for( int i = 0; i < contours.size(); i++ ) {
        minRect[i] = minAreaRect( Mat(contours[i]) );
        siz = minRect[i].size;
        area = siz.height * siz.width;
        diff = abs( area - mu[i].m00 ) / area;
        if (area > maxArea && diff < .25) {
            biggest = minRect[i];
            maxArea = area;
        }
    }

    Point2f rect_points[4];
    biggest.points(rect_points);
    for( int j = 0; j < 4; j++ ) {
       line( src, rect_points[j], rect_points[(j+1)%4], Scalar(0,255, 0), 5, 8 );
    }

    /// Show in a window
    imshow( "Contours", src );

}

