#include "marker2a.h"
#include "harris.h"

using namespace cv;
using namespace std;

Mat lsrc, dst, cdst;
int othresh = 50,
    lthresh = 125,
    mthresh = 100,
    max_lthresh = 400;

void marker2a(string filename) {
    lsrc = imread(filename, 0);
    imshow("src", lsrc);
    namedWindow("lines", CV_WINDOW_NORMAL);
    resizeWindow("lines", 640, 480);
    createTrackbar( "Threshold: ", "lines", &othresh, max_lthresh, houghCallback );
    createTrackbar( "MinLineLength: ", "lines", &lthresh, max_lthresh, houghCallback );
    createTrackbar( "MaxLineGap: ", "lines", &mthresh, max_lthresh, houghCallback );

    Canny(lsrc, dst, 50, 200, 3);
    cvtColor(dst, cdst, CV_GRAY2BGR);
    houghCallback(0,0);

}

void houghCallback(int, void*) {

     vector<Vec4i> lines;
     HoughLinesP(dst, lines, 1, CV_PI/180, othresh+1, lthresh, mthresh );
     Mat drawing = Mat::zeros(lsrc.size(), CV_8UC3);
     for( size_t i = 0; i < lines.size(); i++ )
     {
       Vec4i l = lines[i];
       line( drawing, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255,255,255), 1, CV_AA);
     }
     Mat lin;
     cvtColor(drawing, lin, CV_BGR2GRAY);
     //imshow("pre", lin);

     //lin = skeleton(lin);


    Mat element = getStructuringElement(MORPH_RECT, Size(3,3));

    imshow("lines", lin);
    //harris(lin);
}

Mat skeleton(Mat imgIn) {

    Mat skel(imgIn.size(), CV_8UC1, Scalar(0));
    Mat temp(imgIn.size(), CV_8UC1);
    Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));

    bool done;
    do {
        morphologyEx(imgIn, temp, MORPH_OPEN, element);
        bitwise_not(temp, temp);
        bitwise_and(imgIn, temp, temp);
        bitwise_or(skel, temp, skel);
        erode(imgIn, imgIn, element);

        double max;
        minMaxLoc(imgIn, 0, &max);
        done = (max == 0);
    } while (!done);
    return skel;
}
