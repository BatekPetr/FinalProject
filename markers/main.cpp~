/*
 *  ex3_template.cpp
 *  Exercise 3 - The Frequency domain and filtering
 *
 *  Created by Stefan-Daniel Suvei on 19/09/16.
 *  Copyright 2016 SDU Robotics. All rights reserved.
 *
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <sstream>
#include "marker1.h"
#include "rw/math.hpp"

using namespace cv;
using namespace std;

int main() {

    // Test marker1 on each image in the hard sequence
    for (int i = 1; i <= 52; ++i ) {
        ostringstream num;
        string filen = "/media/petr/WD_HDD/SDU/RoVi1/FinalProject/marker_color_hard/marker_color_hard_";
        if(i<10) num << 0;
        num << i;
        string filename = filen + num.str() + ".png";
        cv::Mat img = imread(filename);
        NoOfTargets No = single;
        vector<rw::math::Vector2D<int> > centers = marker1(img, No);
/*
        Point2f* centers = marker1(filename);
        rw::math::Vector2D<int> centers2d[3];
        for (size_t i = 0; i < 3; ++i){
            centers2d[i] = rw::math::Vector2D<int>(centers[i].x, centers[i].y);
            cout << centers2d[i][0] <<" "<< centers2d[i][1] << endl;
        }
        //*centers2d = new rw::math::Vector2D<>(0, 0);
        //cout << centers[0];
*/
        for (int i = 0; i < 3; ++i) {
            //cout << "Center " << i << ": x = " << centers[i][0] << ", y = " << centers[i][1] << endl;
        }

        //delete centers;
        waitKey(0);
    }

/*
    marker2a("../src/marker2b.png");
    waitKey();
    // Test marker2a on each image in the hard sequence
    for (int i = 1; i <= 52; ++i ) {
        ostringstream num;
        string filen = "../src/marker_thickline_hard/marker_thickline_hard_";
        if(i<10) num << 0;
        num << i;
        string filename = filen + num.str() + ".png";

        marker2a(filename);

        waitKey(0);
    }
*/

   return 0;
}
