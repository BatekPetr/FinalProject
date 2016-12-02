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
#include "marker2a.h"
#include "harris.h"
#include "rw/rw.hpp"

using namespace cv;
using namespace std;

int main() {

    // Test marker1 on each image in the hard sequence
    for (int i = 1; i <= 52; ++i ) {
        ostringstream num;
        string filen = "../src/marker_color_hard/marker_color_hard_";
        if(i<10) num << 0;
        num << i;
        string filename = filen + num.str() + ".png";

        Point2f* centers = marker1(filename);
        //cout << centers[0];

        delete centers;
        for (int i = 1; i <= 3; ++i) {
            cout << "Center " << i << ": x = " << (int)centers[i].x << ", y = " << (int)centers[i].y << endl;
        }

        //waitKey(0);
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
