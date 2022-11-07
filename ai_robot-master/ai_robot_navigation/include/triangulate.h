#ifndef TRANGULATE_H
#define TRANGULATE_H

#include "opencv2/opencv.hpp"

using namespace cv;

void cvTriangulatePts(CvMat* projMatr1, CvMat* projMatr2, CvMat* projPoints1, CvMat* projPoints2, CvMat* points4D);

void triangulatePts( InputArray _projMatr1, InputArray _projMatr2,InputArray _projPoints1, InputArray _projPoints2,OutputArray _points4D );

#endif
