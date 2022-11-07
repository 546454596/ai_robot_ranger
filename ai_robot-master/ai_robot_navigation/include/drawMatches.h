#ifndef __DRAWMATCHES_H
#define __DRAWMATCHES_H

#include <opencv2/opencv.hpp>
using namespace cv;

void drawMatches( const Mat& img1, const vector<cv::Point2f>& keypoints1,
                  const Mat& img2, const vector<cv::Point2f>& keypoints2,
                   Mat& outImg,
                  const Scalar& matchColor, const Scalar& singlePointColor, int flags );

#endif
