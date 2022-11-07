#include "drawMatches.h"

const int draw_shift_bits = 4;
const int draw_multiplier = 1 << draw_shift_bits;

static inline void _drawKeypoint( Mat& img, const cv::Point2f& p, const Scalar& color, int flags )
{
    CV_Assert( !img.empty() );
    Point center( cvRound(p.x * draw_multiplier), cvRound(p.y * draw_multiplier) );

    if( flags & DrawMatchesFlags::DRAW_RICH_KEYPOINTS )
    {
        int radius = cvRound(2/2 * draw_multiplier); // KeyPoint::size is a diameter

        // draw the circles around keypoints with the keypoints size
        circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );

    }
    else
    {
        // draw center with R=3
        int radius = 3 * draw_multiplier;
        circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );
    }
}

void drawKeypoints( const Mat& image, const vector<cv::Point2f>& points, Mat& outImage,
                    const Scalar& _color, int flags )
{
    if( !(flags & DrawMatchesFlags::DRAW_OVER_OUTIMG) )
    {
        if( image.type() == CV_8UC3 )
        {
            image.copyTo( outImage );
        }
        else if( image.type() == CV_8UC1 )
        {
            cvtColor( image, outImage, CV_GRAY2BGR );
        }
        else
        {
            CV_Error( CV_StsBadArg, "Incorrect type of input image.\n" );
        }
    }

    RNG& rng=theRNG();
    bool isRandColor = _color == Scalar::all(-1);

    CV_Assert( !outImage.empty() );
    for( int i=0; i<points.size(); ++i )
    {
        Scalar color = isRandColor ? Scalar(rng(256), rng(256), rng(256)) : _color;
        _drawKeypoint( outImage, points[i], color, flags );
    }
}

static void _prepareImgAndDrawKeypoints( const Mat& img1, const vector<cv::Point2f>& keypoints1,
                                         const Mat& img2, const vector<cv::Point2f>& keypoints2,
                                         Mat& outImg, Mat& outImg1, Mat& outImg2,
                                         const Scalar& singlePointColor, int flags )
{
    Size size( img1.cols + img2.cols, MAX(img1.rows, img2.rows) );
    if( flags & DrawMatchesFlags::DRAW_OVER_OUTIMG )
    {
        if( size.width > outImg.cols || size.height > outImg.rows )
            CV_Error( CV_StsBadSize, "outImg has size less than need to draw img1 and img2 together" );
        outImg1 = outImg( Rect(0, 0, img1.cols, img1.rows) );
        outImg2 = outImg( Rect(img1.cols, 0, img2.cols, img2.rows) );
    }
    else
    {
        outImg.create( size, CV_MAKETYPE(img1.depth(), 3) );
        outImg = Scalar::all(0);
        outImg1 = outImg( Rect(0, 0, img1.cols, img1.rows) );
        outImg2 = outImg( Rect(img1.cols, 0, img2.cols, img2.rows) );

        if( img1.type() == CV_8U )
            cvtColor( img1, outImg1, CV_GRAY2BGR );
        else
            img1.copyTo( outImg1 );

        if( img2.type() == CV_8U )
            cvtColor( img2, outImg2, CV_GRAY2BGR );
        else
            img2.copyTo( outImg2 );
    }

    // draw keypoints
    if( !(flags & DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS) )
    {
        Mat _outImg1 = outImg( Rect(0, 0, img1.cols, img1.rows) );
        drawKeypoints( _outImg1, keypoints1, _outImg1, singlePointColor, flags + DrawMatchesFlags::DRAW_OVER_OUTIMG );

        Mat _outImg2 = outImg( Rect(img1.cols, 0, img2.cols, img2.rows) );
        drawKeypoints( _outImg2, keypoints2, _outImg2, singlePointColor, flags + DrawMatchesFlags::DRAW_OVER_OUTIMG );
    }
}

static inline void _drawMatch( Mat& outImg, Mat& outImg1, Mat& outImg2 ,
                          const cv::Point2f& kp1, const cv::Point2f& kp2, const Scalar& matchColor, int flags )
{
    RNG& rng = theRNG();
    bool isRandMatchColor = matchColor == Scalar::all(-1);
    Scalar color = isRandMatchColor ? Scalar( rng(256), rng(256), rng(256) ) : matchColor;

    _drawKeypoint( outImg1, kp1, color, flags );
    _drawKeypoint( outImg2, kp2, color, flags );

    Point2f dpt2 = Point2f( std::min(kp2.x+outImg1.cols, float(outImg.cols-1)), kp2.y );

    line( outImg,
          Point(cvRound(kp1.x*draw_multiplier), cvRound(kp1.y*draw_multiplier)),
          Point(cvRound(dpt2.x*draw_multiplier), cvRound(dpt2.y*draw_multiplier)),
          color, 1, CV_AA, draw_shift_bits );
}

void drawMatches( const Mat& img1, const vector<cv::Point2f>& keypoints1,
                  const Mat& img2, const vector<cv::Point2f>& keypoints2,
                   Mat& outImg,
                  const Scalar& matchColor, const Scalar& singlePointColor, int flags )
{

    Mat outImg1, outImg2;
    _prepareImgAndDrawKeypoints( img1, keypoints1, img2, keypoints2,
                                 outImg, outImg1, outImg2, singlePointColor, flags );

    // draw matches
    for( size_t m = 0; m < keypoints1.size(); m++ )
    {
            _drawMatch( outImg, outImg1, outImg2, keypoints1[m], keypoints2[m], matchColor, flags );
    }
}
