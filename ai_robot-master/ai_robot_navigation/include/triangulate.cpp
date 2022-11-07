
#include "triangulate.h"

// cvCorrectMatches function is Copyright (C) 2009, Jostein Austvik Jacobsen.
// cvTriangulatePoints function is derived from icvReconstructPointsFor3View, originally by Valery Mosyagin.

// HZ, R. Hartley and A. Zisserman, Multiple View Geometry in Computer Vision, Cambridge Univ. Press, 2003.



// This method is the same as icvReconstructPointsFor3View, with only a few numbers adjusted for two-view geometry
void cvTriangulatePts(CvMat* projMatr1, CvMat* projMatr2, CvMat* projPoints1, CvMat* projPoints2, CvMat* points4D)
{
    if( projMatr1 == 0 || projMatr2 == 0 ||
      projPoints1 == 0 || projPoints2 == 0 ||
      points4D == 0)
      CV_Error( CV_StsNullPtr, "Some of parameters is a NULL pointer" );

    if( !CV_IS_MAT(projMatr1) || !CV_IS_MAT(projMatr2) ||
      !CV_IS_MAT(projPoints1) || !CV_IS_MAT(projPoints2) ||
      !CV_IS_MAT(points4D) )
      CV_Error( CV_StsUnsupportedFormat, "Input parameters must be matrices" );

    int numPoints;
    numPoints = projPoints1->cols;

    if( numPoints < 1 )
        CV_Error( CV_StsOutOfRange, "Number of points must be more than zero" );

    if( projPoints2->cols != numPoints || points4D->cols != numPoints )
        CV_Error( CV_StsUnmatchedSizes, "Number of points must be the same" );

    if( projPoints1->rows != 2 || projPoints2->rows != 2)
        CV_Error( CV_StsUnmatchedSizes, "Number of proj points coordinates must be == 2" );

    if( points4D->rows != 4 )
        CV_Error( CV_StsUnmatchedSizes, "Number of world points coordinates must be == 4" );

    if( projMatr1->cols != 4 || projMatr1->rows != 3 ||
       projMatr2->cols != 4 || projMatr2->rows != 3)
        CV_Error( CV_StsUnmatchedSizes, "Size of projection matrices must be 3x4" );

    CvMat matrA;
    double matrA_dat[12];
    matrA = cvMat(4,3,CV_64F,matrA_dat);

    CvMat* projPoints[2];
    CvMat* projMatrs[2];

    projPoints[0] = projPoints1;
    projPoints[1] = projPoints2;

    projMatrs[0] = projMatr1;
    projMatrs[1] = projMatr2;

    /* Solve system for each point */
    int i,j;
    double matrb_dat[4],matrPos_dat[4];
    CvMat matrb=cvMat(4,1,CV_64F,matrb_dat);
    CvMat matrPos=cvMat(3,1,CV_64F,matrPos_dat);
    for( i = 0; i < numPoints; i++ )/* For each point */
    {
        /* Fill matrix for current point */
        for( j = 0; j < 2; j++ )/* For each view */
        {
            double x,y;
            x = cvmGet(projPoints[j],0,i);
            y = cvmGet(projPoints[j],1,i);
            for( int k = 0; k < 3; k++ )
            {
                cvmSet(&matrA, j*2+0, k, x * cvmGet(projMatrs[j],2,k) -     cvmGet(projMatrs[j],0,k) );
                cvmSet(&matrA, j*2+1, k, y * cvmGet(projMatrs[j],2,k) -     cvmGet(projMatrs[j],1,k) );
            }
            int k=3;
            cvmSet(&matrb, j*2, 0, x * cvmGet(projMatrs[j],2,k) -  cvmGet(projMatrs[j],0,k) );
            cvmSet(&matrb, j*2 + 1, 0, y * cvmGet(projMatrs[j],2,k) -  cvmGet(projMatrs[j],1,k) );
        }
        /* Solve system for current point */
        {
            cvSolve(&matrA,&matrb,&matrPos,CV_SVD);

            /* Copy computed point */
            cvmSet(points4D,0,i,cvmGet(&matrPos,0,0));/* X */
            cvmSet(points4D,1,i,cvmGet(&matrPos,1,0));/* Y */
            cvmSet(points4D,2,i,cvmGet(&matrPos,2,0));/* Z */
            cvmSet(points4D,3,i,-1);/* W */
        }
    }

#if 0
    double err = 0;
    /* Points was reconstructed. Try to reproject points */
    /* We can compute reprojection error if need */
    {
        int i;
        CvMat point3D;
        double point3D_dat[4];
        point3D = cvMat(4,1,CV_64F,point3D_dat);

        CvMat point2D;
        double point2D_dat[3];
        point2D = cvMat(3,1,CV_64F,point2D_dat);

        for( i = 0; i < numPoints; i++ )
        {
            double W = cvmGet(points4D,3,i);

            point3D_dat[0] = cvmGet(points4D,0,i)/W;
            point3D_dat[1] = cvmGet(points4D,1,i)/W;
            point3D_dat[2] = cvmGet(points4D,2,i)/W;
            point3D_dat[3] = 1;

            /* !!! Project this point for each camera */
            for( int currCamera = 0; currCamera < 2; currCamera++ )
            {
                cvMatMul(projMatrs[currCamera], &point3D, &point2D);

                float x,y;
                float xr,yr,wr;
                x = (float)cvmGet(projPoints[currCamera],0,i);
                y = (float)cvmGet(projPoints[currCamera],1,i);

                wr = (float)point2D_dat[2];
                xr = (float)(point2D_dat[0]/wr);
                yr = (float)(point2D_dat[1]/wr);

                float deltaX,deltaY;
                deltaX = (float)fabs(x-xr);
                deltaY = (float)fabs(y-yr);
                err += deltaX*deltaX + deltaY*deltaY;
            }
            std::cout<<"err:"<<err<<std::endl;
        }
    }
#endif
}


void triangulatePts( InputArray _projMatr1, InputArray _projMatr2,
                            InputArray _projPoints1, InputArray _projPoints2,
                            OutputArray _points4D )
{
    Mat matr1 = _projMatr1.getMat(), matr2 = _projMatr2.getMat();
    Mat points1 = _projPoints1.getMat(), points2 = _projPoints2.getMat();

    if((points1.rows == 1 || points1.cols == 1) && points1.channels() == 2)
        points1 = points1.reshape(1, static_cast<int>(points1.total())).t();

    if((points2.rows == 1 || points2.cols == 1) && points2.channels() == 2)
        points2 = points2.reshape(1, static_cast<int>(points2.total())).t();

    CvMat cvMatr1 = matr1, cvMatr2 = matr2;
    CvMat cvPoints1 = points1, cvPoints2 = points2;

    _points4D.create(4, points1.cols, points1.type());
    CvMat cvPoints4D = _points4D.getMat();

    cvTriangulatePts(&cvMatr1, &cvMatr2, &cvPoints1, &cvPoints2, &cvPoints4D);
}
