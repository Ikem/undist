extern "C"
{
	int cvCalib(void);
}

//============================================================================
// Name        : OpenCV_Calibration.cpp
// Author      : Michael Gasser
// Version     :
// Copyright   : Your copyright notice
// Description : Computes the undistortion parameters
// 			     and undistorts a picture in C++, Ansi-style,
//				 includes the OpenCV - library
//============================================================================

#include "template.h"
// Calling convention: -
//

#include "cv.h"
#include <stdio.h>
#include <stdlib.h>
//#include <ncurses.h>
#include <iostream>

#if defined(OSC_HOST)
#define IMAGE_DIRECTORY "/home/mike/undist/"
#endif

#if defined(OSC_TARGET)
#define IMAGE_DIRECTORY "/home/httpd/"
#endif


using namespace std;

//Will be used with the Calibration Application
int n_boards = 0; 				//Will be set by input list
const int board_dt = 20; 		//Wait 20 frames per chessboard view
int board_w = 6;
int board_h = 9;
//CvCapture* capture = 0;


int cvCalib(void) {

	n_boards = 1;				//  Number of board views

	IplImage* image = cvCreateImage(cvSize(752,480),IPL_DEPTH_8U,1);	//  Initialize imgage 752,480

    if( image == NULL ) {
       return -1;
    }

    struct OSC_PICTURE pic;

    char* srcImage = IMAGE_DIRECTORY "left12.bmp";

    pic.height = image->height;
    pic.width = image->width;
    pic.data = image->imageData;

    // Loads source image from file
    OscBmpRead (&pic, srcImage);
// 1-----------------------------------------------------------------------------------------------------------

    int board_n  = board_w * board_h;				// Number of the corners on the board: board_n = 6 x 9 = 54
    CvSize board_sz = cvSize( board_w, board_h );

    //ALLOCATE STORAGE
    CvMat* image_points      = cvCreateMat(n_boards*board_n,2,CV_32FC1);
    CvMat* object_points     = cvCreateMat(n_boards*board_n,3,CV_32FC1);
    CvMat* point_counts      = cvCreateMat(n_boards,1,CV_32SC1);
    CvMat* intrinsic_matrix  = cvCreateMat(3,3,CV_32FC1);
    CvMat* distortion_coeffs = cvCreateMat(5,1,CV_32FC1);

    CvPoint2D32f* corners = new CvPoint2D32f[ board_n ];
    int corner_count;
    int successes = 0;
    int step, frame = 0;

    IplImage *gray_image = cvCreateImage(cvGetSize(image),8,1);//subpixel

    // CAPTURE CORNER VIEWS LOOP UNTIL WE’VE GOT n_boards
    // SUCCESSFUL CAPTURES (ALL CORNERS ON THE BOARD ARE FOUND)
    //
    while(successes < n_boards) {
        //Skip every board_dt frames to allow user to move chessboard
      	if(frame++ % board_dt == 0) {
           //Find chessboard corners:
           int found = cvFindChessboardCorners(
                    image, board_sz, corners, &corner_count,
                    CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
           );
           //Get Subpixel accuracy on those corners
           //cvCvtColor(image, gray_image, CV_BGR2GRAY);
           cvFindCornerSubPix(gray_image, corners, corner_count,
                      cvSize(11,11),cvSize(-1,-1), cvTermCriteria(
                      CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));
           //Draw it
           cvDrawChessboardCorners(image, board_sz, corners,
                      corner_count, found);
           //cvShowImage( "Calibration", image );
           //cvSaveImage("Calibration.jpg", image);
           pic.data = image->imageData;

           OscBmpWrite(&pic, IMAGE_DIRECTORY "calibrated.bmp");

           // If we got a good board, add it to our data
           if( corner_count == board_n ) {
              step = successes*board_n;
              for( int i=step, j=0; j<board_n; ++i,++j ) {
                 CV_MAT_ELEM(*image_points, float,i,0) = corners[j].x;
                 CV_MAT_ELEM(*image_points, float,i,1) = corners[j].y;
                 CV_MAT_ELEM(*object_points,float,i,0) = j/board_w;
                 CV_MAT_ELEM(*object_points,float,i,1) = j%board_w;
                 CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
              }
              CV_MAT_ELEM(*point_counts, int,successes,0) = board_n;
              successes++;
           }
        } //end skip board_dt between chessboard capture
    } //END COLLECTION WHILE LOOP.

    // 2-----------------------------------------------
    //ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
    CvMat* object_points2  = cvCreateMat(successes*board_n,3,CV_32FC1);
    CvMat* image_points2   = cvCreateMat(successes*board_n,2,CV_32FC1);
    CvMat* point_counts2   = cvCreateMat(successes,1,CV_32SC1);
    //TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
    //Below, we write out the details in the next two loops. We could
    //instead have written:
    //image_points->rows = object_points->rows
    //successes*board_n; point_counts->rows = successes;
    //
    for(int i = 0; i<successes*board_n; ++i) {
    	CV_MAT_ELEM( *image_points2, float, i, 0) =
        	CV_MAT_ELEM( *image_points, float, i, 0);
        CV_MAT_ELEM( *image_points2, float,i,1) =
            CV_MAT_ELEM( *image_points, float, i, 1);
        CV_MAT_ELEM(*object_points2, float, i, 0) =
        	CV_MAT_ELEM( *object_points, float, i, 0) ;
        CV_MAT_ELEM( *object_points2, float, i, 1) =
            CV_MAT_ELEM( *object_points, float, i, 1) ;
        CV_MAT_ELEM( *object_points2, float, i, 2) =
            CV_MAT_ELEM( *object_points, float, i, 2) ;
    }
	for(int i=0; i<successes; ++i){ //These are all the same number
		CV_MAT_ELEM( *point_counts2, int, i, 0) =
            CV_MAT_ELEM( *point_counts, int, i, 0);
    }
    cvReleaseMat(&object_points);
    cvReleaseMat(&image_points);
    cvReleaseMat(&point_counts);
    // At this point we have all of the chessboard corners we need.
    // Initialize the intrinsic matrix such that the two focal
    // lengths have a ratio of 1.0
    //
    CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
    CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;
    //CALIBRATE THE CAMERA!
    cvCalibrateCamera2(
    	object_points2, image_points2,
        point_counts2,  cvGetSize( image ),
        intrinsic_matrix, distortion_coeffs,
        NULL, NULL,0  //CV_CALIB_FIX_ASPECT_RATIO
        );
    // SAVE THE INTRINSICS AND DISTORTIONS
    cvSave("Intrinsics.xml",intrinsic_matrix);
    cvSave("Distortion.xml",distortion_coeffs);

    // 3----------------------------------------------
    // Undistortion ------------------------------------------------------------

    // EXAMPLE OF LOADING THESE MATRICES BACK IN:
    CvMat *intrinsic = (CvMat*)cvLoad("Intrinsics.xml");
    CvMat *distortion = (CvMat*)cvLoad("Distortion.xml");
    // Build the undistort map that we will use for all
    // subsequent frames.
    //
    IplImage* mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
    IplImage* mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );

    cvInitUndistortMap(
    	intrinsic,
        distortion,
        mapx,
        mapy
        );
    // Just run the camera to the screen, now showing the raw and
    // the undistorted image.
    //
    // ----------------------------------------------------------
    if(image) {
    	IplImage *t = cvCloneImage(image);
        cvRemap( t, image, mapx, mapy );     // Undistort image
        cvReleaseImage(&t);
        //cvSaveImage("Undistort.jpg", image);
        OscBmpWrite(&pic, IMAGE_DIRECTORY "undistorted.bmp");
        //image = cvQueryFrame( capture );
        }
    // ----------------------------------------------------------

    //image = cvLoadImage( input_filename, 1 );
    // Loads source image from file
    OscBmpRead (&pic, srcImage);

    IplImage *a = cvCloneImage(image);

    // Rectify our image
    //
    cvRemap( a, image, mapx, mapy );

    // GET THE CHESSBOARD ON THE PLANE
    //
    int found = cvFindChessboardCorners(
    	image,
        board_sz,
        corners,
        &corner_count,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
        );

#if defined(OSC_HOST)
    if(!found){
       	printf("Couldn't aquire chessboard on %s, "
       			"only found %d of %d corners\n",
       			"left12.jpg",corner_count,board_n
       			);
       	return -1;
    }
#endif

    // Get Subpixel accuracy on those corners:
    cvFindCornerSubPix(
    	gray_image,
    	corners,
    	corner_count,
    	cvSize(11,11),
    	cvSize(-1,-1),
        cvTermCriteria( CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 )
        );



    // -----------------------------------------------
    // release the image
    cvReleaseImage( &image );

    return 0;
}
