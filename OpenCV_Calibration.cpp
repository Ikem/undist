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

#include "includesOpenCV/cv.h"
#include "includesOpenCV/highgui.h"
#include "includesOpenCV/cvcam.h"
#include <stdio.h>
#include <stdlib.h>
//#include <ncurses.h>
#include <iostream>

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

#if defined(OSC_HOST)
    char* srcImage = "/home/mike/undist/left12.bmp";	// Path to the board image for host
#endif

#if defined(OSC_TARGET)
    char* srcImage = "/home/httpd/left12.bmp";	// // Path to the board image for target
#endif


    pic.height = image->height;
    pic.width = image->width;
    pic.data = image->imageData;

    // Loads source image from file
    OscBmpRead (&pic, srcImage);
//-------------------------------------------------------------------------------------------------------------

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

           #if defined(OSC_HOST)
			   OscBmpWrite(&pic, "/home/mike/undist/calibrated.bmp~");
			   rename("/home/mike/undist/calibrated.bmp~", "/home/mike/undist/calibrated.bmp");
		   #endif

		   #if defined(OSC_TARGET)
			   OscBmpWrite(&pic, "/home/httpd/calibrated.bmp~");
			   rename("/home/httpd/calibrated.bmp~", "/home/httpd/calibrated.bmp");
		   #endif

           // If we got a good board, add it to our data
/*           if( corner_count == board_n ) {
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
           }*/
			   successes++;
        } //end skip board_dt between chessboard capture
    } //END COLLECTION WHILE LOOP.

    // -----------------------------------------------



    // release the image
    cvReleaseImage( &image );

    return 0;
}
