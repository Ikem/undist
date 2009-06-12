extern "C"
{
	int cvCalib(void);
	#include <unistd.h>

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
#include "cxcore.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>

#if defined(OSC_HOST)
#define IMAGE_DIRECTORY "/home/mike/undist/"
#endif

#if defined(OSC_TARGET)
#define IMAGE_DIRECTORY "/home/httpd/"
#endif


using namespace std;

struct CV_CALIBRATION {
	int board_w;
	int board_h;
	int corners_found;
	CvPoint2D32f* corners;
	CvMat* object_points2;
    CvMat* image_points2;
    CvMat* point_counts2;
};


IplImage* readImage(const char* srcImage)
{
	OSC_ERR err = SUCCESS;
	IplImage* image = cvCreateImage(cvSize(OSC_CAM_MAX_IMAGE_WIDTH, OSC_CAM_MAX_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);	// Initialize image 752,480

    if( image == NULL )
    	goto fail;

	struct OSC_PICTURE pic;
    pic.height = image->height;
    pic.width = image->width;
    pic.data = image->imageData;
    pic.type = OSC_PICTURE_GREYSCALE;

    // Loads source image from file
    err = OscBmpRead (&pic, srcImage);
    if (err != SUCCESS)
		goto fail;

    return image;

fail:
	cvReleaseImage(&image);
	return NULL;
}


OSC_ERR writeImage(const char * srcImage, IplImage * image)
{
	OSC_ERR err = SUCCESS;

	if( image == NULL ) {
		err = EINVALID_PARAMETER;
		goto fail;
	}

	struct OSC_PICTURE pic;
	pic.height = image->height;
	pic.width = image->width;
	pic.data = image->imageData;
	pic.type = OSC_PICTURE_GREYSCALE;

	// Loads source image from file
	err = OscBmpWrite (&pic, srcImage);
	if (err != SUCCESS)
		goto fail;

	return SUCCESS;

fail:
	cvReleaseImage(&image);
	return err;
}


struct CV_CALIBRATION cmCalibrateCamera(int n_boards, int board_w, int board_h, IplImage* image )
{
	struct CV_CALIBRATION calib;
	const int board_dt = 20;				// Wait 20 frames per chessboard view
	calib.board_w = board_w;
	calib.board_h = board_h;

	int board_n  = board_w * board_h;				// Number of the corners on the board: board_n = 6 x 9 = 54
	CvSize board_sz = cvSize( board_w, board_h );

	// ALLOCATE STORAGE
	CvMat* image_points      = cvCreateMat(n_boards*board_n,2,CV_32FC1);
	CvMat* object_points     = cvCreateMat(n_boards*board_n,3,CV_32FC1);
	CvMat* point_counts      = cvCreateMat(n_boards,1,CV_32SC1);


	calib.corners = new CvPoint2D32f[ board_n ];
	int corner_count;
	int successes = 0;
	int step, frame = 0;


//	IplImage *gray_image = cvCreateImage(cvGetSize(image),IPL_DEPTH_8U,1);	//subpixel

	// CAPTURE CORNER VIEWS LOOP UNTIL WE’VE GOT n_boards
	// SUCCESSFUL CAPTURES (ALL CORNERS ON THE BOARD ARE FOUND)
	//
	while(successes < n_boards) {
		// Skip every board_dt frames to allow user to move chessboard
		if(frame++ % board_dt == 0) {
		   // Find chessboard corners:
		   calib.corners_found = cvFindChessboardCorners(
				   image, board_sz, calib.corners, &corner_count,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
		   );
		   // Get Subpixel accuracy on those corners
		   // cvCvtColor(image, gray_image, CV_BGR2GRAY);
           cvFindCornerSubPix(image, calib.corners, corner_count,
					  cvSize(11,11),cvSize(-1,-1), cvTermCriteria(
					  CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

		   // Draw it
		   cvDrawChessboardCorners(image, board_sz, calib.corners,
					  corner_count, calib.corners_found);

		   // If we got a good board, add it to our data
		   if( corner_count == board_n ) {
			  step = successes*board_n;
			  for( int i=step, j=0; j<board_n; ++i,++j ) {
				 CV_MAT_ELEM(*image_points, float,i,0) = calib.corners[j].x;
				 CV_MAT_ELEM(*image_points, float,i,1) = calib.corners[j].y;
				 CV_MAT_ELEM(*object_points,float,i,0) = j/board_w;
				 CV_MAT_ELEM(*object_points,float,i,1) = j%board_w;
				 CV_MAT_ELEM(*object_points,float,i,2) = 0.0f;
			  }
			  CV_MAT_ELEM(*point_counts, int,successes,0) = board_n;
			  successes++;
		   }
		} //end skip board_dt between chessboard capture
	} //END COLLECTION WHILE LOOP.

    // ALLOCATE MATRICES ACCORDING TO HOW MANY CHESSBOARDS FOUND
    CvMat* object_points2  = cvCreateMat(successes*board_n,3,CV_32FC1);
    CvMat* image_points2   = cvCreateMat(successes*board_n,2,CV_32FC1);
    CvMat* point_counts2   = cvCreateMat(successes,1,CV_32SC1);

    // TRANSFER THE POINTS INTO THE CORRECT SIZE MATRICES
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

    calib.image_points2 = image_points2;
    calib.object_points2 = object_points2;
    calib.point_counts2 = point_counts2;
//  cvReleaseImage( &gray_image );

	// At this point we have all of the chessboard corners we need.
	// Initialize the intrinsic matrix such that the two focal
	// lengths have a ratio of 1.0
	//
	CvMat* intrinsic_matrix  = cvCreateMat(3,3,CV_32FC1);
	CvMat* distortion_coeffs = cvCreateMat(5,1,CV_32FC1);

	CV_MAT_ELEM( *intrinsic_matrix, float, 0, 0 ) = 1.0f;
	CV_MAT_ELEM( *intrinsic_matrix, float, 1, 1 ) = 1.0f;

	// CALIBRATE THE CAMERA!
	cvCalibrateCamera2(
		calib.object_points2, calib.image_points2,
		calib.point_counts2,  cvGetSize( image ),
		intrinsic_matrix, distortion_coeffs,
		NULL, NULL,0  //CV_CALIB_FIX_ASPECT_RATIO
		);

	// SAVE THE INTRINSICS AND DISTORTIONS
	cvSave(IMAGE_DIRECTORY "Intrinsics.xml",intrinsic_matrix);
	cvSave(IMAGE_DIRECTORY "Distortion.xml",distortion_coeffs);
	return calib;
}


IplImage* cmUndistort(IplImage* image)
{
    // Undistortion ------------------------------------------------------------

    // EXAMPLE OF LOADING THESE MATRICES BACK IN:
    CvMat *intrinsic = (CvMat*)cvLoad(IMAGE_DIRECTORY "Intrinsics.xml");
    CvMat *distortion = (CvMat*)cvLoad(IMAGE_DIRECTORY "Distortion.xml");
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

    if(image) {
    	IplImage *t = cvCloneImage(image);
        cvRemap( t, image, mapx, mapy );     // Undistort image
        cvReleaseImage(&t);
        //image = cvQueryFrame( capture );
        }
    return image;
}


IplImage* cmPerspectiveTransform(struct CV_CALIBRATION calib, IplImage* image)
{
    // Bird's Eye ------------------------------------------------------------
    // GET THE IMAGE AND OBJECT POINTS
    // We will choose chessboard object points as (r,c):
    // (0,0), (board_w-1,0), (0,board_h-1), (board_w-1, board_h-1).
    //

    CvPoint2D32f objPts[4], imgPts[4];
    objPts[0].x = 0;			objPts[0].y = 0;
    objPts[1].x = calib.board_w-1;	objPts[1].y = 0;
    objPts[2].x = 0;			objPts[2].y = calib.board_h-1;
    objPts[3].x = calib.board_w-1;	objPts[3].y = calib.board_h-1;
    imgPts[0] = calib.corners[0];
    imgPts[1] = calib.corners[calib.board_w-1];
    imgPts[2] = calib.corners[(calib.board_h-1)*calib.board_w];
    imgPts[3] = calib.corners[(calib.board_h-1)*calib.board_w + calib.board_w-1];

    // DRAW THE POINTS in order: B,G,R,YELLOW
    cvCircle( image, cvPointFrom32f(imgPts[0]), 9, CV_RGB(0,0,255),		3);
    cvCircle( image, cvPointFrom32f(imgPts[1]), 9, CV_RGB(0,255,0),		3);
    cvCircle( image, cvPointFrom32f(imgPts[2]), 9, CV_RGB(255,0,0),		3);
    cvCircle( image, cvPointFrom32f(imgPts[3]), 9, CV_RGB(255,255,0),	3);

    // FIND THE HOMOGRAPHY
    CvMat *H = cvCreateMat( 3, 3, CV_32F);

    cvGetPerspectiveTransform( objPts, imgPts, H);

    // LET THE USER ADJUST THE Z HEIGHT OF THE VIEW
    float Z = 25 + (100 * 0.5);
    IplImage* birds_image = cvCloneImage(image);

    // Set the height
   	CV_MAT_ELEM(*H, float, 2, 2) = 45;

  	// COMPUTE THE FRONTAL PARALLEL OR BIRD'S-EYE VIEW:
    // USING HOMOGRAPHY TO REMAP THE VIEW
   	cvWarpPerspective(  image,
   	        			birds_image,
   	        			H,
   	        			CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
   	        			);

    cvSave(IMAGE_DIRECTORY "H.xml", H); // We can reuse H for the same camera mounting
    return birds_image;
    cvReleaseImage(&birds_image);
}


IplImage* perspectiveTransfrom(IplImage* image)
{
    IplImage* birds_image = cvCreateImage(cvSize(752,480),IPL_DEPTH_8U,1);
    CvMat *H = cvCreateMat( 3, 3, CV_32F);
    H = (CvMat*)cvLoad(IMAGE_DIRECTORY "H.xml");
    cvWarpPerspective(  image,
       	        		birds_image,
       	        		H,
       	        		CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
       	        		);

	return birds_image;
    cvReleaseImage( &birds_image );
}


int cvCalib(void) {

	// Will be used with the Calibration Application
	int n_boards = 1; 				// Number of chessboard views
	int board_w = 6;				// Number of points horizontal
	int board_h = 9;				// Number of points vertical

	struct CV_CALIBRATION calib;
	const char* srcFile = IMAGE_DIRECTORY "left12.bmp";
	const char* calibFile = IMAGE_DIRECTORY "calibrated.bmp";
	const char* undistFile = IMAGE_DIRECTORY "undistorted.bmp";
	const char* perspTransFile = IMAGE_DIRECTORY "birdseye.bmp";
	//const char* test_perspTransFile = IMAGE_DIRECTORY "test_birdseye.bmp";


	IplImage* image = readImage(srcFile);	// Read image via OSCAR and save for OpenCV
	calib = cmCalibrateCamera(n_boards, board_w, board_h, image);

	if(calib.corners_found){
		// Save calibrated image
		writeImage(calibFile, image);
	} else {
		printf("wrong input parameters x, y");
	}

	IplImage* undistimage = readImage(srcFile);
	undistimage = cmUndistort(undistimage);
    // Save undistorted image
    writeImage(undistFile, undistimage);


    undistimage = cmPerspectiveTransform(calib, undistimage);
   	// Save birds_eye image
   	writeImage(perspTransFile, undistimage);
	//undistimage = perspectiveTransfrom(undistimage);
   	//writeImage(test_perspTransFile, undistimage);

	// release the images
    cvReleaseImage( &image );
    cvReleaseImage( &undistimage );

    //cvGetModuleInfo();
    return 0;
}
