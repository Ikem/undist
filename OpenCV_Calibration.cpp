
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
#include "OpenCV_Calibration.h"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <unistd.h>

#define BENCHMARK

#ifdef BENCHMARK
 #define SW_START(startCyc) \
        startCyc = OscSupCycGet();
#else /* BENCHMARK */
 #define SW_START(startCyc) \
        {};
#endif /* BENCHMARK */

#ifdef BENCHMARK
    #define SW_STOP(label, startCyc) \
    OscLog(CRITICAL, label ": %dus\n", \
    OscSupCycToMicroSecs(OscSupCycGet() - startCyc));
#else /* BENCHMARK */
 #define SW_STOP(label, startCyc) \
       {};
#endif /* BENCHMARK */

#if defined(OSC_HOST)
#define IMAGE_DIRECTORY "/home/mike/undist/"
#endif

#if defined(OSC_TARGET)
#define IMAGE_DIRECTORY "/home/httpd/"
#endif


using namespace std;



void createUndistFile(const char* directory, IplImage* mapx, IplImage* mapy )
{
	// Generating undist.ud file-------------------------------------------------------------------------------
	#if defined(OSC_HOST)
	FILE* file = fopen(directory, "w"); // file open	"undist.ud"
	#endif

	#if defined(OSC_TARGET)
	FILE* file = fopen(directory, "w"); // file open	"/mnt/app/undist.ud"
	#endif


	uint32 bounding_rectangle[4];
	bounding_rectangle[0] = 0;
	bounding_rectangle[1] = 0;
	bounding_rectangle[2] = 752;
	bounding_rectangle[3] = 480;

	uint32 matrix_size[2];
	matrix_size[0] = mapx->width;
	matrix_size[1] = mapx->height;

	float resolution[2];
	resolution[0] = 0.5161;
	resolution[1] = 0.5161;

	uint16 undist[(mapx->width*mapx->height)*2];

	for (int row=0, dst=0; row<(mapx->height); row += 1) {
		float * ptrx = (float*) (mapx->imageData + row*mapx->widthStep);
		float * ptry = (float*) (mapy->imageData + row*mapy->widthStep);
		for(int pix=0; pix<mapx->width; pix++, dst += 2){
			float valx = ptrx[pix];
			float valy = ptry[pix];

			if (0 < valx && valx <= 752 && 0 < valy && valy <= 480) {
				undist[dst] = (uint16) (valx * (65536 / 1024));
				undist[dst + 1] = (uint16) (valy * (65536 / 1024));
			} else {
				undist[dst] = (uint16) -1;
				undist[dst + 1] = (uint16) -1;
			}
			//printf("%d/%d: %f -> %d\n", row, pix, ptr[pix], (uint16)(ptr[pix]*65536/1024));
		}
	}

	fwrite(bounding_rectangle, sizeof(uint32), 4, file);
	fwrite(matrix_size, sizeof(uint32), 2, file);
	fwrite(resolution, sizeof(uint32), 2, file);
	fwrite(undist, sizeof(uint16), (mapx->width*mapx->height)*2, file);
	fclose(file); // close file
}


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
	//struct CV_CALIBRATION calib;
	const int board_dt = 20;				// Wait 20 frames per chessboard view
	calib.board_w = board_w;
	calib.board_h = board_h;

	int board_n  = board_w * board_h;				// Number of the corners on the board: board_n = 6 x 9 = 54
	calib.board_sz = cvSize( board_w, board_h );

	// ALLOCATE STORAGE
	CvMat* image_points      = cvCreateMat(n_boards*board_n,2,CV_32FC1);
	CvMat* object_points     = cvCreateMat(n_boards*board_n,3,CV_32FC1);
	CvMat* point_counts      = cvCreateMat(n_boards,1,CV_32SC1);


	calib.corners = new CvPoint2D32f[ board_n ];
	calib.corner_count;
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
				   image, calib.board_sz, calib.corners, &calib.corner_count,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS
		   );
		   // Get Subpixel accuracy on those corners
		   // cvCvtColor(image, gray_image, CV_BGR2GRAY);
           cvFindCornerSubPix(image, calib.corners, calib.corner_count,
					  cvSize(11,11),cvSize(-1,-1), cvTermCriteria(
					  CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));


		   // If we got a good board, add it to our data
		   if( calib.corner_count == board_n ) {
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
	calib.intrinsic_matrix  = cvCreateMat(3,3,CV_32FC1);
	calib.distortion_coeffs = cvCreateMat(5,1,CV_32FC1);

	CV_MAT_ELEM( *calib.intrinsic_matrix, float, 0, 0 ) = 1.0f;
	CV_MAT_ELEM( *calib.intrinsic_matrix, float, 1, 1 ) = 1.0f;

	// CALIBRATE THE CAMERA!
	cvCalibrateCamera2(
		calib.object_points2, calib.image_points2,
		calib.point_counts2,  cvGetSize( image ),
		calib.intrinsic_matrix, calib.distortion_coeffs,
		NULL, NULL,0  //CV_CALIB_FIX_ASPECT_RATIO
		);

	return calib;
}

IplImage* cmDrawChessboardCorners(IplImage* image, struct CV_CALIBRATION calib)
{
	IplImage* cornerImage = cvCloneImage(image);
	// Draw it
	cvDrawChessboardCorners(cornerImage, calib.board_sz, calib.corners,
							calib.corner_count, calib.corners_found);
	return cornerImage;
}


struct CV_CALIBRATION saveModel (struct CV_CALIBRATION calib)
{
	if(calib.corners_found)
	{
	struct CV_CALIBRATION LVcalib;
	// SAVE THE INTRINSICS AND DISTORTIONS
	cvSave(IMAGE_DIRECTORY "Intrinsics.xml",calib.intrinsic_matrix);
	cvSave(IMAGE_DIRECTORY "Distortion.xml",calib.distortion_coeffs);
	LVcalib = calib;
	return LVcalib;
	}
	else
	{
		OscLog(INFO, "No Calibration Corners found\n");
	}

}

struct CV_PERSPECTIVE saveConfig (struct CV_PERSPECTIVE persp)
{
	if(persp.perspTransform)
	{
	struct CV_PERSPECTIVE LVpersp;
	// SAVE THE HOMOGRAPHIE MATRIX
	cvSave(IMAGE_DIRECTORY "H.xml", persp.H); // We can reuse H for the same camera mounting
	LVpersp = persp;
	return LVpersp;
	}
	else
	{
		OscLog(INFO, "Perspective Transformation not active\n");
	}
}

struct CV_CALIBRATION loadModel ()
{
	// EXAMPLE OF LOADING THESE MATRICES BACK IN:
	struct CV_CALIBRATION calib;
	calib.intrinsic_matrix = (CvMat*)cvLoad(IMAGE_DIRECTORY "Intrinsics.xml");
	calib.distortion_coeffs = (CvMat*)cvLoad(IMAGE_DIRECTORY "Distortion.xml");
	return calib;
}

struct CV_PERSPECTIVE loadConfig ()
{
	// EXAMPLE OF LOADING THESE MATRICES BACK IN:
	struct CV_PERSPECTIVE persp;
	persp.H = (CvMat*)cvLoad(IMAGE_DIRECTORY "H.xml");
	return persp;
}



IplImage* cmUndistort(struct CV_CALIBRATION calib, IplImage* image)
{
    // Undistortion ------------------------------------------------------------

    // Build the undistort map that we will use for all
    // subsequent frames.
    //
    IplImage* mapx = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );
    IplImage* mapy = cvCreateImage( cvGetSize(image), IPL_DEPTH_32F, 1 );

    cvInitUndistortMap(
    	calib.intrinsic_matrix,
        calib.distortion_coeffs,
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


struct CV_PERSPECTIVE cmCalculatePerspectiveTransform(struct CV_CALIBRATION calib, int Z, bool perspTransform)
{
    // Bird's Eye ------------------------------------------------------------
    // GET THE IMAGE AND OBJECT POINTS
    // We will choose chessboard object points as (r,c):
    // (0,0), (board_w-1,0), (0,board_h-1), (board_w-1, board_h-1).
    //

	struct CV_PERSPECTIVE persp;
	persp.Z = Z;
	persp.perspTransform = perspTransform;

    CvPoint2D32f objPts[4], imgPts[4];
    objPts[0].x = 0;			objPts[0].y = 0;
    objPts[1].x = calib.board_w-1;	objPts[1].y = 0;
    objPts[2].x = 0;			objPts[2].y = calib.board_h-1;
    objPts[3].x = calib.board_w-1;	objPts[3].y = calib.board_h-1;
    imgPts[0] = calib.corners[0];
    imgPts[1] = calib.corners[calib.board_w-1];
    imgPts[2] = calib.corners[(calib.board_h-1)*calib.board_w];
    imgPts[3] = calib.corners[(calib.board_h-1)*calib.board_w + calib.board_w-1];

/*  // DRAW THE POINTS in order: B,G,R,YELLOW
    cvCircle( image, cvPointFrom32f(imgPts[0]), 9, CV_RGB(0,0,255),		3);
    cvCircle( image, cvPointFrom32f(imgPts[1]), 9, CV_RGB(0,255,0),		3);
    cvCircle( image, cvPointFrom32f(imgPts[2]), 9, CV_RGB(255,0,0),		3);
    cvCircle( image, cvPointFrom32f(imgPts[3]), 9, CV_RGB(255,255,0),	3);
*/
    // FIND THE HOMOGRAPHY
    persp.H = cvCreateMat( 3, 3, CV_32F);

    if(persp.perspTransform)
	{
    cvGetPerspectiveTransform( objPts, imgPts, persp.H);

    // LET THE USER ADJUST THE Z HEIGHT OF THE VIEW
    //float Z = 25 + (100 * 0.5);

    // Set the height
   	CV_MAT_ELEM(*persp.H, float, 2, 2) = persp.Z;//45;
	}
    return persp;
}


IplImage* PerspectiveTransform(struct CV_PERSPECTIVE LVpersp, IplImage* image)
{
    IplImage* birds_image = cvCreateImage(cvSize(752,480),IPL_DEPTH_8U,1);

  	// COMPUTE THE FRONTAL PARALLEL OR BIRD'S-EYE VIEW:
    // USING HOMOGRAPHY TO REMAP THE VIEW
    cvWarpPerspective(  image,
       	        		birds_image,
       	        		LVpersp.H,
       	        		CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
       	        		);

	return birds_image;
    cvReleaseImage( &birds_image );
}


IplImage* cmPerspectiveTransform(struct CV_PERSPECTIVE persp, IplImage* image)
{
    //IplImage* birds_image = cvCloneImage(image);
    IplImage* birds_image = cvCreateImage(cvSize(752,480),IPL_DEPTH_8U,1);

  	// COMPUTE THE FRONTAL PARALLEL OR BIRD'S-EYE VIEW:
    // USING HOMOGRAPHY TO REMAP THE VIEW
    cvWarpPerspective(  image,
       	        		birds_image,
       	        		persp.H,
       	        		CV_INTER_LINEAR | CV_WARP_INVERSE_MAP | CV_WARP_FILL_OUTLIERS
       	        		);

	return birds_image;
    cvReleaseImage( &birds_image );
}





void printModule()
{
	const char* libraries;
    const char* modules;
    cvGetModuleInfo(0, &libraries, &modules );
    printf("Libraries: %s\nModules: %s\n", libraries, modules);
}

int cvCalib(void) {

//	#define MODULE
	#ifdef MODULE
		printModule();
	#endif

	#ifdef BENCHMARK
		uint32 startCyc;
	#endif

	// Will be used with the Calibration Application
	int n_boards = 1; 				// Number of chessboard views
	int board_w = 6;				// Number of points horizontal
	int board_h = 9;				// Number of points vertical

	struct CV_CALIBRATION calib;
	struct CV_PERSPECTIVE persp, LVpersp;

	const char* srcFile = IMAGE_DIRECTORY "left12.bmp";
	const char* calibFile = IMAGE_DIRECTORY "calibrated.bmp";
	const char* undistFile = IMAGE_DIRECTORY "undistorted.bmp";
	const char* perspTransFile = IMAGE_DIRECTORY "birdseye.bmp";
	const char* test_perspTransFile = IMAGE_DIRECTORY "test_birdseye.bmp";
	const char* allInOne = IMAGE_DIRECTORY "allInOne.bmp";

	// Calibrate and save image -------------------------------------------------------------------------------
	SW_START(startCyc);
	IplImage* image = readImage(srcFile);	// Read image via OSCAR and save for OpenCV
	calib = cmCalibrateCamera(n_boards, board_w, board_h, image);
	image = cmDrawChessboardCorners(image, calib);

	if(calib.corners_found){
		// Save calibrated image
		writeImage(calibFile, image);
	} else {
		printf("wrong input parameters x, y");
	}
	SW_STOP("CalibrateCamera", startCyc);

	// Undistort and save image -------------------------------------------------------------------------------
	SW_START(startCyc);
	IplImage* undistimage = readImage(srcFile);
	undistimage = cmUndistort(calib, undistimage);
    // Save undistorted image
    writeImage(undistFile, undistimage);
    SW_STOP("Undistort", startCyc);


    // All in one, undistortion and perspective transformation
    IplImage* gugu = readImage(srcFile);
    gugu = cmUndistort(calib, gugu);
    // Save undistorted image
    writeImage(allInOne, gugu);

    // TestImage for perspectiveTransform, function call in live-view mode
    IplImage* gaga = cvCloneImage(undistimage);

/*    SW_START(startCyc);
    persp = cmCalculatePerspectiveTransform(calib, Z);
    undistimage = cmPerspectiveTransform(persp, undistimage);
   	// Save birds_eye image
   	writeImage(perspTransFile, undistimage);
    SW_STOP("PerspectiveTransform", startCyc);
*/
    int Z = 45;
	bool perspTransform = TRUE;//FALSE;
	persp = cmCalculatePerspectiveTransform(calib, Z, perspTransform);
    if(persp.perspTransform){
    	gugu = cmPerspectiveTransform(persp, gugu);
   	    // Save birds_eye image
   	    writeImage(allInOne, gugu);
    }



    // perspectiveTransform, function call in live-view mode
    LVpersp = saveConfig (persp);
    if(LVpersp.perspTransform)
    {
    gaga = PerspectiveTransform(LVpersp, gaga);
   	writeImage(test_perspTransFile, gaga);
    }


	// release the images
    cvReleaseImage( &image );
    cvReleaseImage( &undistimage );
    cvReleaseImage( &gaga );
    cvReleaseImage( &gugu );

    return 0;
}
