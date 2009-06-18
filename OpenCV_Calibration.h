/*
 * OpenCV_Calibration.h
 *
 *  Created on: 18.06.2009
 *      Author: Michael Gasser
 */

#ifndef OPENCV_CALIBRATION_H_
#define OPENCV_CALIBRATION_H_

#ifdef __cplusplus
extern "C"
{
#endif

// Hack to prevent types from openCV to get redefined by the oscar framework.
#include <stdint.h>
#include <stdbool.h>
#define OSCAR_INCLUDE_NIH_H_
typedef int OSC_ERR;
#define TRUE true
#define FALSE false
typedef int16_t int16;
typedef int32_t int32;
typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
//typedef uint64_t uint64;
// End Hack

/*--------------------------- Includes -----------------------------*/
#include "cv.h"
#include "cxcore.h"
#include "oscar.h"

/*--------------------------- Settings ------------------------------*/
#if defined(OSC_HOST)
#define IMAGE_DIRECTORY "/home/mike/undist/"
#endif

#if defined(OSC_TARGET)
#define IMAGE_DIRECTORY "/home/httpd/"
#endif

/*------------------- Main data object and members ------------------*/
struct CV_CALIBRATION {
	int board_w;
	int board_h;
	int n_boards;
	CvSize board_sz;
	int corner_count;
	int corners_found;
	CvPoint2D32f* corners;
	CvMat* object_points2;
    CvMat* image_points2;
    CvMat* point_counts2;
    CvMat* intrinsic_matrix;
    CvMat* distortion_coeffs;
}calib;

struct CV_PERSPECTIVE {
    CvMat *H;
    bool perspTransform;
    int Z;
}persp;


/*-------------------------- Functions --------------------------------*/
void createUndistFile(const char* directory, IplImage* mapx, IplImage* mapy );

IplImage* readImage(const char* srcImage);

OSC_ERR writeImage(const char * srcImage, IplImage * image);

struct CV_CALIBRATION cmCalibrateCamera(int n_boards, int board_w, int board_h, IplImage* image );

IplImage* cmDrawChessboardCorners(IplImage* image, struct CV_CALIBRATION calib);

struct CV_CALIBRATION saveModel (struct CV_CALIBRATION calib);

struct CV_PERSPECTIVE saveConfig (struct CV_PERSPECTIVE persp);

struct CV_CALIBRATION loadModel ();

struct CV_PERSPECTIVE loadConfig ();

IplImage* cmUndistort(struct CV_CALIBRATION calib, IplImage* image);

struct CV_PERSPECTIVE cmCalculatePerspectiveTransform(struct CV_CALIBRATION calib, int Z, bool perspTransform);

IplImage* cmPerspectiveTransform(struct CV_PERSPECTIVE persp, IplImage* image);

void printModule();











int cvCalib(void);


#ifdef __cplusplus
}
#endif





#endif /* OPENCV_CALIBRATION_H_ */
