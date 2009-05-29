extern "C"
{
	int mainDemo(void);
}

//============================================================================
// Name        : OpenCV_Calibration.cpp				First Test !!!
// Author      :
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
//int n_boards = 0; 			//Will be set by input list
//const int board_dt = 20; 		//Wait 20 frames per chessboard view
//int board_w = 6;
//int board_h = 9;
//CvCapture* capture = 0;


int mainDemo(void) {

    IplImage* img = cvCreateImage(cvSize(752,480),IPL_DEPTH_8U,1);
    //IplImage* dst; 													// Test
    //IplImage* sobelImg;												// Test

    if( img == NULL ) {
       return -1;
    }

    struct OSC_PICTURE pic;
    int height,width,step,channels;
    uint8_t *data;
    int i,j,k;

#if defined(OSC_HOST)
    char* srcImage = "/home/mike/undist/test.bmp";
#endif

#if defined(OSC_TARGET)
    char* srcImage = "/home/httpd/test.bmp";
#endif


    pic.height = img->height;
    pic.width = img->width;
    pic.data = img->imageData;

    // Loads source image from file
    //img=cvLoadImage(srcImage, 1) ;
    OscBmpRead (&pic, srcImage);


//-------------------------------------------------------------------------------------------------------------

    // get the image data
    height    = img->height;
    width     = img->width;
    //step      = img->widthStep;
    //step = 100;					// Noch unklar was hier der gute Wert ist
    //channels  = img->nChannels;
    channels = 1;
    data      = (uint8_t *)img->imageData;

#if defined(OSC_HOST)
    printf("Processing a %dx%d image with %d channels\n",height,width,channels);

    OscBmpWrite(&pic, "/home/mike/undist/sourceImage.bmp~");
    rename("/home/mike/undist/sourceImage.bmp~", "/home/mike/undist/sourceImage.bmp");
#endif

#if defined(OSC_TARGET)
    OscBmpWrite(&pic, "/home/httpd/sourceImage.bmp~");
    rename("/home/httpd/sourceImage.bmp~", "/home/httpd/sourceImage.bmp");
#endif
    // -----------------------------------------------

    // invert the image
    for(i=0;i<height;i++) for(j=0;j<width;j++) for(k=0;k<channels;k++)
    	data[(i*width + j) * channels + k] = 255 - data[(i*width + j) * channels + k];  // Version M.Schwarz
		//data[i*step+j*channels+k]=255-data[i*step+j*channels+k];

#if defined(OSC_HOST)
	OscBmpWrite(&pic, "/home/mike/undist/invertedImage.bmp~");
    rename("/home/mike/undist/invertedImage.bmp~", "/home/mike/undist/invertedImage.bmp");
#endif


#if defined(OSC_TARGET)
    OscBmpWrite(&pic, "/home/httpd/invertedImage.bmp~");
    rename("/home/httpd/invertedImage.bmp~", "/home/httpd/invertedImage.bmp");
#endif


    // Test ------------------------------------------------------------------------------------------------
    // Aply sobel
    /*
    dst = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1  );
    cvSobel( img, dst, 1, 1 );
    sobelImg = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1  );
    cvConvertScale( dst, sobelImg );
    pic.data = sobelImg->imageData;
    */// -----------------------------------------------------------------------------------------------------


    IplImage *out = cvCreateImage( cvGetSize(img), IPL_DEPTH_8U, 1 );
    cvSmooth( img, out, CV_GAUSSIAN, 3, 1 );
    pic.data = out->imageData;

#if defined(OSC_HOST)
    OscBmpWrite(&pic, "/home/mike/undist/smoothedImage.bmp~");
    rename("/home/mike/undist/smoothedImage.bmp~", "/home/mike/undist/smoothedImage.bmp");
#endif

#if defined(OSC_TARGET)
    OscBmpWrite(&pic, "/home/httpd/smoothedImage.bmp~");
    rename("/home/httpd/smoothedImage.bmp~", "/home/httpd/smoothedImage.bmp");
#endif

    // release the image
    cvReleaseImage( &img );
    cvReleaseImage( &out );
    //cvReleaseImage( &dst );														 // Test
    //cvReleaseImage( &sobelImg );													 // Test
    return 0;
}
