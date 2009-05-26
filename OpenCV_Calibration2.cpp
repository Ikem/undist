extern "C"
{
	int mainDemo(void);
}

//============================================================================
// Name        : OpenCV_Calibration.cpp
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
// Hit ESC to quit
//
#include "includesOpenCV/cv.h"
#include "includesOpenCV/highgui.h"
#include "includesOpenCV/cvcam.h"
#include <stdio.h>
#include <stdlib.h>
//#include <ncurses.h>
#include <iostream>
using namespace std;

//int n_boards = 0; 			//Will be set by input list
//const int board_dt = 20; 	//Wait 20 frames per chessboard view
//int board_w = 6;
//int board_h = 9;
//CvCapture* capture = 0;


int mainDemo(void) {

    IplImage* img;
    int height,width,step,channels;
    uchar *data;
    int i,j,k;

#if defined(OSC_HOST)
    char* srcImage = "/home/mike/undist/TestImage.bmp";
#endif

#if defined(OSC_TARGET)
    char* srcImage = "/home/httpd/TestImage.bmp";
#endif

    // Loads source image from file
    img=cvLoadImage(srcImage, 1) ;
    if( img == 0 ) {
    	#if defined(OSC_HOST)
    	cout << "Image cannot be loaded!" << endl ;
    	#endif
       return -1;
    }

#if defined(OSC_HOST)
    // Show image properties
    cout << "Image height: " << img->height << " pixels" << endl ;
    cout << "Image width : " << img->width << " pixels" << endl ;
#endif
//--------------------------------------------------------------

     // get the image data
     height    = img->height;
     width     = img->width;
     step      = img->widthStep;
     channels  = img->nChannels;
     data      = (uchar *)img->imageData;

#if defined(OSC_HOST)
     printf("Processing a %dx%d image with %d channels\n",height,width,channels);


     // create a window
     cvNamedWindow("Inverted Image", CV_WINDOW_AUTOSIZE);
     cvMoveWindow("Inverted Image", 100, 100);

     // create a test window --------------------------
     cvNamedWindow("Source Image", CV_WINDOW_AUTOSIZE);
     cvMoveWindow("Source Image", 300, 300);
     cvShowImage("Source Image", img );

     cvSaveImage("/home/mike/undist/sourceImage.bmp", img);
#endif

#if defined(OSC_TARGET)
     cvSaveImage("/home/httpd/sourceImage.bmp", img);
#endif
     // -----------------------------------------------

     // invert the image
     for(i=0;i<height;i++) for(j=0;j<width;j++) for(k=0;k<channels;k++)
       data[i*step+j*channels+k]=255-data[i*step+j*channels+k];

#if defined(OSC_TARGET)
     cvSaveImage("/home/httpd/invertedImage.bmp", img);
#endif

#if defined(OSC_HOST)
     // show the image
     cvShowImage("Inverted Image", img );
     cvSaveImage("/home/mike/undist/invertedImage.bmp", img);
     // wait for a key
     cvWaitKey(0);
#endif

     // release the image
     cvReleaseImage(&img );
     return 0;
}
