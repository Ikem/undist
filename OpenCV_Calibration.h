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

#include "template.h"
#include "template_ipc.h"


#define USE_OPTIMIZED_UNDISTORT


//-------- Definitions used by UndistortChunk and LoadUndistortionInfo ------------


/*! @brief Number of bits to shift up (to boost) the output of the
 * UndistAndSobel functions.
 *
 * Normally, shift factors are calculated from
 * worst-case assumptions, which is over-cautious for real-world data.
 * To yield a greater dynamic range, the output can be boosted a little.
 * Boosting will saturate, so even if boosting a bit too far, the
 * consequences are minor. */
#define INTERP_BOOST 1


/*! @brief Generic two-dimensional vector. */
struct VEC_2D
{
	uint16 x;
	uint16 y;
};


/*********************************************************************//*!
 * @brief Return the result of x/y round up to the next full integer.
 *//*********************************************************************/
#define ROUND_UP_DIVISION(x, y) (((x) + ((y) - 1))/(y))


/*! @brief The maximum size (width x lengh) of the undistortion
 * interpolation matrix. */
#define MAX_UNDISTORT_INTERP_SIZE 400000

/*! @brief The directions in which the insertion control can operate. */
enum EnInsertionControlDirs
{
    DIR_FROM_LEFT = 0,
    DIR_FROM_RIGHT,
    NUM_OF_INS_CTRL_DIRECTIONS
};


/*! @brief Holds all the information regarding the interpolation matrix
 * used for undistortion. */
struct UNDIST_INTERP_INFO
{
    /*! @brief The rectangle describing where the pixels needed for undistortion
     *  are in the distorted image.*/
    struct IMG_RECT boundRect;
    /*! @brief The rectangle for which undistortion information is available
     *  (size of the undistortion matrix).*/
    struct IMG_RECT undistRect;

    /*! @brief The width of the undistortion matrix (and thus the maximum
     * width of the undistorted image. */
    uint16          undistWidth;
    /*! @brief The height of the undistortion matrix (and thus the maximum
     * height of the undistorted image. */
    uint16          undistHeight;
    /*! @brief The X resolution of the undistortion matrix (mm/px).*/
    float           resX;
    /*! @brief The Y resolution of the undistortion matrix (mm/px).*/
    float           resY;

    /*! @brief The actual undistortion data as read from file.
     *
     * The maximum size is limited which allows us to statically allocate
     * its memory. Matrix consists of packed X/Y values each representing
     * a fixed-point unsigned fractional value with INTERP_PRECISION bits
     * after the comma. */
    struct VEC_2D u2x16UndistInterpMatrix[MAX_UNDISTORT_INTERP_SIZE];
};


/*------------------------- Patch area -----------------------------*/
/*! @brief The width (and height) of an image patch.
 *
 * A patch is the small area that is analyzed during control mode. */
#define PATCH_SIDE 128

/*! @brief The maximum image width of the sensor. */
#define MAX_IMAGE_WIDTH 752
/*! @brief The maximum image height of the sensor. */
#define MAX_IMAGE_HEIGHT 480

//end Patch area ----------------------------------------------------


/*! @brief Number of valid insertion control samples required in the
 * learning phase to derive the edge positions. */
#define INSERTION_CONTROL_OBSERVER_LENGTH	5


// Definitions from cmdline.h ---------------------------------------
/*! @brief Contains all runtime options settable over command line
 * arguments. */
struct SHT_OPTIONS
{
    /*! @brief The mode the application runs in. */
    char        mode[64];
    /*! @brief The path to the reference image file. */
    char        strRefImage[1024];
    /*! @brief The path to the file containing the undistortion
     * information. */
    char        strUnDistFile[1024];
    /*! @brief The X coordinate of the upper left corner of the
     * predetermined patch square. */
    uint32      SOI_Xmin;
    /*! @brief The Y coordinate of the upper left corner of the
     * predetermined patch square. */
    uint32      SOI_Ymin;
    /*! @brief The width of the predetermined patch square. */
    uint32      SOI_width;
    /*! @brief The height of the predetermined patch square. */
    uint32      SOI_height;
    /*! @brief The rotation correlation threshold. */
    float       Thr_RotCorr;
    /*! @brief The translation correlation threshold. */
    float       Thr_TransCorr;
    /*! @brief The maximum allowed rotation of a sheet. */
    uint32      Thr_Rot;
    /*! @brief The maximum allowed translation of a sheet in the X
     * direction. */
    uint32      Thr_Tx;
    /*! @brief The maximum allowed translation of a sheet in the Y
     * direction. */
    uint32      Thr_Ty;
    /*! @brief The path to the file list used. */
    char        strFileList[1024];
    /*! @brief The X coordinate of the upper left corner of the
     * predetermined sheet rectangle. */
    uint32      AreaXmin;
    /*! @brief The Y coordinate of the upper left corner of the
     * predetermined sheet rectangle. */
    uint32      AreaYmin;
    /*! @brief The width of the predetermined sheet rectangle. */
    uint32      AreaWidth;
    /*! @brief The height of the predetermined sheet rectangle. */
    uint32      AreaHeight;
};
//end Definitions from cmdline.h ------------------------------------


// Definitions from mulituse_buffer.h -------------------------------

/*! @brief Pad a variable to be a multiple of the cache-line length.
 *
 * Only works within structures by invoking this macro directly
 * after the declaration of the member variable to be padded.
 *
 * @param currentLen The unpadded length of the structure to be
 * padded.
 * @param seq A sequence number used to name the pad. Don't use
 * a number twice in the same structure. */
#define CACHEALIGN_PAD(currentLen, seq) \
    uint8 __pad##seq[\
            (((ROUND_UP_DIVISION(currentLen, CACHE_LINE_LEN)\
            *CACHE_LINE_LEN) - currentLen) > 0) \
            ? (((ROUND_UP_DIVISION(currentLen, CACHE_LINE_LEN)\
            *CACHE_LINE_LEN) - currentLen)) \
            : CACHE_LINE_LEN\
            ]


/*! @brief Intermediate results for CalculateReferencePatch operation. */
struct CALCULATE_REFERENCE_PATCH
{
    /*! @brief The reference sobel filtered patch after Fourier
     * transformation.
     * (65x128) => 32.5 kB.*/
    complex_fract16 cf16SpecPatchRef[(PATCH_SIDE/2 + 1) * PATCH_SIDE];
    /*! @brief The polar transform of cf16FftPatchRef.
     * (128x64) => 16 kB */
    fract16         f16PolarPatchRef[PATCH_SIDE*PATCH_SIDE/2];
    /*! @brief The Fourier transform of f16PolarPatch.
     * (65x64) => 16.25 kB*/
    complex_fract16 cf16PolarSpecPatchRef[(PATCH_SIDE/2 + 1) * PATCH_SIDE/2];
};

/*---------------------- Initialization ------------------------------*/
/*! @brief Multi-use buffer description during initialization. */
struct MUB_INIT
{
    /*! @brief The reference image cropped to the bounding rectangle
     * used by the undistortion matrix. */
    uint8           u8CroppedReference[MAX_IMAGE_HEIGHT*MAX_IMAGE_WIDTH];

    /*! @brief The reference image after undistortion and sobel filtering
     * cropped to the patch area.
     * (128x128) => 32 kB.*/
    fract16         f16SobelPatchRef[PATCH_SIDE*PATCH_SIDE];

    /*! @brief Intermediate results for reference patch calculation */
    struct CALCULATE_REFERENCE_PATCH calRefPatch;

    /*! @brief The run-time options that are set over the command line.*/
    struct SHT_OPTIONS  rtOptions;
    CACHEALIGN_PAD(sizeof(struct SHT_OPTIONS), rtOptions);
};

/*---------------------- Insertion Control mode -------------------------*/
/*! @brief Holds parameters for the Insertion Control algoritm. */
struct INSERTION_CONTROL_PARAMS
{
    /*! @brief The threshold (percent) for the gradient of a sheet edge. */
    int32                       threshold;
    /*! @brief The direction in which the algorithm searches for edges. */
    enum EnInsertionControlDirs enDir;


    /*! @brief The margin (mm) from the image border towards
     * the inside of the sheet in which edges are searched. */
    int32                       marginInside;
    /*! @brief The blank area (mm) from the right undistortable
     * image border  */
    int32                       blankRight;
    /*! @brief The blank area (mm) from the left undistortable
     * image border  */
    int32                       blankLeft;
    /*! @brief The position in mm and undistortion coordinates of the
     * checking location if operated from left (from bottom left). */
    int32                       posLeft;
    /*! @brief The position in mm and undistortion coordinates of the
     * checking location if operated from right (from bottom left). */
    int32                       posRight;
    /*! @brief The vertical checking position in mm and undistortion
     * coordinates (from bottom left). */
    int32                       posVert;


    /*! @brief The margin (px) from the image border towards
     * the inside of the sheet in which edges are searched. */
    int32                       marginInside_pix;
    /*! @brief The blank area (mm) from the right undistortable
     * image border  */
    int32                       blankRight_pix;
    /*! @brief The blank area (mm) from the left undistortable
     * image border  */
    int32                       blankLeft_pix;
    /*! @brief The position in pixels and undistortion coordinates of the
     * checking location if operated form left (from top left). */
    int32                       posLeft_pix;
    /*! @brief The position in pixels and undistortion coordinates of the
     * checking location if operated form right (from top left). */
	int32                       posRight_pix;
    /*! @brief The vertical checking position in pixels and undistortion
     * coordinates (from top left).*/
    int32                       posVert_pix;
    /*! @brief The horizontal position at which the insertion control algorithm
     * starts checking when operating from the left side (pix). */
    int32						posLeftStart_pix;
    /*! @brief The horizontal position at which the insertion control algorithm
     * starts checking when operating from the right side (pix). */
    int32						posRightStart_pix;
};

/*! @brief Holds the results of the Insertion Control algoritm for
 * one particular search direction. */
struct INSERTION_CONTROL_RESULT
{
    /*! @brief Found horizontal edge position of the sheet [pixels] */
    int32						position_pix;
    /*! @brief Found horizontal edge position of the sheet [mm]. */
    int32						position;
    /*! @brief Correlation value at the location of the edge. */
    uint16						correlation;
    /*! @brief Measure for the confidence of the result. */
    float						confidence;
    /*! @brief How many mm of distance between the inside controlpoint
     * position and the detected sheet edge.*/
    int32						overlap;
    /*! @brief Overall insertion control success. */
    bool						passOverall;
};


/*---------------------- Idle/LearnEdge mode -------------------------*/
/*! @brief Parameters for edge learning in idle mode. */
struct LEARN_EDGE_PARAMS
{
    /*! @brief Copy of insertion control parameters for learning phase.
     * blankRight parameter is usually overwritten to guarantee
     * edge searching over the whole belt aera (limited by marginLeft
     * and marginRight). */
    struct INSERTION_CONTROL_PARAMS params;
    /*! @brief Results of the insertion control algorithm for each
     * search direction. */
    struct INSERTION_CONTROL_RESULT     result[NUM_OF_INS_CTRL_DIRECTIONS];

    int32 left_pix[INSERTION_CONTROL_OBSERVER_LENGTH];
    int32 right_pix[INSERTION_CONTROL_OBSERVER_LENGTH];
    uint16 writeIdx;
    uint16 fillstate;
    uint32 sampleTime;
    uint32 lastValidSampleTime;
};

/*! @brief Multi-use buffer description in the idle mode. */
struct MUB_IDLE_MODE
{
    /*! @brief The raw distorted image relevant for undistortion. */
    uint8           u8DistImg[MAX_IMAGE_WIDTH*MAX_IMAGE_HEIGHT];
    /*! @brief The image after undistortion. Maximum size is the size
     * of the undistortion matrix.
     * (MAX_UNDISTORT_INTERP_SIZE => 800 kB)
     * There exist two copies of this image, one for processing and the
     * other containing the last live image containing a sheet for the
     * web interface. The two pointers f16CurImage and f16LastValidImage
     * each point to one of them but are interchanged when a new valid
     * image is detected.*/
    fract16         f16Image[2][MAX_UNDISTORT_INTERP_SIZE];
    /*! @brief Pointer to the live image buffer currently used for
     * processing. */
    fract16         *f16CurImage;
    /*! @brief Pointer to the live image buffer containing the last
     * image displaying a valid sheet. */
    fract16         *f16LastValidImage;
    /*! @brief Parameters used for learning edges. */
    struct LEARN_EDGE_PARAMS            learnEdge;
};


/*---------------------- Learning mode -------------------------------*/
/*! @brief The default exposure time of the CMOS sensor in microseconds.
 * This is the starting value the brightness adaptation starts from. */
#define DEFAULT_EXPOSURE_TIME 500
/*! @brief The minimum exposure time that gets configured during the
 * automatic brightness correction. (us) */
#define MIN_EXPOSURE_TIME 80
/*! @brief The maximum exposure time that gets configured during the
 * automatic brightness correction. (us) */
#define MAX_EXPOSURE_TIME 1000

/*! @brief The step size for the first phase of the automatic brightness
 * correction (us).*/
#define EXPOSURE_STEP_COARSE 100
/*! @brief The step size for the second phase of the automatic brightness
 * correction (us).*/
#define EXPOSURE_STEP_FINE 34
/*! @brief The number of iterations to average over for automatic
 * brightness adaptation. */
#define BRIGHTNESS_ADAPT_AVERAGING_ITERATIONS 5
/*! @brief The maximum number of iterations for automatic brightness
 * correction. For the coarse adapation, the number of iterations
 * is actually less, but for memory reservations we need to assume
 * the biggest value. */
#define BRIGHTNESS_ADAPT_MAX_ITERATIONS (                    \
    (MAX((MAX_EXPOSURE_TIME - DEFAULT_EXPOSURE_TIME),        \
            (DEFAULT_EXPOSURE_TIME - MIN_EXPOSURE_TIME))     \
            / EXPOSURE_STEP_FINE)                            \
        + BRIGHTNESS_ADAPT_AVERAGING_ITERATIONS )

/*! @brief The number of iterations to find the optimal patch
 * region. */
#define FIND_PATCH_POSITION_ITERATIONS     10

/*! @brief The difference in position (pixels) of the patch area
 * candidates.*/
#define PATCH_POS_STEP 10

/*! @brief The maximum number of patch position candidates that can
 * occur. This is the estimated maximum number of horizontal pixels
 * in the sheet area divided by the step size of the X position.*/
#define MAX_PATCH_POS_CANDIDATES (1000/PATCH_POS_STEP)

/*! @brief The number of iterations to find the best reference patch
 * image. */
#define FIND_REF_PATCH_ITERATIONS  25

/*! @brief Lower percentile level for Learning stage4. */
#define CHOOSE_PATCH_PERCENTILE_LOW  25
/*! @brief Upper percentile level for Learning stage4. */
#define CHOOSE_PATCH_PERCENTILE_HIGH 75


/*! @brief Holds all data necessary for automatic brightness adaption. */
struct BRIGHT_ADAPT_DATA
{
    /*! @brief An array with past exposure times for averaging. */
    uint32          expTimes[BRIGHTNESS_ADAPT_MAX_ITERATIONS + 2];
    /*! @brief An array with the means over past exposure times. */
    uint32          meanExpTimes[BRIGHTNESS_ADAPT_MAX_ITERATIONS -
                                 BRIGHTNESS_ADAPT_AVERAGING_ITERATIONS];
};

/*! @brief Holds all information regarding the reference patch. */
struct REFERENCE_PATCH_INFO
{
    /*! @brief The undistorted and sobel filtered reference patch. */
    fract16 f16SobelPatchRef[PATCH_SIDE*PATCH_SIDE];

    /*! @brief The complex conjugate of the polar reference patch. */
    complex_fract16
        cf16PolarSpecPatchRef[(PATCH_SIDE/2 + 1) * PATCH_SIDE/2];

    /*! @brief The complex conjugate of the Fourier transformation
     * of the reference patch in cartesian coordinates. */
    complex_fract16
        cf16CartSpecPatchRef[(PATCH_SIDE/2 + 1)* PATCH_SIDE];

    /*! @brief The edge position (undistorted px) as detected by insertion control
     *  associated with the reference patch. This is used for
     *  online patch position adaptation. */
    int32          edgePos;
    CACHEALIGN_PAD(sizeof(int32), edgePos);
    /*! @brief The mean-corrected energy of f16PolarPatchRef */
    double polarSpecPatchRef_Energy;
    /*! @brief The total scale factor of the polar patch by the FFT in bits. */
    int16              polarSpecPatchRef_Scale;
    /*! @brief The mean-corrected nergy of f16CartPatchRef */
    double cartSpecPatchRef_Energy;
    /*! @brief The total scale factor of the cart patch by the FFT in bits. */
    int16              cartSpecPatchRef_Scale;
    CACHEALIGN_PAD(2*sizeof(double) + sizeof(int16), corrScale);
};


/*------------------------ Sheet Correlation ------------------------------
 * used in Control mode and Learning mode stage4 */
/*! @brief Multi-use buffer description for control mode results. */
struct CORRELATE_SHEET_RESULT
{
    /*! @brief Rotation correlation.
     * [0, 0x3fff] represents correlation value [0.0, 1.0] */
    fract16 rotCorr;
    /*! @brief Translation correlation.
     * [0, 0x3fff] represents correlation value [0.0, 1.0] */
    fract16 transCorr;
    /*! @brief Rotation angle of test image, CCW.
     * [0xfff, 0, 0x3fff] represents angle [-pi/2, 0, pi/2) */
    fract16 rotAngle;
    /*! @brief Translation of test image in X.
     * [-127, 127] represents offset in pixel */
    int16  transX;
    /*! @brief Translation of test image in Y.
     * [-127, 127] represents offset in pixel */
    int16  transY;

    bool passRotCorr; /*!< @brief rotCorr value within treshold */
    bool passTransCorr; /*!< @brief transCorr value within treshold */
    bool passRotAngle; /*!< @brief rotAngle value within treshold */
    bool passTransX; /*!< @brief transX value within treshold */
    bool passTransY; /*!< @brief transY value within treshold */

    bool passOverall; /*!< @brief Overall result of SheetRecognition */
    /*! @brief The number of mm the patch location has been compensated with
      the edge position information of insertion control. */
    int32 positionCompensation;
};

/*! @brief Multi-use buffer description for correlate sheet data.*/
struct CORRELATE_SHEET
{
    /*! @brief The Fourier transformed sobel patch.
     * (65x128) => 32.5 kB. */
    complex_fract16 cf16FftPatch[(PATCH_SIDE/2 + 1) * PATCH_SIDE];
    fract16 f16FftPatch_DbgAbs[(PATCH_SIDE/2 + 1) * PATCH_SIDE];
    /*! @brief The polar transformed frequency spectrum of the patch.
     * (128x64) => 16 kB. */
    fract16         f16PolarPatch[PATCH_SIDE * PATCH_SIDE/2];
    /*! @brief The Fourier transformation of the polar patch.
     * (65x64) => 16.25 kB. */
    complex_fract16 cf16PolarSpecPatch[(PATCH_SIDE/2 + 1) * PATCH_SIDE/2];
    fract16 f16PolarSpecPatch_DbgAbs[(PATCH_SIDE/2 + 1) * PATCH_SIDE/2];
    /*! @brief cf16PolarSpecPatch correlated with the reference patch.
     * (65x64 => 16.25 kB. */
    complex_fract16 cf16CorrSpecPolarPatch[(PATCH_SIDE/2 + 1) * PATCH_SIDE/2];
    fract16 f16CorrSpecPolarPatch_DbgAbs[(PATCH_SIDE/2 + 1) * PATCH_SIDE/2];
    /*! @brief Inverse FFT of the polar correlation.
     * (128x64 => 16 kB) */
    fract16         f16CorrPolarPatch[PATCH_SIDE * PATCH_SIDE/2];
    /*! @brief The matrix used to correct the rotation of the original
     * sobel filtered patch.
     * (128x128) => 64 kB. */
    fract2x16       f2x16RotateInterpMatrix[PATCH_SIDE * PATCH_SIDE];
    /*! @brief The original patch rotated back by the found rotation
     * angle.
     * (128x128) => 32 kB. */
    fract16         f16RotatedPatch[PATCH_SIDE * PATCH_SIDE];
    /*! @brief The Fourier transform of the back-rotated sobel patch.
     * (65x128) => 32.5 kB. */
    complex_fract16 cf16RotatedSpecPatch[(PATCH_SIDE/2 + 1) * PATCH_SIDE];
    fract16 f16RotatedSpecPatch_DbgAbs[(PATCH_SIDE/2 + 1) * PATCH_SIDE];
    /*! @brief The correlation of the fourier transformed patch in
     * cartesion coordinates with the reference.
     * (65x128) => 32.5 kB. */
    complex_fract16 cf16CorrSpecCartPatch[(PATCH_SIDE/2 + 1) * PATCH_SIDE];
    fract16 f16CorrSpecCartPatch_DbgAbs[(PATCH_SIDE/2 + 1) * PATCH_SIDE];
    /*! @brief The inverse fourier transformed cartesion correlation.
     * (128x128) => 32 kB. */
    fract16         f16CorrCartPatch[PATCH_SIDE * PATCH_SIDE];
};


/*! @brief Holds all data necessary for analyzing the reference patch. */
struct FIND_REF_PATCH_DATA
{
    /*! @brief The reference patch image info */
    struct REFERENCE_PATCH_INFO ref[FIND_REF_PATCH_ITERATIONS];

    /*! @brief Rotation correlation.
     * [0, 0x3fff] represents correlation value [0.0, 1.0] */
    fract16 rotCorr[FIND_REF_PATCH_ITERATIONS][FIND_REF_PATCH_ITERATIONS];
    /*! @brief Translation correlation.
     * [0, 0x3fff] represents correlation value [0.0, 1.0] */
    fract16 transCorr[FIND_REF_PATCH_ITERATIONS][FIND_REF_PATCH_ITERATIONS];

    /*! @brief Result collection of the sheet correlation */
    struct CORRELATE_SHEET_RESULT resultTmp;
    CACHEALIGN_PAD(sizeof(struct CORRELATE_SHEET_RESULT), resultTmp);
};

/*! @brief Holds all data necessary for choosing the reference patch. */
struct CHOOSE_REF_PATCH_DATA
{
    /*! @brief Rotation correlation. Same as in FIND_REF_PATCH_DATA
     * but without digonal elemnts */
    fract16 rotCorr[FIND_REF_PATCH_ITERATIONS][FIND_REF_PATCH_ITERATIONS-1];
    /*! @brief Translation correlation. Same as in FIND_REF_PATCH_DATA
     * but without digonal elemnts */
    fract16 transCorr[FIND_REF_PATCH_ITERATIONS][FIND_REF_PATCH_ITERATIONS-1];

    /*! @brief Rotation correlation median average */
    fract16 rotCorrAvg[FIND_REF_PATCH_ITERATIONS];
    /*! @brief Translation correlation median average */
    fract16 transCorrAvg[FIND_REF_PATCH_ITERATIONS];

    /*! @brief Rotation correlation percentile values:
     * [0]: low percent level, [1]: high percent level */
    fract16 rotCorrPercent[2];
    /*! @brief Translation correlation percentile values:
     * [0]: low percent level, [1]: high percent level */
    fract16 transCorrPercent[2];

    /*! @brief Normalized correlation distance */
    float dist[FIND_REF_PATCH_ITERATIONS-1];
};

/*! @brief Multi-use buffer description in the learning mode stage 1. */
struct MUB_LEARN_STG1
{
#ifdef TEST
    /*! @brief The image after undistortion. Maximum size is the size
     * of the undistortion matrix.
     * (MAX_UNDISTORT_INTERP_SIZE => 800 kB) */
    fract16         f16Image[MAX_UNDISTORT_INTERP_SIZE];
#endif /*TEST*/
    /*! @brief Data fields for automatic brightness adaptation. */
    struct BRIGHT_ADAPT_DATA brightAdapt;
    CACHEALIGN_PAD(sizeof(struct BRIGHT_ADAPT_DATA), brightAdapt);
};

/*! @brief Multi-use buffer description in the learning mode stage 2. */
struct MUB_LEARN_STG2
{
    /*! @brief The image after undistortion and sobel filtering.
     * Maximum size is the size of the undistortion matrix.
     * (MAX_UNDISTORT_INTERP_SIZE => 800 kB) */
    fract16         f16SobelImage[MAX_UNDISTORT_INTERP_SIZE];
    /*! @brief The quality statistics of all the patch position
     * candidates over the last frames.*/
    float           fPatchQualityStats[MAX_PATCH_POS_CANDIDATES][FIND_PATCH_POSITION_ITERATIONS];
    CACHEALIGN_PAD(MAX_PATCH_POS_CANDIDATES*FIND_PATCH_POSITION_ITERATIONS*sizeof(float), fPatchQualityStats);
    /*! @brief The measured edge distance over the frames used for patch candidate
     *  statistics. */
    fract16         edgePos[FIND_PATCH_POSITION_ITERATIONS];
    CACHEALIGN_PAD(FIND_PATCH_POSITION_ITERATIONS*sizeof(fract16), edgePos);
};

/*! @brief Multi-use buffer description in the learning mode stage 3. */
struct MUB_LEARN_STG3
{
    /*! @brief The pixels of the distorted image coming from the camera
      that are needed to create the undistorted image of the patch. */
    uint8           u8DistPatch[MAX_IMAGE_WIDTH*MAX_IMAGE_HEIGHT];
    CACHEALIGN_PAD(MAX_IMAGE_WIDTH*MAX_IMAGE_HEIGHT, u8DistPatch);
#ifdef TEST
    /*! @brief The patch after undistortion. Maximum size is the size
     * (130x130) => 34 kB */
    fract16         f16Patch[UNDIST_PATCH_SIDE*UNDIST_PATCH_SIDE];
    CACHEALIGN_PAD(UNDIST_PATCH_SIDE*UNDIST_PATCH_SIDE*sizeof(fract16),
            f16Patch);
#endif /*TEST*/
    /*! @brief Data fields for automatic brightness adaptation. */
    struct BRIGHT_ADAPT_DATA brightAdapt;
    CACHEALIGN_PAD(sizeof(struct BRIGHT_ADAPT_DATA), brightAdapt);
};

/*! @brief Multi-use buffer description in the learning mode stage 4. */
struct MUB_LEARN_STG4
{
    /*! @brief The reference image after undistortion and sobel filtering
     * cropped to the patch area.
     * (128x128) => 32 kB.*/
    fract16         f16SobelPatchRef[PATCH_SIDE*PATCH_SIDE];
    /*! @brief Intermediate results for reference patch calculation */
    struct CALCULATE_REFERENCE_PATCH calRefPatch;

    /*! @brief Data collection for sheet correlation */
    struct CORRELATE_SHEET corrSheet;
    /*! @brief Data fields for analyzing reference patch. */
    struct FIND_REF_PATCH_DATA findRefPatch;
    CACHEALIGN_PAD(sizeof(struct FIND_REF_PATCH_DATA), findRefPatch);
    /*! @brief Data fields for choosing reference patch. */
    struct CHOOSE_REF_PATCH_DATA chooseRefPatch;
    CACHEALIGN_PAD(sizeof(struct CHOOSE_REF_PATCH_DATA), chooseRefPatch);
 };

/*! @brief Multi-use buffer description in the learning mode. */
union MUB_LEARN_PATCH_MODE
{
    /*! @brief Multi-use buffer description of the first stage in the
     * learning mode. */
    struct MUB_LEARN_STG1       stg1;
    CACHEALIGN_PAD(sizeof(struct MUB_LEARN_STG1), stg1);

    /*! @brief Multi-use buffer description of the second stage in the
     * learning mode. */
    struct MUB_LEARN_STG2       stg2;
    CACHEALIGN_PAD(sizeof(struct MUB_LEARN_STG2), stg2);

    /*! @brief Multi-use buffer description of the third stage in the
     * learning mode. */
    struct MUB_LEARN_STG3       stg3;
    CACHEALIGN_PAD(sizeof(struct MUB_LEARN_STG3), stg3);

    /*! @brief Multi-use buffer description of the fourth stage in the
     * learning mode. */
    struct MUB_LEARN_STG4       stg4;
    CACHEALIGN_PAD(sizeof(struct MUB_LEARN_STG4), stg4);
};


/*------------------------ Control mode ------------------------------*/

/*! @brief Multi-use buffer description in the control mode. */
struct MUB_CONTROL_MODE
{
    /*! @brief The pixels of the distorted image coming from the camera
      that are needed to create the undistorted image of the patch. */
    uint8           u8DistPatch[MAX_IMAGE_WIDTH*MAX_IMAGE_HEIGHT];
    CACHEALIGN_PAD(MAX_IMAGE_WIDTH*MAX_IMAGE_HEIGHT, u8DistPatch);

    /*! @brief The patch after undistortion and sobel filtering.
     * This is a second instance so we can store away
     * the image of the last wrong image without having to copy
     * data. The pointer corrSheet.f16SobelPatch below points to the
     * instance currently processed and f16SobelPatchFail to the last
     * failed one.
     * (128x128) => 32 kB. */
    fract16         f16SobelPatch1[PATCH_SIDE * PATCH_SIDE];
    /*! @brief The patch after undistortion and sobel filtering.
     * The pointer f16SobelPatch below usually points to this instance.
     * (128x128) => 32 kB. */
    fract16         f16SobelPatch2[PATCH_SIDE * PATCH_SIDE];
    /*! @brief The reference image after undistortion and sobel filtering
     * cropped to the patch area.
     * (128x128) => 32 kB.*/
    fract16         *f16SobelPatch;
    /*! @brief Pointer to the sobel patch of the last wrong sheet. */
    fract16         *f16SobelPatchFail;

    /*! @brief Temporary data collection for correlate sheet */
    struct CORRELATE_SHEET corrSheet;
    /*! @brief Result collection of the sheet correlation */
    struct CORRELATE_SHEET_RESULT result;
    CACHEALIGN_PAD(sizeof(struct CORRELATE_SHEET_RESULT), result);
};


/*! @brief The multi-use buffer contains space for all temporary data
 * fields specific to a mode of the algorithm. */
union MULTI_USE_BUFFER
{
    /*! @brief Multi-use buffer description during initialization. */
    struct MUB_INIT            				init;
    /*! @brief Multi-use buffer description in the idle mode. */
    struct MUB_IDLE_MODE				idle;
    /*! @brief Multi-use buffer description in the learn patch mode. */
    union MUB_LEARN_PATCH_MODE     			learn;
    /*! @brief Multi-use buffer description in the control mode. */
    struct MUB_CONTROL_MODE    				ctrl;
};
//end Definitions form multiuse_buffer.h -----------------------------


/*! @brief The structure storing all important variables of the algorithm.
 *
 * This structure is aligned to the cache-line length, but the members
 * themselves must be aligned manually.
 * */
struct SHT
{
    /*! @brief The multi-use buffer for temporary data fields. Must
     * be cache-line aligned! */
    union MULTI_USE_BUFFER buf;
    /*! @brief The information of the reference patch generated by the
     * learning mode and used by the control mode. Must be cache-line
     * aligned.*/
//    struct REFERENCE_PATCH_INFO ref;
    /*! @brief Structure containing all the information for undistortion.
     * Must be cache-line aligned.*/
    struct UNDIST_INTERP_INFO undist;
    CACHEALIGN_PAD(sizeof(struct UNDIST_INTERP_INFO), undist);
    /*! @brief Lookup-table for the reciprocal of all positive integers
     * fitting into 16 bit.
     *
     * Only the 16 most significant non-sign bits are stored to keep the
     * table small and thus increase the probability of a value being
     * in cache. Must be cache-line aligned.
     * .
     * @see FillReciprocalTable
     * */
    fract16             f16ReciprocalTable[32768];
    /*! @brief The pre-calculated interpolation matrix for the polar
     * transformation. Must be cache-line aligned. */
    fract2x16           f2x16PolarInterpMatrix[PATCH_SIDE * PATCH_SIDE/2];
    /*! @brief The frame buffers for the frame capture device driver.
     * Must be cache-line aligned.*/
    uint8               u8FrameBuffers[NR_FRAME_BUFFERS][MAX_IMAGE_HEIGHT*MAX_IMAGE_WIDTH];

    /*! @brief Handle to the framework instance. */
    void                *hFramework;
    /*! @brief Handle to the DMA chain used. */
    void                *hDmaChain;

    /*! @brief Pointer to Block A of the L1 SRAM. */
    void                *pL1A;
    /*! @brief Pointer to Block B of the L1 SRAM. */
    void                *pL1B;
    /*! @brief Pointer to the Scratchpad L1 SRAM. */
    void                *pScratch;
    /*! @brief The rectangle defining the configured sheet after
     * undistortion in the coordinate system of the undistortion matrix.
     * A 1 pixel border needed for the sobel is included here.*/
    struct IMG_RECT     undistSheetRect;
    /*! @brief The rectangle defining the configured sheet after
     * undistortion and sobel filtering in the coordinate system of
     * the undistortion matrix. */
    struct IMG_RECT     sheetRect;
    /*! @brief The pixels that have to be read out of the camera to undistort
     *  the undistSheetRect. */
    struct IMG_RECT     undistSheetCamRect;
    /*! @brief The rectangle containing all potential patch candidates.
     *  This is a horizontally narrowed version of the undistSheetRect.*/
    struct IMG_RECT     undistPatchCandidatesRect;
    /*! @brief The rectangle describing the current patch after
     * undistortion in the coordinate system of the undistortion matrix.
     * This should be of size UNDIST_PATCH_SIDE x UNDIST_PATCH_SIDE
     * since a 1 pixel border needed for the sobel is included there.*/
    struct IMG_RECT     undistPatchRect;
    /*! @brief The rectangle describing the current patch after
     * undistortion and sobel filtering in the coordinate system of
     * the undistortion matrix. Should be of size PATCH_SIDE x PATCH_SIDE.
     */
    struct IMG_RECT     patchRect;
    /*! @brief The rectangle describing the pixels that have to be captured
     * from the camera for the (undist-) patch. */
    struct IMG_RECT     patchCamRect;
    /*! @brief The rectangle describing the pixels that have to be captured
     * from the camera for the belt (insertion control). */
    struct IMG_RECT     beltCamRect;
    /*! @brief The rectangle describing the pixels as captured by the
     * camera in the coordinate system of the camera image. */
    struct IMG_RECT     camRect;
    /*! @brief The twiddle coefficients used for the Fourier
     * transformation. */
    complex_fract16     *cf16TwidCoeffs;

    /*! @brief The mode the application is running in. This distinguishes
     * between test modes and normal mode. */
//    enum EnRunMode      enMode;

    /*! @brief Handle to the configuration file */
    CFG_FILE_CONTENT_HANDLE hConfig;

    /*! @brief Handle to the settings file */
    CFG_FILE_CONTENT_HANDLE hSettings;

    /*! @brief Exposure Delay for CPLD */
    int16               ExposureDelay;  /* valid range: 0..99 */

    /*! @brief Exposure time used for idle mode. */
    uint32              ExposureTime;

    /*! @brief Camera-Scene perspective */
    enum EnOscCamPerspective perspective;

    /*! @brief All data necessary for IPC. */
    struct IPC_DATA     ipc;

    /*! @brief The parameters used for correlation. */
//    struct CORRELATION_PARAMETERS corrParams;

    /*! @brief The collected statistics for the user interface. */
//    struct STATISTICS   stats;

    /*! @brief Determines whether some helpful debug information
     * is written to the local file system. */
    uint16              debugOutput;
    /*! @brief Structure holding file system information to avoid
     * overfilling the file system. */
//    struct statfs       statfs;
    /*! @brief Path for debug output from this camera. */
    char                strDebugOutputPath[256];

#if defined(TEST)
    /*! @brief Holds all data only needed for tests, that can be
     * removed from build at the end. */
    struct TEST_DATA    test;
#endif
    /*! @brief Applied output scaling in 2D FFT function:
     * image * 2^fftScale */
    int16              fftScale;
    /*! @brief Contains all variable used to calculate the correct scaling
      during correlation. */
//    struct CORRELATION_SCALE corrScale;
    /* @brief Tells the main loop to skip the current frame. Used during
     * state transitions. */
    bool                bSkipCurrentFrame;
    /*! @brief How long a new image can already be in the buffer maximally
     * and still gets used. */
    uint32              imageMaxAge;
    /*! @brief Number of frames to delay the output. Set over webinterface. */
    uint16 				outputDelayFrames;
    /*! @brief Number of fine clocks to delay the output. Set over webinterface. */
    uint16				outputDelayFine;
    /*! @brief GPIO configuration parameters */
//    struct GPIO_CONFIGURATION gpioConfig;
    /*! @brief Insertion control algorithm configuration parameters. */
//    struct INSERTION_CONTROL_DATA insertionControl;
    /*! @brief FIFO holding the output states being delayed. */
//    struct OUTPUT_VECTOR outputDelayFifo[MAX_OUTPUT_DELAY_FRAMES];
    /*! @brief Index into the outputDelayFifo array to where the next
     * output vector must be written. */
    uint16 			idxOutputDelayFifoWriteNext;
    /*! @brief Index into the outputDelayFifo array from where the next
     * output vector must be read. */
    uint16 			idxOutputDelayFifoReadNext;
    /*! @brief Meta information to debounce the TEACH button input */
//    struct DEBOUNCE techDebounce;
    /*! @brief Remember TeachLong event fired until the button is
     * released again. */
    bool teachLongFired;
    /*! @brief CPLD OutputLate flag occurence count */
    uint16 OutputLateCount;
    /*! @brief Default Language for WebInterface as ISO 639-2 code*/
    char language[4];
    /*! @brief The Linux system parameters. */
//    struct SYSTEM_PARAM system;
};


extern struct SHT *pSht;





/*---------------------- Undistortion options ------------------------*/
/*! @brief Default file name of the undistortion matrix. */
#define UNDISTORTION_MATRIX_FILENAME "Undist.ud"
/*! @brief The precision in bits after the comma of the values
 * from the interpolation matrix. */
#define INTERP_PRECISION 6
/*! @brief The bitmask to extract the digits after the comma of
 * the interpolation matrix. */
#define INTERP_PRECISION_MASK ((1<<INTERP_PRECISION) - 1)


#ifdef OSC_TARGET

/*********************************************************************//*!
 * @brief Extract the 16 most significant non-sign bits of a
 * 32 bit number.
 *
 * This counts the number of bits of the sign-extension and shifts until
 * the most significant non-sign bit plus one sign bit is placed in the
 * lower half-word.
 *
 * Target: Inline assembler macro.
 * Host: Inline C macro with helper function.
 *//*********************************************************************/
#define EXTRACT_16_MSBS(_in)       \
        asm(                        \
        "R1.l = SIGNBITS %0;\n"     \
        "%0 = LSHIFT %0 BY R1.l;\n" \
        "%0 >>= 16;\n"              \
        : "+d" (_in)               \
        :                           \
        : "R1")

/*********************************************************************//*!
 * @brief Shift up the argument complex number by boostBits bit
 * and saturate the result.
 *
 * Target: Inline assembler macro.
 * Host: Inline C macro with helper function.
 *//*********************************************************************/
#define BOOST_SATURATE_COMPLEX(_in, boostBits)	\
     asm(\
            "%0.h = %0.h << " STR(boostBits) " (S);\n"\
            "%0.l = %0.l << " STR(boostBits) " (S);\n"\
            : "+d" (_in)\
            : \
         )

/*********************************************************************//*!
 * @brief Shift up the real part of the argument complex number by
 * boostBits bit and saturate the result.
 *
 * Target: Inline assembler macro.
 * Host: Inline C macro with helper function.
 *//*********************************************************************/
#define BOOST_SATURATE_REAL(_in, boostBits) \
     asm(\
            "%0.l = %0.l << " STR(boostBits) " (S);\n"\
            : "+d" (_in)\
            : \
         )

/*********************************************************************//*!
 * @brief Saturates the low word of the input in that the maximum
 * positive or negative value is taken if the number is bigger than what
 * fits into 16 bits.
 *
 * Target: Inline assembler macro.
 * Host: Inline C macro.
 *//*********************************************************************/
#define SATURATE_LOW_WORD(_fr32)				\
    asm(                                                        \
            "%1 = %1 << 16 (S);\n"                              \
            "%1 >>>= 16;\n"                                     \
            : [out] "=d" (_fr32)                                \
            : [in] "d" (_fr32)                                  \
            )

/*********************************************************************//*!
 * @brief Counts the number of redundant sign bits.
 *
 * The result is written to the C variable __signbits_out, which must be
 * declared by the caller.
 *
 * Target: Inline assembler macro.
 * Host: Inline C macro with helper function.
 *//*********************************************************************/
#define SIGNBITS_16(_in)		\
    asm(                                \
    "R0 = %1;\n"                        \
    "R1.l = SIGNBITS R0.l;\n"           \
    "%0 = R1;\n"                        \
    : "=d" (__signbits_out)             \
    : "d" (_in)                         \
    : "R0", "R1")

#endif /*OSC_TARGET*/

#ifdef OSC_HOST

/*********************************************************************//*!
 * @brief Helper function as replacement for the SIGNBITS DSP Assembler
 * instruction.
 *
 * Determines how many redundant sign bits are present in a signed
 * integer.
 * Host only.
 *
 * @param __in Number to check for redundant sign bits.
 * @param __bits Number of bits in __in.
 * @return Number of redundant sign bits in __in.
 *//*********************************************************************/
int16 _SIGNBITS(int32 __in, uint16 __bits);

/* For description see target implementation. */
#define SIGNBITS_16(_in) \
    __signbits_out = _SIGNBITS(_in, 16)

/* For description see target implementation. */
#define EXTRACT_16_MSBS(_in) \
    _in >>= (16 - _SIGNBITS(_in, 32))

#define BOOST_SATURATE_REAL(_in, boostBits) \
    _in = _BOOST_SATURATE_COMPLEX(_in, boostBits)


/* For description see target implementation. */
#define SATURATE_LOW_WORD(_fr32) \
  _fr32 = ((_fr32 >= 0)				\
            ? ((_fr32 > 32767) ? 32767 : _fr32) \
	   : ((_fr32 < -32768) ? -32768 : _fr32))

int32 _BOOST_SATURATE_COMPLEX(int32 _in,
        uint16 boostBits);

#define BOOST_SATURATE_COMPLEX(_in, boostBits) \
    _in = _BOOST_SATURATE_COMPLEX(_in, boostBits)

#endif /* OSC_HOST */
//end definitions UndistortChunk and LoadUndistortionInfo -----------------

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

struct CV_UNDISTORT{
	IplImage* mapx;
	IplImage* mapy;
#ifdef USE_OPTIMIZED_UNDISTORT
	struct VEC_2D pInterp[OSC_CAM_MAX_IMAGE_WIDTH * OSC_CAM_MAX_IMAGE_HEIGHT];
#endif
}undist;

struct CV_PERSPECTIVE {
    CvMat *H;
    bool perspTransform;
    int Z;
    bool undistort;
}persp;

struct BUTTON{
	bool readFile;
}button;

/*-------------------------- Functions --------------------------------*/
void createUndistFile(const char* directory, IplImage* mapx, IplImage* mapy );

IplImage* readImage(const char* srcImage);

OSC_ERR writeImage(const char * srcImage, IplImage * image);

OSC_ERR captureImage(IplImage * image);

struct CV_CALIBRATION cmCalibrateCamera(int n_boards, int board_w, int board_h, IplImage* image );

IplImage* cmDrawChessboardCorners(IplImage* image, struct CV_CALIBRATION calib);

void saveModel (struct CV_CALIBRATION calib);

void saveConfig (struct CV_PERSPECTIVE persp);

struct CV_CALIBRATION loadModel ();

struct CV_PERSPECTIVE loadConfig ();

struct CV_UNDISTORT cmCalibrateUndistort(struct CV_CALIBRATION calib, IplImage * image);

IplImage* cmUndistort(struct CV_UNDISTORT undist, IplImage* image);

struct CV_PERSPECTIVE cmCalculatePerspectiveTransform(struct CV_CALIBRATION calib, int Z, bool perspTransform);

IplImage* cmPerspectiveTransform(struct CV_PERSPECTIVE persp, IplImage* image);

inline void UndistortChunk(uint8 *pOut,
			   const uint8 *u8DistImage,
			   const uint16 distWidth,
			   const struct VEC_2D *pInterp,
			   const uint16 interpWidth,
			   const struct IMG_RECT *pUndistRect,
			   const int16 xOff,
			   const int16 yOff,
			   const bool bShiftForSobel);

OSC_ERR LoadUndistortionInfo(const char strUndistFN[]);


void printModule();











int cvCalib(void);


#ifdef __cplusplus
}
#endif





#endif /* OPENCV_CALIBRATION_H_ */
