/*	A collection of example applications for the LeanXcam platform.
	Copyright (C) 2008 Supercomputing Systems AG

	This library is free software; you can redistribute it and/or modify it
	under the terms of the GNU Lesser General Public License as published by
	the Free Software Foundation; either version 2.1 of the License, or (at
	your option) any later version.

	This library is distributed in the hope that it will be useful, but
	WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser
	General Public License for more details.

	You should have received a copy of the GNU Lesser General Public License
	along with this library; if not, write to the Free Software Foundation,
	Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

/*! @file mainstate.c
 * @brief Main State machine for template application.
 *
 * Makes use of Framework HSM module.
	************************************************************************/

#include "OpenCV_Calibration.h"
#include "template.h"
#include "mainstate.h"
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

// Global image structure used with StateMachine
struct {
	IplImage *original, *calibrated, *undistort, *perspTrans;
} image;


const Msg mainStateMsg[] = {
	{ FRAMESEQ_EVT },
	{ FRAMEPAR_EVT },
	{ IPC_GET_APP_STATE_EVT },
	{ SHOW_RAW_IMAGE_EVT },
	{ SHOW_UNDIST_IMAGE_EVT },
	{ GO_TO_LIVE_VIEW_EVT },
	{ GO_TO_CALIBRATION_EVT },
	{ GET_NEW_GRID_EVT },
	{ IMG_SEQ_EVT },
	{ CALIBRATE_CAMERA_EVT },
	{ UNDISTORT_GRID_EVT }
};

/*********************************************************************//*!
 * @brief Inline function to throw an event to be handled by the statemachine.
 *
 * @param pHsm Pointer to state machine
 * @param evt Event to be thrown.
 *//*********************************************************************/

void ThrowEvent(struct MainState *pHsm, unsigned int evt)
{
	const Msg *pMsg = &mainStateMsg[evt];
	HsmOnEvent((Hsm*)pHsm, pMsg);
}

/*********************************************************************//*!
 * @brief Checks for IPC events, schedules their handling and
 * acknowledges any executed ones.
 *
 * @param pMainState Initalized HSM main state variable.
 * @return 0 on success or an appropriate error code.
 *//*********************************************************************/
/*
static OSC_ERR HandleIpcRequests(MainState *pMainState)
{
	OSC_ERR err;
	uint32 paramId;

	err = CheckIpcRequests(&paramId);
	if (err == SUCCESS)
		{

		}
	return err;
}
*/

Msg const *MainState_top(MainState *me, Msg *msg)
{
	bool ModelPresent = 0;
	switch (msg->evt)
	{
	/*
	case ENTRY_EVT:
		return 0;
	*/
	case START_EVT:
		OscLog(INFO, "Start Event\n");
		if(ModelPresent)
		{
			STATE_START(me, &me->Undistort);
			OscLog(INFO, "ModelPresent\n");
		}else
		{
			STATE_START(me, &me->Raw);
			OscLog(INFO, "ModelNotPresent\n");
		}

		return 0;
	case SHOW_RAW_IMAGE_EVT:
		STATE_TRAN(me, &me->Raw);
		return 0;
	case FRAMESEQ_EVT:
		return 0;
	case FRAMEPAR_EVT:
		return 0;
	case IPC_GET_APP_STATE_EVT:
		return 0;
	}
	return msg;
}

Msg const *MainState_LiveViewMode(MainState *me, Msg *msg)
{
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State LiveViewMode!\n");
		return 0;
	case GO_TO_CALIBRATION_EVT:
		OscLog(INFO, "GO_TO_CALIBRATION_EVT\n");
		STATE_TRAN(me, &me->CalibrationMode);
		return 0;
	}
	return msg;
}

Msg const *MainState_CalibrationMode(MainState *me, Msg *msg)
{
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State CalibrationMode!\n");
		return 0;
	case GO_TO_LIVE_VIEW_EVT:
		OscLog(INFO, "GO_TO_LIVE_VIEW_EVT\n");
		STATE_TRAN(me, &me->Undistort);
	case GET_NEW_GRID_EVT:
		OscLog(INFO, "GET_NEW_GRID_EVT\n");
		STATE_TRAN(me, &me->WaitForGrid);
		return 0;
	}
	return msg;
}

Msg const *MainState_Raw(MainState *me, Msg *msg)
{
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State Raw!\n");
		return 0;
	case SHOW_UNDIST_IMAGE_EVT:
		OscLog(INFO, "SHOW_UNDIST_IMAGE_EVT\n");
		STATE_TRAN(me, &me->Undistort);
		return 0;
	}
	return msg;
}

Msg const *MainState_Undistort(MainState *me, Msg *msg)
{
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State Undistort!\n");
		usleep(2000000);
		OscLog(INFO, "Waited 5s !\n");
		return 0;
	}
	return msg;
}

Msg const *MainState_WaitForGrid(MainState *me, Msg *msg)
{
	bool read_file = 1;
	const char* srcFile = IMAGE_DIRECTORY "left12.bmp";
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State WaitForGrid!\n");
		if(read_file){
			OscLog(INFO, "Read bmp\n");
			image.original = readImage(srcFile);
		}else{
			OscLog(INFO, "Read image from sensor\n");
		}
		return 0;
	case IMG_SEQ_EVT:
		OscLog(INFO, "IMG_SEQ_EVT\n");
		STATE_TRAN(me, &me->ShowGrid);
		return 0;
	}
	return msg;
}

Msg const *MainState_ShowGrid(MainState *me, Msg *msg)
{
	const char* showFile = IMAGE_DIRECTORY "original.bmp";

	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State ShowGrid!\n");
		writeImage(showFile, image.original);
		return 0;
	case CALIBRATE_CAMERA_EVT:
		OscLog(INFO, "CALIBRATE_CAMERA_EVT\n");
		STATE_TRAN(me, &me->CalibrateCamera);
		return 0;
	}
	return msg;
}

Msg const *MainState_CalibrateCamera(MainState *me, Msg *msg)
{
	const char* calibFile = IMAGE_DIRECTORY "calibrated.bmp";

	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State CalibrateCamera!\n");
		calib = cmCalibrateCamera(calib.n_boards, calib.board_w, calib.board_h, image.original);
		image.calibrated = cmDrawChessboardCorners(image.original, calib);
		if(calib.corners_found){
				// Save calibrated image
				writeImage(calibFile, image.calibrated);
			} else {
				OscLog(ERROR, "wrong input parameters x, y\n");
			}
		return 0;
	case UNDISTORT_GRID_EVT:
		OscLog(INFO, "UNDISTORT_GRID_EVT\n");
		STATE_TRAN(me, &me->UndistortGridAndShow);
		return 0;
	}
	return msg;
}

Msg const *MainState_UndistortGridAndShow(MainState *me, Msg *msg)
{
	const char* undistFile = IMAGE_DIRECTORY "undistorted.bmp";
	const char* perspTransFile = IMAGE_DIRECTORY "birdseye.bmp";
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State UndistortGridAndShow!\n");
		image.undistort = cmUndistort(calib, image.original);
		// Save undistorted image
		writeImage(undistFile, image.undistort);
		persp = cmCalculatePerspectiveTransform(calib, persp.Z, persp.perspTransform);
		if(persp.perspTransform)
		{
			image.perspTrans = cmPerspectiveTransform(persp, image.undistort);
			// Save birds_eye image
			writeImage(perspTransFile, image.perspTrans);
		}
		return 0;
	}
	return msg;
}


void MainStateConstruct(MainState *me)
{
	//OscHsmCreate(hFramework);

	HsmCtor((Hsm *)me, "MainState", (EvtHndlr)MainState_top);

	StateCtor(&me->LiveViewMode, "Live-view Mode", &((Hsm *)me)->top, (EvtHndlr)MainState_LiveViewMode);
	StateCtor(&me->CalibrationMode, "Calibration Mode", &((Hsm *)me)->top, (EvtHndlr)MainState_CalibrationMode);
	StateCtor(&me->Raw, "Raw", &me->LiveViewMode, (EvtHndlr)MainState_Raw);
	StateCtor(&me->Undistort, "Undistort", &me->LiveViewMode, (EvtHndlr)MainState_Undistort);
	StateCtor(&me->WaitForGrid, "Wait for Grid", &me->CalibrationMode, (EvtHndlr)MainState_WaitForGrid);
	StateCtor(&me->ShowGrid, "Show Grid", &me->CalibrationMode, (EvtHndlr)MainState_ShowGrid);
	StateCtor(&me->CalibrateCamera, "Calibrate Camera", &me->CalibrationMode, (EvtHndlr)MainState_CalibrateCamera);
	StateCtor(&me->UndistortGridAndShow, "Undistort Grid and Show", &me->CalibrationMode, (EvtHndlr)MainState_UndistortGridAndShow);

}

int readLine() {
	char buffer[100];
	int n = 0;
	char command[100];
	int args[5];

	printf("> ");
	fflush(stdout);

	fgets(buffer, sizeof buffer, stdin);

	n = sscanf(buffer, "%s %d %d %d %d %d", command, &args[0], &args[1], &args[2], &args[3], &args[4]);

	if (n > 0) {
		//printf("%d: %s %d %d %d %d %d\n", n, command, args[0], args[1], args[2], args[3], args[4]);

		if (strcmp(command, "show-raw") == 0) {
			printf("%s", "ThrowEvent(&mainState, SHOW_RAW_IMAGE_EVT);");
		}

		}
	}
}

OSC_ERR StateControl( void)
{
	//OscHsmCreate( data.hFramework);

	/* Setup main state machine */
	MainState mainState;
	MainStateConstruct(&mainState);

	HsmOnStart((Hsm *)&mainState);
	//OscSimInitialize();

	bool ShowRawImageBtn = 1;
	bool ShowUndistImageBtn = 0, GoToLiveViewBtn = 0, GoToCalibrateBtn = 1, GetNewGridBtn = 1, ImageReady = 1,
		 CalibrateCameraBtn = 1, UndistortGridBtn = 1;

	/*----------- infinite main loop */
	/*	Enters in state Raw and then goes to state CalibrationMode and follows all states to calibrate and
		undistort a given bmp.

		Output images:
		original.bmp, calibrated.bmp, undistort.bmp, birdseye.bmp
	*/
	while (TRUE)
	{
		readLine();
		/*
	// Input parameters
		calib.n_boards = 1; 			// Number of chessboard views
		calib.board_w = 6;				// Number of points horizontal
		calib.board_h = 9;				// Number of points vertical

	    persp.Z = 45;
		persp.perspTransform = TRUE;//FALSE;

	// ----------------------------------------------------------------------------------------------------
		if(ShowRawImageBtn){
			OscLog(INFO, "ShowRawImageBtn\n");
			ThrowEvent(&mainState, SHOW_RAW_IMAGE_EVT);
			ShowRawImageBtn = 0;
		}
		if(ShowUndistImageBtn){
			OscLog(INFO, "ShowUndistImageBtn\n");
			ThrowEvent(&mainState, SHOW_UNDIST_IMAGE_EVT);
			ShowUndistImageBtn = 0;
		}
		if(GoToLiveViewBtn){
			OscLog(INFO, "GoToLiveViewBtn\n");
			ThrowEvent(&mainState, GO_TO_LIVE_VIEW_EVT);
			GoToLiveViewBtn = 0;
		}
		if(GoToCalibrateBtn){
			OscLog(INFO, "GoToCalibrateBtn\n");
			ThrowEvent(&mainState, GO_TO_CALIBRATION_EVT);
			GoToCalibrateBtn = 0;
		}
		if(GetNewGridBtn){
			OscLog(INFO, "GetNewGridBtn\n");
			ThrowEvent(&mainState, GET_NEW_GRID_EVT);
			GetNewGridBtn = 0;
		}
		if(ImageReady){
			OscLog(INFO, "ImageReady\n");
			ThrowEvent(&mainState, IMG_SEQ_EVT);
			ImageReady = 0;
		}
		if(CalibrateCameraBtn){
			OscLog(INFO, "CalibrateCameraBtn\n");
			ThrowEvent(&mainState, CALIBRATE_CAMERA_EVT);
			CalibrateCameraBtn = 0;
		}
		if(UndistortGridBtn){
			OscLog(INFO, "UndistortGridBtn\n");
			ThrowEvent(&mainState, UNDISTORT_GRID_EVT);
			UndistortGridBtn = 0;
		}
		*/
		/* Advance the simulation step counter. */
		//OscSimStep();
	} /* end while ever */

	return SUCCESS;
	//OscHsmDestroy( data.hFramework);
}
