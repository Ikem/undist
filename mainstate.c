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
	IplImage *original, *calibrated, *undistort, *lvundist;
} image;


const Msg mainStateMsg[] = {
	{ IPC_GET_APP_STATE_EVT },
	{ SHOW_CAMERA_IMAGE_EVT },
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



Msg const *MainState_top(MainState *me, Msg *msg)
{
	bool ModelPresent = 0;
	switch (msg->evt)
	{
	case START_EVT:
		OscLog(INFO, "Start Event\n");
		persp = loadConfig();

		if(persp.undistort)
		{
			OscLog(INFO, "ModelPresent\n");
			STATE_START(me, &me->ShowCameraImage);
		}else
		{
			OscLog(INFO, "ModelNotPresent\n");
			STATE_START(me, &me->CalibrationMode);
		}
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
		data.ipc.state.appMode = appMode_LiveViewMode;
		return 0;
	case GO_TO_CALIBRATION_EVT:
		OscLog(INFO, "GO_TO_CALIBRATION_EVT\n");
		STATE_TRAN(me, &me->CalibrationMode);
		return 0;
	case SHOW_CAMERA_IMAGE_EVT:
		OscLog(INFO, "SHOW_CAMERA_IMAGE_EVT\n");
		STATE_TRAN(me, &me->ShowCameraImage);
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
		data.ipc.state.appMode = appMode_CalibrationMode;
		return 0;
	case GO_TO_LIVE_VIEW_EVT:
		OscLog(INFO, "GO_TO_LIVE_VIEW_EVT\n");
		persp.undistort = false;
		STATE_TRAN(me, &me->ShowCameraImage);
		return 0;
	case GET_NEW_GRID_EVT:
		OscLog(INFO, "GET_NEW_GRID_EVT\n");
		STATE_TRAN(me, &me->WaitForGrid);
		return 0;
	}
	return msg;
}

Msg const *MainState_ShowCameraImage(MainState *me, Msg *msg)
{
	const char* undistortFile = IMAGE_DIRECTORY "lvundist.bmp";
	const char* showFile = IMAGE_DIRECTORY "original.bmp";
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State ShowCameraImage!\n");
		captureImage(image.original);
		calib = loadModel();
		persp = loadConfig();
		if(persp.undistort)
		{
			undist = cmCalibrateUndistort(calib, image.original);
		}

		return 0;
	case IMG_SEQ_EVT:
		captureImage(image.original);
		writeImage(showFile, image.original);
		if(persp.undistort)
		{
			image.lvundist = cmUndistort(undist, image.original);
			if(persp.perspTransform)
			{
				image.lvundist = cmPerspectiveTransform(persp, image.lvundist);
			}
		}
		writeImage(undistortFile, image.lvundist);
		return 0;
	}
	return msg;
}


Msg const *MainState_WaitForGrid(MainState *me, Msg *msg)
{
	const char* showFile = IMAGE_DIRECTORY "original.bmp";
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State WaitForGrid!\n");
		captureImage(image.original);
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
		undist = cmCalibrateUndistort(calib, image.original);
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
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State UndistortGridAndShow!\n");
		image.undistort = cmUndistort(undist, image.original);
		// Save undistorted image
		writeImage(undistFile, image.undistort);
		persp = cmCalculatePerspectiveTransform(calib, persp.Z, persp.perspTransform);
		if(persp.perspTransform)
		{
			image.undistort = cmPerspectiveTransform(persp, image.undistort);
			// Save birds_eye image
			writeImage(undistFile, image.undistort);
		}
		saveConfig(persp);
		saveModel(calib);
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
	StateCtor(&me->ShowCameraImage, "ShowCameraImage", &me->LiveViewMode, (EvtHndlr)MainState_ShowCameraImage);
	StateCtor(&me->WaitForGrid, "Wait for Grid", &me->CalibrationMode, (EvtHndlr)MainState_WaitForGrid);
	StateCtor(&me->CalibrateCamera, "Calibrate Camera", &me->CalibrationMode, (EvtHndlr)MainState_CalibrateCamera);
	StateCtor(&me->UndistortGridAndShow, "Undistort Grid and Show", &me->CalibrationMode, (EvtHndlr)MainState_UndistortGridAndShow);

}

int readLine(MainState*  mainState) {
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

		if(strcmp(command, "show-camera") == 0){
			OscLog(INFO, "ShowRawImageBtn\n");
			ThrowEvent(mainState, SHOW_CAMERA_IMAGE_EVT);
		}
		if(strcmp(command, "live-view") == 0){
			OscLog(INFO, "GoToLiveViewBtn\n");
			ThrowEvent(mainState, GO_TO_LIVE_VIEW_EVT);
		}
		if(strcmp(command, "calibration-mode") == 0){
			OscLog(INFO, "GoToCalibrateBtn\n");
			ThrowEvent(mainState, GO_TO_CALIBRATION_EVT);
		}
		if(strcmp(command, "new-grid") == 0){
			OscLog(INFO, "GetNewGridBtn\n");
			ThrowEvent(mainState, GET_NEW_GRID_EVT);
		}
		if(strcmp(command, "calibrate") == 0){
			OscLog(INFO, "CalibrateCameraBtn\n");
			ThrowEvent(mainState, CALIBRATE_CAMERA_EVT);
		}
		if(strcmp(command, "undistort") == 0){
			OscLog(INFO, "UndistortGridBtn\n");
			ThrowEvent(mainState, UNDISTORT_GRID_EVT);
		}
	}else{
		OscLog(INFO, "Image ready\n");
		ThrowEvent(mainState, IMG_SEQ_EVT);
	}
}

/*********************************************************************//*!
 * @brief Checks for IPC events, schedules their handling and
 * acknowledges any executed ones.
 *
 * @param pMainState Initalized HSM main state variable.
 * @return 0 on success or an appropriate error code.
 *//*********************************************************************/
static OSC_ERR HandleIpcRequests(MainState *pMainState)
{
	OSC_ERR err;
	uint32 paramId;

	err = CheckIpcRequests(&paramId);
	if (err == SUCCESS)
	{
		/* We have a request. See to it that it is handled
		 * depending on the state we're in. */
		switch(paramId)
		{
#if 0
		case GET_APP_STATE:
			/* Request for the current state of the application. */
			ThrowEvent(pMainState, IPC_GET_APP_STATE_EVT);
			break;
		case GET_COLOR_IMG:
			/* Request for the live image. */
			ThrowEvent(pMainState, IPC_GET_COLOR_IMG_EVT);
			break;
		case GET_RAW_IMG:
			/* Request for the live image. */
			ThrowEvent(pMainState, IPC_GET_RAW_IMG_EVT);
			break;
		case SET_CAPTURE_MODE:
			/* Set the debayering option. */
			ThrowEvent(pMainState, IPC_SET_CAPTURE_MODE_EVT);
			break;
#endif
		case GO_TO_LIVE_VIEW_MODE:
			ThrowEvent(pMainState, GO_TO_LIVE_VIEW_EVT);
			data.ipc.enReqState = REQ_STATE_ACK_PENDING;
			break;
		case GO_TO_CALIBRATION_MODE:
			ThrowEvent(pMainState, GO_TO_CALIBRATION_EVT);
			data.ipc.enReqState = REQ_STATE_ACK_PENDING;
			break;
		case SET_UNDISTORT_ACTIVE:
			persp.undistort = *(bool *) data.ipc.req.pAddr;
			break;
		case GET_APP_STATE:
			memcpy(data.ipc.req.pAddr, (void *) &data.ipc.state, sizeof(data.ipc.state));
		//	*((void **) data.ipc.req.pAddr) = (void *) &data.ipc.state;
			data.ipc.enReqState = REQ_STATE_ACK_PENDING;
			break;
		default:
			OscLog(ERROR, "%s: Unkown IPC parameter ID (%d)!\n", __func__, paramId);
			data.ipc.enReqState = REQ_STATE_NACK_PENDING;
			break;
		}
	}
	else if (err == -ENO_MSG_AVAIL)
	{
		/* No new message available => do nothing. */
	}
	else
	{
		/* Error.*/
		OscLog(ERROR, "%s: IPC request error! (%d)\n", __func__, err);
		return err;
	}

	/* Try to acknowledge the new or any old unacknowledged
	 * requests. It may take several tries to succeed.*/
	err = AckIpcRequests();
	if (err != SUCCESS)
	{
		OscLog(ERROR, "%s: IPC acknowledge error! (%d)\n", __func__, err);
	}
	return err;
}

OSC_ERR StateControl( void)
{
	//OscHsmCreate( data.hFramework);

	image.lvundist = cvCreateImage(cvSize(OSC_CAM_MAX_IMAGE_WIDTH, OSC_CAM_MAX_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);	// Initialize image 752,480
	image.original = cvCreateImage(cvSize(OSC_CAM_MAX_IMAGE_WIDTH, OSC_CAM_MAX_IMAGE_HEIGHT), IPL_DEPTH_8U, 1);	// Initialize image 752,480

	/* Setup main state machine */
	MainState mainState;
	MainStateConstruct(&mainState);

	HsmOnStart((Hsm *)&mainState);
	//OscSimInitialize();


	/*----------- infinite main loop */
	/*	Enters in state Raw and then goes to state CalibrationMode and follows all states to calibrate and
		undistort a given bmp.

		Output images:
		original.bmp, calibrated.bmp, undistorted.bmp, lvundist.bmp
	*/

	while (TRUE)
	{


	// Input parameters
		calib.n_boards = 1; 			// Number of chessboard views
		calib.board_w = 6;				// Number of points horizontal
		calib.board_h = 9;				// Number of points vertical

	    persp.Z = 45;
		persp.perspTransform = FALSE;//FALSE;

		button.readFile = TRUE;

		HandleIpcRequests(&mainState);
		ThrowEvent(&mainState, IMG_SEQ_EVT);

	//	readLine(&mainState);






	// ----------------------------------------------------------------------------------------------------

/*
		if(ShowRawImageBtn){
			OscLog(INFO, "ShowRawImageBtn\n");
			ThrowEvent(&mainState, SHOW_RAW_IMAGE_EVT);
			ShowRawImageBtn = 0;
		}

		if(GoToCalibrateBtn){
			OscLog(INFO, "GoToCalibrateBtn\n");
			ThrowEvent(&mainState, GO_TO_CALIBRATION_EVT);
			GoToCalibrateBtn = 0;
		}

		if(GoToLiveViewBtn){
			OscLog(INFO, "GoToLiveViewBtn\n");
			ThrowEvent(&mainState, GO_TO_LIVE_VIEW_EVT);//GET_NEW_GRID_EVT);
			GoToLiveViewBtn = 0;
		}
*/
		/* Advance the simulation step counter. */
		//OscSimStep();
	} /* end while ever */

	return SUCCESS;
	//OscHsmDestroy( data.hFramework);
}
