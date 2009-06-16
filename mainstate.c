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

#include "template.h"
#include "mainstate.h"
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

const Msg mainStateMsg[] = {
	{ FRAMESEQ_EVT },
	{ FRAMEPAR_EVT },
	{ IPC_GET_APP_STATE_EVT },
	{ MODEL_NOT_PRESENT_EVT },
	{ MODEL_PRESENT_EVT },
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
	switch (msg->evt)
	{
	case MODEL_NOT_PRESENT_EVT:
		STATE_START(me, &me->Raw);
		return 0;
	case MODEL_PRESENT_EVT:
		STATE_START(me, &me->Undistort);
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
		return 0;
	}
	return msg;
}

Msg const *MainState_WaitForGrid(MainState *me, Msg *msg)
{
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State WaitForGrid!\n");
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
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State ShowGrid!\n");
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
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State CalibrateCamera!\n");
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
	switch (msg->evt)
	{
	case ENTRY_EVT:
		OscLog(INFO, "Enter in State UndistortGridAndShow!\n");
		return 0;
	}
	return msg;
}


void MainStateConstruct(MainState *me)
{
	//OscHsmCreate(hFramework);
	/*
	HsmCtor((Hsm *)me, "MainState", (EvtHndlr)MainState_top);
	StateCtor(&me->captureRaw, "Capture Raw", &((Hsm *)me)->top, (EvtHndlr)MainState_CaptureRaw);
	StateCtor(&me->captureColor, "Capture Color", &((Hsm *)me)->top, (EvtHndlr)MainState_CaptureColor);
	*/
	HsmCtor((Hsm *)me, "MainState", (EvtHndlr)MainState_top);

	//StateCtor(&me->Initial, "Initial", &((Hsm *)&me)->top, (EvtHndlr)MainState_Initial);
	StateCtor(&me->LiveViewMode, "Live-view Mode", &((Hsm *)&me)->top, (EvtHndlr)MainState_LiveViewMode);
	StateCtor(&me->CalibrationMode, "Calibration Mode", &((Hsm *)&me)->top, (EvtHndlr)MainState_CalibrationMode);
	StateCtor(&me->Raw, "Raw",&me->LiveViewMode, (EvtHndlr)MainState_Raw);
	StateCtor(&me->Undistort, "Undistort",&me->LiveViewMode, (EvtHndlr)MainState_Undistort);
	StateCtor(&me->WaitForGrid, "Wait for Grid",&me->CalibrationMode, (EvtHndlr)MainState_WaitForGrid);
	StateCtor(&me->ShowGrid, "Show Grid",&me->CalibrationMode, (EvtHndlr)MainState_ShowGrid);
	StateCtor(&me->CalibrateCamera, "Calibrate Camera",&me->CalibrationMode, (EvtHndlr)MainState_CalibrateCamera);
	StateCtor(&me->UndistortGridAndShow, "Undistort Grid and Show",&me->CalibrationMode, (EvtHndlr)MainState_UndistortGridAndShow);

}


OSC_ERR StateControl( void)
{
	// OSC_ERR camErr, err;

	MainState mainState;
	OscHsmCreate( data.hFramework);

	/* Setup main state machine */
	MainStateConstruct(&mainState);
/*
	HsmCtor((Hsm *)&mainState, "MainState", (EvtHndlr)MainState_top);

	//StateCtor(&me->Initial, "Initial", &((Hsm *)&me)->top, (EvtHndlr)MainState_Initial);
	StateCtor(&mainState.LiveViewMode, "Live-view Mode", &((Hsm *)&mainState)->top, (EvtHndlr)MainState_LiveViewMode);
	StateCtor(&mainState.CalibrationMode, "Calibration Mode", &((Hsm *)&mainState)->top, (EvtHndlr)MainState_CalibrationMode);
	StateCtor(&mainState.Raw, "Raw",&(&mainState)->LiveViewMode, (EvtHndlr)MainState_Raw);
	StateCtor(&mainState.Undistort, "Undistort",&(&mainState)->LiveViewMode, (EvtHndlr)MainState_Undistort);
	StateCtor(&mainState.WaitForGrid, "Wait for Grid",&(&mainState)->CalibrationMode, (EvtHndlr)MainState_WaitForGrid);
	StateCtor(&mainState.ShowGrid, "Show Grid",&(&mainState)->CalibrationMode, (EvtHndlr)MainState_ShowGrid);
	StateCtor(&mainState.CalibrateCamera, "Calibrate Camera",&(&mainState)->CalibrationMode, (EvtHndlr)MainState_CalibrateCamera);
	StateCtor(&mainState.UndistortGridAndShow, "Undistort Grid and Show",&(&mainState)->CalibrationMode, (EvtHndlr)MainState_UndistortGridAndShow);
*/
	HsmOnStart((Hsm *)&mainState);

	OscSimInitialize();

	int ModelPresent = 0;
	int ShowRawImageBtn = 1;
	int ShowUndistImageBtn, GoToLiveViewBtn, GoToCalibrateBtn, GetNewGridBtn, ImageReady,
		CalibrateCameraBtn, UndistortGridBtn;

	/*----------- infinite main loop */
	while (TRUE)
	{
		if(!ModelPresent)
				ThrowEvent(&mainState, MODEL_NOT_PRESENT_EVT);
		if(ModelPresent)
			ThrowEvent(&mainState, MODEL_PRESENT_EVT);
		if(ShowRawImageBtn)
			ThrowEvent(&mainState, SHOW_RAW_IMAGE_EVT);
				OscLog(INFO, "ShowRawImageBtn\n");
		if(ShowUndistImageBtn)
			ThrowEvent(&mainState, SHOW_UNDIST_IMAGE_EVT);
		if(GoToLiveViewBtn)
			ThrowEvent(&mainState, GO_TO_LIVE_VIEW_EVT);
		if(GoToCalibrateBtn)
			ThrowEvent(&mainState, GO_TO_CALIBRATION_EVT);
		if(GetNewGridBtn)
			ThrowEvent(&mainState, GET_NEW_GRID_EVT);
		if(ImageReady)
			ThrowEvent(&mainState, IMG_SEQ_EVT);
		if(CalibrateCameraBtn)
			ThrowEvent(&mainState, CALIBRATE_CAMERA_EVT);
		if(UndistortGridBtn)
			ThrowEvent(&mainState, UNDISTORT_GRID_EVT);

		/* Advance the simulation step counter. */
		OscSimStep();
	} /* end while ever */

	return SUCCESS;
	OscHsmDestroy( data.hFramework);
}
