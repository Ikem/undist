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

/*! @file mainstate.h
 * @brief Definitions for main state machine
	************************************************************************/

#ifndef MAINSTATE_H
#define MAINSTATE_H

#include "template.h"

/*
enum MainStateEvents {
	FRAMESEQ_EVT,       		// frame ready to process (before setting up next frame capture)
	FRAMEPAR_EVT,       		// frame ready to process (parallel to next capture)
	IPC_GET_APP_STATE_EVT, 		// Webinterface asks for the current application state.
	IPC_GET_COLOR_IMG_EVT, 		// Webinterface asks for a color image.
	IPC_GET_RAW_IMG_EVT, 		// Webinterface asks for a raw image.
	IPC_SET_CAPTURE_MODE_EVT 	// Webinterface wants to set whether we capture color or raw images.
};
*/

enum MainStateEvents {
	IPC_GET_APP_STATE_EVT, 		// Webinterface asks for the current application state.
	SHOW_CAMERA_IMAGE_EVT,
	GO_TO_LIVE_VIEW_EVT,
	GO_TO_CALIBRATION_EVT,
	GET_NEW_GRID_EVT,
	IMG_SEQ_EVT,
	CALIBRATE_CAMERA_EVT,
	UNDISTORT_GRID_EVT,
	SAVE_MODEL_CONFIG_EVT
};


/* Define states */
typedef struct MainState {
	Hsm super;
	/* Declare all states and substates that are going to be used. */
	State LiveViewMode, CalibrationMode;
	State ShowCameraImage, WaitForGrid, CalibrateCamera, UndistortGridAndShow;
} MainState;


void MainStateConstruct(MainState *me);


#endif /*MAINSTATE_H*/
