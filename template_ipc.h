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

/*! @file template_ipc.h
 * @brief Shared header file between application and its
 * CGI. Contains all information relevant to IPC between these two.
 */

#ifndef TEMPLATE_IPC_H_
#define TEMPLATE_IPC_H_

int dummy; // FIXME: May be removed

/* The parameter IDs to identify the different requests/responses. */
enum EnIpcParamIds
{
	GET_APP_STATE,
	GO_TO_LIVE_VIEW_MODE,
	GO_TO_CALIBRATION_MODE,
	SET_UNDISTORT_ACTIVE,
	GET_NEW_GRID,
	CALIBRATE_CAMERA,
	UNDISTORT_GRID,
	SAVE_MODEL_CONFIG
};

/*! @brief The path of the unix domain socket used for IPC between the application and its user interface. */
#define USER_INTERFACE_SOCKET_PATH "/tmp/IPCSocket.sock"

/*! @brief Describes a rectangular sub-area of an image. */
struct IMG_RECT
{
	/*! @brief Rectangle width. */
	uint16 width;
	/*! @brief Rectangle height. */
	uint16 height;
	/*! @brief X Coordinate of the lower left corner.*/
	uint16 xPos;
	/*! @brief Y Coordinate of the lower left corne.*/
	uint16 yPos;
};

/*! @brief Object describing all the state information the web interface needs to know about the application. */
struct APPLICATION_STATE
{
	/*! @brief Whether a new image is ready to display by the web interface. */
	enum {
		appMode_LiveViewMode,
		appMode_CalibrationMode
	} appMode;
};

enum imageSource {
	imageSource_file,
	imageSource_sensor
};

struct CALIBRATE_CAMERA
{
	int boardW, boardH;
};

struct UNDISTORT_GRID
{
	bool perspTransform;
	int Z;
};

#endif /*TEMPLATE_IPC_H_*/
