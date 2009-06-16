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

/*! @file main.c
 * @brief Main file of the template application. Mainly contains initialization
 * code.
 */

#include "template.h"
#include <string.h>
#include <sched.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>

/*! @brief This stores all variables needed by the algorithm. */
struct TEMPLATE data;

/*! @brief The framework module dependencies of this application. */
struct OSC_DEPENDENCY deps[] = {
	{ "log", OscLogCreate, OscLogDestroy },
	{ "sup", OscSupCreate, OscSupDestroy },
	{ "bmp", OscBmpCreate, OscBmpDestroy },
	{ "cam", OscCamCreate, OscCamDestroy },
	{ "hsm", OscHsmCreate, OscHsmDestroy },
	{ "ipc", OscIpcCreate, OscIpcDestroy },
	{ "vis", OscVisCreate, OscVisDestroy },
	{ "gpio", OscGpioCreate, OscGpioDestroy }
};

/*********************************************************************//*!
 * @brief Initialize everything so the application is fully operable
 * after a call to this function.
 *
 * @return SUCCESS or an appropriate error code.
 *//*********************************************************************/
static OSC_ERR init(const int argc, const char * argv[])
{
	OSC_ERR err = SUCCESS;
	uint8 multiBufferIds[2] = {0, 1};

	memset(&data, 0, sizeof(struct TEMPLATE));

	/******* Create the framework **********/
	err = OscCreate(&data.hFramework);
	if (err < 0)
	{
		fprintf(stderr, "%s: Unable to create framework.\n", __func__);
		return err;
	}

	/******* Load the framework module dependencies. **********/
	err = OscLoadDependencies(data.hFramework, deps, sizeof(deps)/sizeof(struct OSC_DEPENDENCY));

	if (err != SUCCESS)
	{
		fprintf(stderr, "%s: ERROR: Unable to load dependencies! (%d)\n", __func__, err);
		goto dep_err;
	}

	/********* Seed the random generator *************/
	srand(OscSupCycGet());

#if defined(OSC_HOST) || defined(OSC_SIM)
	err = OscFrdCreateConstantReader(&data.hFileNameReader, TEST_IMAGE_FN);
	if (err != SUCCESS)
	{
		OscLog(ERROR, "%s: Unable to create constant file name reader for %s! (%d)\n", __func__, TEST_IMAGE_FN, err);
		goto frd_err;
	}
	err = OscCamSetFileNameReader(data.hFileNameReader);
	if (err != SUCCESS)
	{
		OscLog(ERROR, "%s: Unable to set file name reader for camera! (%d)\n", __func__, err);
		goto frd_err;
	}
#endif /* OSC_HOST or OSC_SIM */

	/* Set the camera registers to sane default values. */
	err = OscCamPresetRegs();
	if (err != SUCCESS)
	{
		OscLog(ERROR, "%s: Unable to preset camera registers! (%d)\n", __func__, err);
		goto fb_err;
	}

	/* Set up two frame buffers with enough space for the maximum
	 * camera resolution in cached memory. */
	err = OscCamSetFrameBuffer(0, OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT, data.u8FrameBuffers[0], TRUE);
	if (err != SUCCESS)
	{
		OscLog(ERROR, "%s: Unable to set up first frame buffer!\n", __func__);
		goto fb_err;
	}
	err = OscCamSetFrameBuffer(1, OSC_CAM_MAX_IMAGE_WIDTH*OSC_CAM_MAX_IMAGE_HEIGHT, data.u8FrameBuffers[1], TRUE);
	if (err != SUCCESS)
	{
		OscLog(ERROR, "%s: Unable to set up second frame buffer!\n", __func__);
		goto fb_err;
	}

	/* Create a double-buffer from the frame buffers initilalized above.*/
	err = OscCamCreateMultiBuffer(2, multiBufferIds);
	if (err != SUCCESS)
	{
		OscLog(ERROR, "%s: Unable to set up multi buffer!\n", __func__);
		goto mb_err;
	}

	/* Register an IPC channel to the CGI for the user interface. */
	err = OscIpcRegisterChannel(&data.ipc.ipcChan, USER_INTERFACE_SOCKET_PATH, F_IPC_SERVER | F_IPC_NONBLOCKING);
	if (err != SUCCESS)
	{
		OscLog(ERROR, "%s: Unable to initialize IPC channel to web interface! (%d)\n", __func__, err);
		goto ipc_err;
	}

	err |= OscCamPerspectiveStr2Enum("DEFAULT", &data.perspective);
	if ( err != SUCCESS)
	{
		OscLog(ERROR, "%s: Invalid camera perspective.\n", __func__);
		goto per_err;
	}
	OscCamSetupPerspective( data.perspective);

	return SUCCESS;

per_err:
ipc_err:
mb_err:
fb_err:
#if defined(OSC_HOST) || defined(OSC_SIM)
frd_err:
#endif
	OscUnloadDependencies(data.hFramework, deps, sizeof(deps)/sizeof(struct OSC_DEPENDENCY));
dep_err:
	OscDestroy(&data.hFramework);

	return err;
}

OSC_ERR Unload()
{
	/******** Unload the framework module dependencies **********/
	OscUnloadDependencies(data.hFramework, deps, sizeof(deps)/sizeof(struct OSC_DEPENDENCY));

	OscDestroy(data.hFramework);

	return SUCCESS;
}

/*********************************************************************//*!
 * @brief Program entry
 *
 * @param argc Command line argument count.
 * @param argv Command line argument strings.
 * @return 0 on success
 *//*********************************************************************/
int main(const int argc, const char * argv[])
{
	OSC_ERR err = SUCCESS;

	err = init(argc, argv);
	if (err != SUCCESS)
	{
		OscLog(ERROR, "%s: Initialization failed!(%d)\n", __func__, err);
		return err;
	}
	OscLog(INFO, "Initialization successful!\n");

	OscLogSetConsoleLogLevel(INFO);
	OscLogSetFileLogLevel(WARN);

	//StateControl();
///*
	uint64_t time = OscSupCycGet64();
	cvCalib();
    time = OscSupCycGet64() - time;
    printf("%lf s\n", OscSupCycToMicroSecs64(time) / 1000000.);
//*/
	Unload();
	return 0;
}
