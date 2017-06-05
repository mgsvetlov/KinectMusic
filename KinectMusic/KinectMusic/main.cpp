/*
 * This file is part of the OpenKinect Project. http://www.openkinect.org
 *
 * Copyright (c) 2010 individual OpenKinect contributors. See the CONTRIB file
 * for details.
 *
 * This code is licensed to you under the terms of the Apache License, version
 * 2.0, or, at your option, the terms of the GNU General Public License,
 * version 2.0. See the APACHE20 and GPL2 files for the text of the licenses,
 * or the following URLs:
 * http://www.apache.org/licenses/LICENSE-2.0
 * http://www.gnu.org/licenses/gpl-2.0.txt
 *
 * If you redistribute this file in source form, modified or unmodified, you
 * may:
 *   1) Leave this header intact and distribute it under the same terms,
 *      accompanying it with the APACHE20 and GPL20 files, or
 *   2) Delete the Apache 2.0 clause and accompany it with the GPL2 file, or
 *   3) Delete the GPL v2 clause and accompany it with the APACHE20 file
 * In all cases you must keep the copyright notice intact and include a copy
 * of the CONTRIB file.
 *
 * Binary distributions must follow the binary distribution requirements of
 * either License.
 */


/*#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "libfreenect.h"*/

#include <unistd.h>
#include <math.h>

#include "kinect.h"
#include "analyze.h"
#include "visualization.h"
#include "gestureExtraction/types.h"
#include "gestureExtraction/gesture/gesturefabrique.h"
#include "share.h"
#include "log/logs.h"
#include "config/config.h"

//#include "opengl_.h"


int main(int argc, char **argv)
{
    Config::setFileName("../../../config.ini");
    Config* config = Config::instance();
    if(config == nullptr){
        Logs::closeLogs();
        return -1;
    }
    
	//depth_mid = (uint8_t*)malloc(640*480*3);
	//depth_front = (uint8_t*)malloc(640*480*3);
    
	printf("Kinect camera test\n");
    
	/*int i;
	for (i=0; i<2048; i++) {
		float v = i/2048.0;
		v = powf(v, 3)* 6;
		t_gamma[i] = v*6*256;
	}*/
    
	//g_argc = argc;
	//g_argv = argv;
    
	if (freenect_init(&f_ctx, NULL) < 0) {
		printf("freenect_init() failed\n");
		return 1;
	}
    
	freenect_set_log_level(f_ctx, FREENECT_LOG_DEBUG);
	freenect_select_subdevices(f_ctx, (freenect_device_flags)(FREENECT_DEVICE_MOTOR | FREENECT_DEVICE_CAMERA));
    
	int nr_devices = freenect_num_devices (f_ctx);
	printf ("Number of devices found: %d\n", nr_devices);
    
	int user_device_number = 0;
	if (nr_devices < 1) {
		freenect_shutdown(f_ctx);
		return 1;
	}
    
	if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
		printf("Could not open device\n");
		freenect_shutdown(f_ctx);
		return 1;
	}
   
    pthread_t freenect_thread;
	int res = pthread_create(&freenect_thread, NULL, freenect_threadfunc, NULL);
	if (res) {
		printf("pthread_create freenect_thread failed\n");
		freenect_shutdown(f_ctx);
		return 1;
	}
    
    size_t analyzeThreadsCount = 2;
    pthread_t* p_analyze_create = new pthread_t[analyzeThreadsCount];
    for(int i = 0; i < analyzeThreadsCount; i++){
        res = pthread_create(&p_analyze_create[i], NULL, analyze_threadfunc, NULL);
        if (res) {
            printf("pthread_create   p_analyze_create[%u] failed\n", i);
            return 1;
        }
    }
    
    while(true) {
        if(config->getIsVisualisation()){
            if(Visualization::getIsNeedRedraw()){
                if(!Visualization::showImage())
                    break;
            }
            else {
                usleep(10);
            }
        }
        else {
            std::string s;
            std::cin >> s;
                if(s == "q")
                    break;
            usleep(10);
        }
    }
    
    die_gesture = 1;
    for(int i = 0; i < analyzeThreadsCount; i++){
        pthread_join(p_analyze_create[i], NULL);
    }
    
    die_kinect = 1;
    pthread_join(freenect_thread, NULL);
    
    GestureFabrique::destroy();
    Share::destroy();
    Logs::closeLogs();
    free(rgb_back);
    free(rgb_mid);
    free(rgb_front);

	// OS X requires GLUT to run on the main thread
	//gl_threadfunc(NULL);
    
	return 0;
}
