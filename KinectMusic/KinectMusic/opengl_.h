//
//  opengl_.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 20/09/15.
//  Copyright (c) 2015 mgsvetlov. All rights reserved.
//

#ifndef __KinectMusic__opengl___
#define __KinectMusic__opengl___
#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "kinect.h"

//extern int g_argc;
//char **g_argv;

//extern int depth_window;
//extern int video_window;

// back: owned by libfreenect (implicit for depth)
// mid: owned by callbacks, "latest frame ready"
// front: owned by GL, "currently being drawn"
//uint8_t *depth_front;

/*extern GLuint gl_depth_tex;
extern GLuint gl_rgb_tex;

void DispatchDraws();
void DrawDepthScene();
void ReSizeGLScene(int Width, int Height);
void InitGL(int Width, int Height);
void keyPressed(unsigned char key, int x, int y);
void *gl_threadfunc(void *arg);
*/
#endif /* defined(__KinectMusic__opengl___) */
