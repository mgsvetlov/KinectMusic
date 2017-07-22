//
//  opengl_.c
//  KinectMusic
//
//  Created by Mikhail Svetlov on 20/09/15.
//  Copyright (c) 2015 mgsvetlov. All rights reserved.
//

//#include "opengl_.h"
//#include "csound_.h"

//int g_argc;

/*int depth_window;
int video_window;

GLuint gl_depth_tex;
GLuint gl_rgb_tex;

void DispatchDraws() {
    pthread_mutex_lock(&ExtractFrameData::depth_mutex);
    if (got_depth) {
        glutSetWindow(depth_window);
        glutPostRedisplay();
    }
    pthread_mutex_unlock(&ExtractFrameData::depth_mutex);
    
}

void DrawDepthScene()
{
    pthread_mutex_lock(&ExtractFrameData::depth_mutex);
    if (got_depth) {
        uint8_t* tmp = depth_front;
        depth_front = depth_mid;
        depth_mid = tmp;
        got_depth = 0;
    }
    pthread_mutex_unlock(&ExtractFrameData::depth_mutex);
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    
    glEnable(GL_TEXTURE_2D);
    
    glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, 3, 640, 480, 0, GL_RGB, GL_UNSIGNED_BYTE, depth_front);
    
    glBegin(GL_TRIANGLE_FAN);
    glColor4f(1.0f, 1.0f, 1.0f, 1.0f);
    glTexCoord2f(1, 0); glVertex3f(0,0,0);
    glTexCoord2f(0, 0); glVertex3f(640,0,0);
    glTexCoord2f(0, 1); glVertex3f(640,480,0);
    glTexCoord2f(1, 1); glVertex3f(0,480,0);
    glEnd();
    
    glutSwapBuffers();
}

void ReSizeGLScene(int Width, int Height)
{
    glViewport(0,0,Width,Height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho (0, Width, Height, 0, -1.0f, 1.0f);
    glMatrixMode(GL_MODELVIEW);
}

void InitGL(int Width, int Height)
{
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClearDepth(1.0);
    glDepthFunc(GL_LESS);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glShadeModel(GL_SMOOTH);
    glGenTextures(1, &gl_depth_tex);
    glBindTexture(GL_TEXTURE_2D, gl_depth_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glGenTextures(1, &gl_rgb_tex);
    glBindTexture(GL_TEXTURE_2D, gl_rgb_tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    ReSizeGLScene(Width, Height);
}

void keyPressed(unsigned char key, int x, int y)
{
    if (key == 27) {
        die = 1;
        pthread_join(freenect_thread, NULL);
        pthread_join(csound_thread, NULL);
        glutDestroyWindow(depth_window);
        glutDestroyWindow(video_window);
        free(depth_mid);
        free(depth_front);
        free(rgb_back);
        free(rgb_mid);
        free(rgb_front);
        // Not pthread_exit because OSX leaves a thread lying around and doesn't exit
        exit(0);
    }
}

void *gl_threadfunc(void *arg)
{
    printf("GL thread\n");
    
    glutInit(&g_argc, g_argv);
    
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);
    glutInitWindowSize(640, 480);
    glutInitWindowPosition(0, 0);
    
    depth_window = glutCreateWindow("Depth");
    glutDisplayFunc(&DrawDepthScene);
    glutIdleFunc(&DispatchDraws);
    glutKeyboardFunc(&keyPressed);
    InitGL(640, 480);
    
    glutMainLoop();
    
    return NULL;
}
*/