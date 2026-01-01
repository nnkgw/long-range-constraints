#pragma once

#if defined(_WIN32) || defined(WIN32)
  #pragma warning(disable:4996)
  #include <GL/freeglut.h>
  #define LRC_HAS_FREEGLUT 1
#elif defined(__APPLE__) || defined(MACOSX)
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  #define GL_SILENCE_DEPRECATION
  #include <GLUT/glut.h>
#else
  #if defined(__has_include)
    #if __has_include(<GL/freeglut.h>)
      #include <GL/freeglut.h>
      #define LRC_HAS_FREEGLUT 1
    #elif __has_include(<GL/glut.h>)
      #include <GL/glut.h>
    #else
      #include <GL/glut.h>
    #endif
  #else
    #include <GL/glut.h>
  #endif
#endif
