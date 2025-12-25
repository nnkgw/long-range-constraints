// Long Range Constraints demo (rigid body chain)
// Step 0 refactor: introduce Scene abstraction without changing behavior.

#if defined(WIN32)
#pragma warning(disable:4996)
#include <GL/freeglut.h>
#elif defined(__APPLE__) || defined(MACOSX)
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#define GL_SILENCE_DEPRECATION
#include <GLUT/glut.h>
#else
#if defined(__has_include)
#if __has_include(<GL/freeglut.h>)
#include <GL/freeglut.h>
#elif __has_include(<GL/glut.h>)
#include <GL/glut.h>
#else
#include <GLUT/glut.h>
#endif
#else
#include <GL/freeglut.h>
#endif
#endif

#include <cstdlib>

#include "SceneManager.h"
#include "ChainLRCScene.h"

static SceneManager g_sceneManager;

static IScene* activeScene() {
  return g_sceneManager.active();
}

static void displayCb() {
  IScene* s = activeScene();
  if (s) s->render();
}

static void reshapeCb(int w, int h) {
  IScene* s = activeScene();
  if (s) s->reshape(w, h);
}

static void idleCb() {
  IScene* s = activeScene();
  if (s) s->idle();
}

static void keyboardCb(unsigned char key, int x, int y) {
  // Optional: scene switching keys (does not affect existing controls).
  if (key >= '1' && key <= '9') {
    std::size_t idx = (std::size_t)(key - '1');
    if (idx < g_sceneManager.count()) {
      g_sceneManager.setActiveIndex(idx);
      IScene* s = activeScene();
      if (s) {
        s->reset();
        s->usage();
      }
      return;
    }
  }

  IScene* s = activeScene();
  if (s) s->onKeyboard(key, x, y);
}

static void mouseButtonCb(int button, int state, int x, int y) {
  IScene* s = activeScene();
  if (s) s->onMouseButton(button, state, x, y);
}

static void mouseMotionCb(int x, int y) {
  IScene* s = activeScene();
  if (s) s->onMouseMotion(x, y);
}

int main(int argc, char** argv) {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  glutInitWindowSize(1280, 720);
  glutCreateWindow("LRC demo");

  glEnable(GL_DEPTH_TEST);
  glClearColor(0.05f, 0.06f, 0.07f, 1.0f);

  // Step 0: register a single scene (original behavior).
  g_sceneManager.addScene<ChainLRCScene>();

  IScene* s = activeScene();
  if (s) {
    s->reset();
    s->usage();
  }

  glutDisplayFunc(displayCb);
  glutReshapeFunc(reshapeCb);
  glutIdleFunc(idleCb);
  glutKeyboardFunc(keyboardCb);
  glutMouseFunc(mouseButtonCb);
  glutMotionFunc(mouseMotionCb);

  glutMainLoop();
  return 0;
}
