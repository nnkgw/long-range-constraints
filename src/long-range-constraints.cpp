#include <cstdio>
#include <memory>

#include "GlutCompat.h"
#include "SceneManager.h"
#include "ChainLRCScene.h"
#include "ChainAngleBoundsScene.h"

static SceneManager g_sceneManager;

static void display() {
  IScene* s = g_sceneManager.active();
  if (s) s->display();
}

static void reshape(int w, int h) {
  IScene* s = g_sceneManager.active();
  if (s) s->reshape(w, h);
}

static void idle() {
  IScene* s = g_sceneManager.active();
  if (s) s->idle();
}

static void keyboard(unsigned char key, int x, int y) {
  // Scene switching is handled here so individual scenes can keep their original key maps.
  if (key == '1') {
    g_sceneManager.setActive(0);
    return;
  }
  if (key == '2') {
    g_sceneManager.setActive(1);
    return;
  }

  IScene* s = g_sceneManager.active();
  if (s) s->keyboard(key, x, y);
}

static void mouse(int button, int state, int x, int y) {
  IScene* s = g_sceneManager.active();
  if (s) s->mouse(button, state, x, y);
}

static void motion(int x, int y) {
  IScene* s = g_sceneManager.active();
  if (s) s->motion(x, y);
}

static void initGL() {
  glEnable(GL_DEPTH_TEST);
  glClearColor(0.08f, 0.09f, 0.10f, 1.0f);
}

int main(int argc, char** argv) {
  std::printf("Long Range Constraints for Rigid Body Simulations\n");
  std::printf("  1: Phase A (MaxDistance LRC)\n");
  std::printf("  2: Phase B (Angle->Distance Bounds)\n");

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);

  glutInitWindowSize(1280, 720);
  glutCreateWindow("Long Range Constraints");

  initGL();

  g_sceneManager.add(std::make_unique<ChainLRCScene>());
  g_sceneManager.add(std::make_unique<ChainAngleBoundsScene>());
  g_sceneManager.setActive(0);

  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  glutIdleFunc(idle);

  glutMainLoop();
  return 0;
}
