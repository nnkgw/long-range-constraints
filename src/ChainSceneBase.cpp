#include "ChainSceneBase.h"

#include <algorithm>
#include <cstdio>

#include "GlutCompat.h"

ChainSceneBase::ChainSceneBase() {
}

void ChainSceneBase::reshape(int w, int h) {
  if (h < 1) h = 1;
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, (double)w / (double)h, 0.01, 50.0);
}

void ChainSceneBase::applyCameraTransform() {
  glTranslatef(camPan_.x, camPan_.y, -camDist_);
  glRotatef(camPitch_ * 180.0f / lrc::kPi, 1, 0, 0);
  glRotatef(camYaw_   * 180.0f / lrc::kPi, 0, 1, 0);
}

void ChainSceneBase::idle() {
  int now = glutGet(GLUT_ELAPSED_TIME);
  if (lastTms_ == 0) lastTms_ = now;
  int dtms = now - lastTms_;
  lastTms_ = now;

  acc_ += (float)dtms / 1000.0f;

  float simStart = (float)glutGet(GLUT_ELAPSED_TIME);

  int steps = 0;
  while (acc_ >= dt_ && steps < maxSubStepsPerFrame_) {
    simulateStep(dt_);
    acc_ -= dt_;
    ++steps;
  }

  float simEnd = (float)glutGet(GLUT_ELAPSED_TIME);
  lastSimMs_ = (simEnd - simStart);

  updateWindowTitle();
  glutPostRedisplay();
}

void ChainSceneBase::mouse(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON)  lbtn_ = (state == GLUT_DOWN);
  if (button == GLUT_RIGHT_BUTTON) rbtn_ = (state == GLUT_DOWN);

#if defined(LRC_HAS_FREEGLUT)
  if (button == 3 && state == GLUT_DOWN) camDist_ *= 0.9f;
  if (button == 4 && state == GLUT_DOWN) camDist_ *= 1.1f;
#endif

  lastMouseX_ = x;
  lastMouseY_ = y;
}

void ChainSceneBase::motion(int x, int y) {
  int dx = x - lastMouseX_;
  int dy = y - lastMouseY_;
  lastMouseX_ = x;
  lastMouseY_ = y;

  if (lbtn_) {
    camYaw_ += dx * 0.005f;
    camPitch_ += dy * 0.005f;
    const float lim = 1.5f;
    if (camPitch_ > lim) camPitch_ = lim;
    if (camPitch_ < -lim) camPitch_ = -lim;
  }
  if (rbtn_) {
    float s = 0.002f * camDist_;
    camPan_.x += dx * s;
    camPan_.y -= dy * s;
  }
}

void ChainSceneBase::updateWindowTitle(const char* extra) const {
  char buf[512];
  if (extra && extra[0]) {
    std::snprintf(buf, sizeof(buf),
                  "%s | dt=%.4f it=%d LRC=%s | sim %.2f ms | %s",
                  titleName_.c_str(),
                  dt_,
                  iters_,
                  useLrc_ ? "ON" : "OFF",
                  lastSimMs_,
                  extra);
  } else {
    std::snprintf(buf, sizeof(buf),
                  "%s | dt=%.4f it=%d LRC=%s | sim %.2f ms",
                  titleName_.c_str(),
                  dt_,
                  iters_,
                  useLrc_ ? "ON" : "OFF",
                  lastSimMs_);
  }
  glutSetWindowTitle(buf);
}
