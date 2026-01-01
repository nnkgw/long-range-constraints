#include "ChainAngleBoundsScene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include "GlutCompat.h"
#include "DrawUtils.h"

static glm::mat4 bodyTransform(const lrc::RigidBody& b) {
  glm::mat4 T(1.0f);
  T = glm::translate(T, b.x);
  T *= glm::toMat4(b.q);
  return T;
}

static glm::vec3 worldPoint(const lrc::RigidBody& b, const glm::vec3& localP) {
  return b.x + glm::rotate(b.q, localP);
}

ChainAngleBoundsScene::ChainAngleBoundsScene() {
  titleName_ = "Long Range Constraints - Phase B";
  camPitch_ = 0.6f;
  camDist_ = 4.0f;
}

void ChainAngleBoundsScene::usage() const {
  std::printf("\n");
  std::printf("Phase B (Angle limits -> Distance bounds)\n");
  std::printf("  Mouse LMB: orbit, RMB: pan, wheel: zoom\n");
  std::printf("  [ / ] : iterations\n");
  std::printf("  L     : toggle LRC\n");
  std::printf("  , / . : joint limit (deg)\n");
  std::printf("  R     : reset\n");
  std::printf("  ESC   : quit\n");
  std::printf("\n");
}

void ChainAngleBoundsScene::reset() {
  buildScene();
  updateWindowTitle();
}

void ChainAngleBoundsScene::buildScene() {
  glm::vec3 rootPos(0.0f, 1.2f, 0.0f);
  chain_.buildVerticalChain(numBodies_, boxHalf_, mass_, rootPos);

  for (lrc::JointPointConstraint& jc : chain_.joints()) {
    jc.compliance = jointCompliance_;
    jc.lambda = glm::vec3(0.0f);
  }

  float jointLimitRad = jointLimitDeg_ * (lrc::kPi / 180.0f);
  chain_.buildLrcBounds(jointLimitRad, lrcCompliance_);

  if (!chain_.bodies().empty()) {
    rootBasePos_ = chain_.bodies()[0].x;
  }
  rootDriveT_ = 0.0f;

  lastTms_ = 0;
  acc_ = 0.0f;

  char extra[128];
  std::snprintf(extra, sizeof(extra), "limit=%.1f deg", jointLimitDeg_);
  updateWindowTitle(extra);
}

void ChainAngleBoundsScene::simulateStep(float dt) {
  // Same root drive as Phase A to keep the comparison clear.
  rootDriveT_ += dt;
  if (!chain_.bodies().empty()) {
    const float omega = 2.0f * lrc::kPi * rootDriveHz_;
    const float offX  = rootDriveAmp_ * std::sin(omega * rootDriveT_);
    const float velX  = rootDriveAmp_ * omega * std::cos(omega * rootDriveT_);

    lrc::RigidBody& root = chain_.bodies()[0];
    root.x = rootBasePos_ + glm::vec3(offX, 0.0f, 0.0f);
    root.v = glm::vec3(velX, 0.0f, 0.0f);
    root.w = glm::vec3(0.0f);
    root.q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    root.xPred = root.x;
    root.qPred = root.q;
  }

  chain_.step(dt, iters_, useLrc_, true, gravity_);
}

void ChainAngleBoundsScene::drawSceneContents() {
  glDisable(GL_LIGHTING);
  glLineWidth(1.0f);

  glColor3f(0.25f, 0.25f, 0.25f);
  drawGround(-0.8f);

  for (int i = 0; i < (int)chain_.bodies().size(); ++i) {
    const lrc::RigidBody& b = chain_.bodies()[i];
    glPushMatrix();
    glm::mat4 T = bodyTransform(b);
    glMultMatrixf(glm::value_ptr(T));
    if (i == 0) glColor3f(0.9f, 0.6f, 0.2f); // highlight the kinematic root in orange.
    else        glColor3f(0.8f, 0.8f, 0.9f);
    drawBoxWire(b.halfExtents);
    glPopMatrix();
  }

  if (useLrc_) {
    glLineWidth(2.0f);
    glBegin(GL_LINES);

    glm::vec3 p0 = worldPoint(chain_.bodies()[0], chain_.rootAnchorLocal());
    for (const lrc::DistanceBoundsConstraint& c : chain_.lrcBounds()) {
      const lrc::RigidBody& B = chain_.bodies()[c.b];
      glm::vec3 pj = worldPoint(B, c.lb);

      float dist = glm::length(pj - p0);
      bool violated = (dist < c.minDist - 1e-4f) || (dist > c.maxDist + 1e-4f);

      if (violated) {
        glColor3f(1.0f, 0.3f, 0.2f);
        glVertex3fv(glm::value_ptr(p0));
        glVertex3fv(glm::value_ptr(pj));
      }
    }

    glEnd();
    glLineWidth(1.0f);
  }
}

void ChainAngleBoundsScene::display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  applyCameraTransform();

  drawSceneContents();

  glutSwapBuffers();
}

void ChainAngleBoundsScene::keyboard(unsigned char key, int /*x*/, int /*y*/) {
  bool needRebuild = false;

  switch (key) {
    case 'l': case 'L':
      useLrc_ = !useLrc_;
      break;
    case '[':
      iters_ = std::max(1, iters_ - 1);
      break;
    case ']':
      iters_ = std::min(80, iters_ + 1);
      break;
    case ',':
      jointLimitDeg_ = std::max(0.0f, jointLimitDeg_ - 2.5f);
      needRebuild = true;
      break;
    case '.':
      jointLimitDeg_ = std::min(90.0f, jointLimitDeg_ + 2.5f);
      needRebuild = true;
      break;
    case 'r': case 'R':
      needRebuild = true;
      break;
    case '=':
      camDist_ *= 0.9f;
      if (camDist_ < 0.2f) camDist_ = 0.2f;
      break;
    case '-':
      camDist_ *= 1.1f;
      if (camDist_ > 20.0f) camDist_ = 20.0f;
      break;
    case 27:
#if defined(LRC_HAS_FREEGLUT)
      glutLeaveMainLoop();
#else
      std::exit(0);
#endif
      break;
  }

  if (needRebuild) {
    reset();
  } else {
    char extra[128];
    std::snprintf(extra, sizeof(extra), "limit=%.1f deg", jointLimitDeg_);
    updateWindowTitle(extra);
  }
}
