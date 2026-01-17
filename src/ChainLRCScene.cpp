#include "ChainLRCScene.h"

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

static glm::vec3 jointWorldPos(const std::vector<lrc::RigidBody>& bodies,
                               const std::vector<lrc::JointPointConstraint>& joints,
                               int jointIdx) {
  if (jointIdx < 0 || jointIdx >= (int)joints.size()) return glm::vec3(0.0f);
  const lrc::JointPointConstraint& jc = joints[jointIdx];

  const lrc::RigidBody& A = bodies[jc.a];
  const lrc::RigidBody& B = bodies[jc.b];

  glm::vec3 pA = worldPoint(A, jc.la);
  glm::vec3 pB = worldPoint(B, jc.lb);
  return 0.5f * (pA + pB);
}

ChainLRCScene::ChainLRCScene() {
  titleName_ = "Long Range Constraints - Phase A";
  camPitch_ = 0.6f;
  camDist_ = 4.0f;
}

void ChainLRCScene::usage() const {
  std::printf("\n");
  std::printf("Phase A (MaxDistance LRC)\n");
  std::printf("  Mouse LMB: orbit, RMB: pan, wheel: zoom\n");
  std::printf("  =/-: zoom (keyboard fallback)\n");
  std::printf("  [ / ] : iterations\n");
  std::printf("  L     : toggle LRC\n");
  std::printf("  R     : reset\n");
  std::printf("  ESC   : quit\n");
  std::printf("\n");
}

void ChainLRCScene::reset() {
  buildScene();
  updateWindowTitle();
}

void ChainLRCScene::buildScene() {
  glm::vec3 rootPos(0.0f, 1.2f, 0.0f);
  chain_.buildVerticalChain(numBodies_, boxHalf_, mass_, rootPos);

  for (lrc::JointPointConstraint& jc : chain_.joints()) {
    jc.compliance = jointCompliance_;
    jc.lambda = glm::vec3(0.0f);
  }

  chain_.buildLrcMax(lrcCompliance_);

  if (!chain_.bodies().empty()) {
    rootBasePos_ = chain_.bodies()[0].x;
  }
  rootDriveT_ = 0.0f;

  lastTms_ = 0;
  acc_ = 0.0f;
}

void ChainLRCScene::simulateStep(float dt) {
  // Drive the root body kinematically in X to create a clear swing motion.
  // This is evaluated in simulation time (dt steps), so it stays deterministic.
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

  chain_.step(dt, iters_, useLrc_, false, gravity_);
}

void ChainLRCScene::drawSceneContents() {
  glDisable(GL_LIGHTING);
  glLineWidth(1.0f);

  glColor3f(0.25f, 0.25f, 0.25f);
  drawGround(-0.8f);

  for (int i = 0; i < (int)chain_.bodies().size(); ++i) {
    const lrc::RigidBody& b = chain_.bodies()[i];
    glPushMatrix();
    glm::mat4 T = bodyTransform(b);
    glMultMatrixf(glm::value_ptr(T));
    if (i == 0) glColor3f(0.9f, 0.6f, 0.2f); // root body highlighted in orange.
    else        glColor3f(0.8f, 0.8f, 0.9f);
    drawBoxWire(b.halfExtents);
    glPopMatrix();
  }

  glPointSize(6.0f); // joint points in red.
  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.2f, 0.2f);
  for (int j = 0; j < (int)chain_.joints().size(); ++j) {
    glm::vec3 p = jointWorldPos(chain_.bodies(), chain_.joints(), j);
    glVertex3fv(glm::value_ptr(p));
  }
  glEnd();

  if (useLrc_) { // Draw violated LRC links
    glLineWidth(2.0f);
    glColor3f(0.2f, 0.9f, 0.2f);
    glBegin(GL_LINES);

    glm::vec3 p0 = worldPoint(chain_.bodies()[0], chain_.rootAnchorLocal());
    for (int jointIdx = 1; jointIdx < chain_.numBodies() - 1; ++jointIdx) {
      int bodyB = jointIdx + 1;
      glm::vec3 pj = worldPoint(chain_.bodies()[bodyB], chain_.childAnchorLocal());

      glm::vec3 d = pj - p0;
      float dist = glm::length(d);
      float maxDist = (float)jointIdx * chain_.segmentLen();
      if (dist > maxDist + 1e-4f) {
        glVertex3fv(glm::value_ptr(p0));
        glVertex3fv(glm::value_ptr(pj));
      }
    }
    glEnd();
    glLineWidth(1.0f);
  }
}

void ChainLRCScene::display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  applyCameraTransform();

  drawSceneContents();

  glutSwapBuffers();
}

void ChainLRCScene::keyboard(unsigned char key, int /*x*/, int /*y*/) {
  switch (key) {
    case 'l': case 'L':
      useLrc_ = !useLrc_;
      updateWindowTitle();
      break;
    case '[':
      iters_ = std::max(1, iters_ - 1);
      updateWindowTitle();
      break;
    case ']':
      iters_ = std::min(80, iters_ + 1);
      updateWindowTitle();
      break;
    case 'r': case 'R':
      reset();
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
}
