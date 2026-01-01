#include "ChainFreeScene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

#include "GlutCompat.h"
#include "DrawUtils.h"

namespace {

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

static bool isMaxDistViolated(const std::vector<lrc::RigidBody>& bodies,
                             const lrc::MaxDistanceConstraint& lc,
                             float eps) {
  if (lc.a < 0 || lc.b < 0) return false;
  if (lc.a >= (int)bodies.size() || lc.b >= (int)bodies.size()) return false;
  glm::vec3 pA = worldPoint(bodies[lc.a], lc.la);
  glm::vec3 pB = worldPoint(bodies[lc.b], lc.lb);
  float dist = glm::length(pB - pA);
  return dist > lc.maxDist + eps;
}

} // namespace

ChainFreeScene::ChainFreeScene() {
  titleName_ = "Long Range Constraints - Phase C";
  camPitch_ = 0.6f;
  camDist_ = 4.0f;
}

void ChainFreeScene::usage() const {
  std::printf("\n");
  std::printf("Phase C (Free Chains)\n");
  std::printf("  Mouse LMB: orbit, RMB: pan, wheel: zoom\n");
  std::printf("  [ / ] : iterations\n");
  std::printf("  L     : toggle LRC\n");
  std::printf("  V     : toggle drawing all LRC links\n");
  std::printf("  R     : reset\n");
  std::printf("  ESC   : quit\n");
  std::printf("\n");
}

void ChainFreeScene::reset() {
  buildScene();
  char extra[64];
  std::snprintf(extra, sizeof(extra), "allLRC=%d", drawAllLrc_ ? 1 : 0);
  updateWindowTitle(extra);
}

void ChainFreeScene::buildScene() {
  glm::vec3 rootPos(0.0f, 1.2f, 0.0f);

  // Free chain: the root is not kinematic in Phase C.
  chain_.buildVerticalChain(numBodies_, boxHalf_, mass_, rootPos, false);

  for (lrc::JointPointConstraint& jc : chain_.joints()) {
    jc.compliance = jointCompliance_;
    jc.lambda = glm::vec3(0.0f);
  }

  // Minimal Phase C step: keep Phase A-style MaxDistance LRC, but build links
  // on a free chain (no kinematic root). This is a stepping stone to the paper's
  // Phase C details.
  chain_.buildLrcFreeMaxHierarchy(lrcCompliance_);

  // Add an initial sideways kick so we can clearly see the chain dynamics.
  int nb = chain_.numBodies();
  for (int i = 0; i < nb; ++i) {
    lrc::RigidBody& b = chain_.bodies()[i];
    float s = (i < nb / 2) ? -1.0f : 1.0f;
    b.v = glm::vec3(0.6f * s, 0.0f, 0.0f);
    b.w = glm::vec3(0.0f, 0.0f, 0.8f * s);
    b.xPred = b.x;
    b.qPred = b.q;
  }

  lastTms_ = 0;
  acc_ = 0.0f;
}

void ChainFreeScene::simulateStep(float dt) {
  chain_.step(dt, iters_, useLrc_, false, gravity_);
}

void ChainFreeScene::drawSceneContents() {
  glDisable(GL_LIGHTING);
  glLineWidth(1.0f);

  glColor3f(0.25f, 0.25f, 0.25f);
  drawGround(-0.8f);

  for (int i = 0; i < (int)chain_.bodies().size(); ++i) {
    const lrc::RigidBody& b = chain_.bodies()[i];
    glPushMatrix();
    glm::mat4 T = bodyTransform(b);
    glMultMatrixf(glm::value_ptr(T));

    // Keep the same highlight convention as Phase A: the first body is orange.
    if (i == 0) glColor3f(0.9f, 0.6f, 0.2f);
    else        glColor3f(0.8f, 0.8f, 0.9f);

    drawBoxWire(b.halfExtents);
    glPopMatrix();
  }

  glPointSize(6.0f);
  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.2f, 0.2f);
  for (int j = 0; j < (int)chain_.joints().size(); ++j) {
    glm::vec3 p = jointWorldPos(chain_.bodies(), chain_.joints(), j);
    glVertex3fv(glm::value_ptr(p));
  }
  glEnd();

  if (useLrc_) {
    const float eps = 1e-4f;

    if (drawAllLrc_) {
      // First pass: draw all links faintly.
      glLineWidth(1.0f);
      glBegin(GL_LINES);
      for (const lrc::MaxDistanceConstraint& lc : chain_.lrcMax()) {
        if (lc.a < 0 || lc.b < 0) continue;
        if (lc.a >= (int)chain_.bodies().size() || lc.b >= (int)chain_.bodies().size()) continue;
        glm::vec3 pA = worldPoint(chain_.bodies()[lc.a], lc.la);
        glm::vec3 pB = worldPoint(chain_.bodies()[lc.b], lc.lb);
        glColor3f(0.35f, 0.35f, 0.35f);
        glVertex3fv(glm::value_ptr(pA));
        glVertex3fv(glm::value_ptr(pB));
      }
      glEnd();
    }

    // Second pass: draw violated links in green, thicker.
    glLineWidth(2.0f);
    glColor3f(0.2f, 0.9f, 0.2f);
    glBegin(GL_LINES);
    for (const lrc::MaxDistanceConstraint& lc : chain_.lrcMax()) {
      if (!isMaxDistViolated(chain_.bodies(), lc, eps)) continue;
      glm::vec3 pA = worldPoint(chain_.bodies()[lc.a], lc.la);
      glm::vec3 pB = worldPoint(chain_.bodies()[lc.b], lc.lb);
      glVertex3fv(glm::value_ptr(pA));
      glVertex3fv(glm::value_ptr(pB));
    }
    glEnd();
    glLineWidth(1.0f);
  }
}

void ChainFreeScene::display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  applyCameraTransform();

  drawSceneContents();

  glutSwapBuffers();
}

void ChainFreeScene::keyboard(unsigned char key, int /*x*/, int /*y*/) {
  auto refreshTitle = [this]() {
    char extra[64];
    std::snprintf(extra, sizeof(extra), "allLRC=%d", drawAllLrc_ ? 1 : 0);
    updateWindowTitle(extra);
  };

  switch (key) {
    case 'l': case 'L':
      useLrc_ = !useLrc_;
      refreshTitle();
      break;
    case 'v': case 'V':
      drawAllLrc_ = !drawAllLrc_;
      refreshTitle();
      break;
    case '[':
      iters_ = std::max(1, iters_ - 1);
      refreshTitle();
      break;
    case ']':
      iters_ = std::min(80, iters_ + 1);
      refreshTitle();
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
