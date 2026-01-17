#include "ChainAngleBoundsScene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

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

ChainAngleBoundsScene::ChainAngleBoundsScene() {
  titleName_ = "Long Range Constraints - Phase B";
  camPitch_ = 0.6f;
  camDist_ = 4.0f;
}

void ChainAngleBoundsScene::usage() const {
  std::printf("\n");
  std::printf("Phase B (Angle limits -> Distance bounds)\n");
  std::printf("  Mouse LMB: orbit, RMB: pan, wheel: zoom\n");
  std::printf("  =/-: zoom (keyboard fallback)\n");
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

  glPointSize(6.0f); // joint points in red.
  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.2f, 0.2f);
  for (int j = 0; j < (int)chain_.joints().size(); ++j) {
    glm::vec3 p = jointWorldPos(chain_.bodies(), chain_.joints(), j);
    glVertex3fv(glm::value_ptr(p));
  }
  glEnd();

  // Visualize the angle limit at each internal joint.
  // Red wedge: +/- jointLimitDeg_ around the parent segment direction.
  // Thin arc: current bend angle (white if within limit, red if violated).
  {
    float limitRad = jointLimitDeg_ * (lrc::kPi / 180.0f);
    float r = chain_.segmentLen() * 0.35f;

    std::vector<glm::vec3> jp;
    jp.reserve(chain_.joints().size());
    for (int j = 0; j < (int)chain_.joints().size(); ++j) {
      jp.push_back(jointWorldPos(chain_.bodies(), chain_.joints(), j));
    }

    glLineWidth(2.0f);
    glColor3f(1.0f, 0.2f, 0.2f);

    for (int j = 1; j + 1 < (int)jp.size(); ++j) {
      glm::vec3 d0 = jp[j] - jp[j - 1];
      glm::vec3 d1 = jp[j + 1] - jp[j];
      float l0 = glm::length(d0);
      float l1 = glm::length(d1);
      if (l0 < 1e-6f || l1 < 1e-6f) continue;
      d0 /= l0;
      d1 /= l1;

      glm::vec3 n = glm::cross(d0, d1);
      float nl = glm::length(n);
      if (nl < 1e-6f) {
        n = glm::cross(d0, glm::vec3(0.0f, 1.0f, 0.0f));
        nl = glm::length(n);
        if (nl < 1e-6f) {
          n = glm::cross(d0, glm::vec3(1.0f, 0.0f, 0.0f));
          nl = glm::length(n);
        }
      }
      if (nl < 1e-6f) continue;
      n /= nl;

      glm::vec3 v = glm::cross(n, d0);
      float vl = glm::length(v);
      if (vl < 1e-6f) continue;
      v /= vl;

      glm::vec3 dirP = d0 * std::cos(limitRad) + v * std::sin(limitRad);
      glm::vec3 dirM = d0 * std::cos(limitRad) - v * std::sin(limitRad);

      glBegin(GL_LINES);
      glVertex3fv(glm::value_ptr(jp[j]));
      glVertex3fv(glm::value_ptr(jp[j] + r * dirP));
      glVertex3fv(glm::value_ptr(jp[j]));
      glVertex3fv(glm::value_ptr(jp[j] + r * dirM));
      glEnd();

      glBegin(GL_LINE_STRIP);
      const int kSeg = 20;
      for (int k = 0; k <= kSeg; ++k) {
        float t = -limitRad + (2.0f * limitRad) * (float)k / (float)kSeg;
        glm::vec3 dir = d0 * std::cos(t) + v * std::sin(t);
        glm::vec3 q = jp[j] + r * dir;
        glVertex3fv(glm::value_ptr(q));
      }
      glEnd();

      float ang = std::atan2(glm::dot(d1, v), glm::dot(d1, d0));
      float absAng = std::fabs(ang);

      float rr = r * 0.82f;
      glLineWidth(1.0f);
      if (absAng > limitRad + 1e-4f) glColor3f(1.0f, 0.2f, 0.2f);
      else                           glColor3f(0.95f, 0.95f, 0.95f);

      glBegin(GL_LINE_STRIP);
      const int kSeg2 = 16;
      for (int k = 0; k <= kSeg2; ++k) {
        float t = ang * (float)k / (float)kSeg2;
        glm::vec3 dir = d0 * std::cos(t) + v * std::sin(t);
        glm::vec3 q = jp[j] + rr * dir;
        glVertex3fv(glm::value_ptr(q));
      }
      glEnd();

      glLineWidth(2.0f);
      glColor3f(1.0f, 0.2f, 0.2f);
    }

    glLineWidth(1.0f);
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
        glColor3f(0.2f, 0.9f, 0.2f);
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

  // Keep the joint limit visible even if ChainSceneBase::idle() overwrites the title.
  char extra[128];
  std::snprintf(extra, sizeof(extra), "limit=%.1f deg", jointLimitDeg_);
  updateWindowTitle(extra);

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
