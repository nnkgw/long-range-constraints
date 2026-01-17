#include "ChainFreeScene.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>

#include "DrawUtils.h"
#include "GlutCompat.h"

#if defined(_WIN32)
  #include <windows.h>
#endif

#if defined(__APPLE__)
  #include <OpenGL/gl.h>
  #include <OpenGL/glu.h>
#else
  #include <GL/gl.h>
  #include <GL/glu.h>
#endif

#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

static glm::mat4 bodyTransform(const lrc::RigidBody& b) {
  glm::mat4 T(1.0f);
  T = glm::translate(T, b.x);
  T *= glm::toMat4(b.q);
  return T;
}

static glm::vec3 worldPoint(const lrc::RigidBody& b, const glm::vec3& localP) {
  return b.x + glm::rotate(b.q, localP);
}

static glm::vec3 jointWorldPos(const lrc::ChainSystem& chain, int jointIdx) {
  // Joint j is between bodies j and j+1. We visualize it on body (j+1) at childAnchorLocal().
  int bodyB = jointIdx + 1;
  if (bodyB < 0 || bodyB >= (int)chain.bodies().size()) return glm::vec3(0.0f);
  return worldPoint(chain.bodies()[bodyB], chain.childAnchorLocal());
}

ChainFreeScene::ChainFreeScene() {
  titleName_ = "Long Range Constraints - Phase C1";
  camPitch_ = 0.6f;
  camDist_ = 4.0f;
  useLrc_ = true;
}

void ChainFreeScene::usage() const {
  std::printf("\n");
  std::printf("Phase C1 (Free chain + Follow Cam)\n");
  std::printf("  Mouse LMB: orbit, RMB: pan, wheel: zoom (freeglut only)\n");
  std::printf("  =/- : zoom (keyboard)\n");
  std::printf("  3 : switch to this scene\n");
  std::printf("  L : toggle LRC\n");
  std::printf("  V : toggle hierarchy visualization\n");
  std::printf("  , . : prev / next hierarchy level (all = -1)\n");
  std::printf("  [ ] : decrease / increase iterations\n");
  std::printf("  C : toggle follow camera\n");
  std::printf("  SPACE : toggle pause\n");
  std::printf("  R : reset scene\n");
  std::printf("  ESC : quit\n");
}

void ChainFreeScene::buildScene() {
  glm::vec3 rootPos(0.0f, 1.2f, 0.0f);

  // Free chain: root is dynamic (not kinematic).
  chain_.buildVerticalChain(numBodies_, boxHalf_, mass_, rootPos, false);

  for (lrc::JointPointConstraint& jc : chain_.joints()) {
    jc.compliance = jointCompliance_;
    jc.lambda = glm::vec3(0.0f);
  }

  // Phase C: hierarchical max-distance constraints between joint points.
  chain_.buildLrcFreeMaxHierarchy(lrcCompliance_);

  // Give the chain a small initial sideways velocity so the motion is visible immediately.
  for (lrc::RigidBody& b : chain_.bodies()) {
    if (b.invMass == 0.0f) continue;
    b.v += glm::vec3(0.7f, 0.0f, 0.0f);
  }

  lastTms_ = 0;
  acc_ = 0.0f;
}

void ChainFreeScene::reset() {
  buildScene();
  updateWindowTitle();
}

void ChainFreeScene::simulateStep(float dt) {
  if (paused_) return;
  chain_.step(dt, iters_, useLrc_, false, gravity_);
}

void ChainFreeScene::onFrameEnd() {
  if (!followCam_) return;
  if (chain_.bodies().empty()) return;

  // Follow the center of mass in screen space (pan only).
  glm::vec3 c(0.0f);
  float wsum = 0.0f;
  for (const lrc::RigidBody& b : chain_.bodies()) {
    float w = (b.invMass == 0.0f) ? 0.0f : (1.0f / b.invMass);
    c += w * b.x;
    wsum += w;
  }
  if (wsum > 0.0f) c /= wsum;
  else c = chain_.bodies()[0].x;

  // OpenGL view transform applies yaw first then pitch, then translation (camPan_).
  glm::quat qYaw   = glm::angleAxis(camYaw_,   glm::vec3(0.0f, 1.0f, 0.0f));
  glm::quat qPitch = glm::angleAxis(camPitch_, glm::vec3(1.0f, 0.0f, 0.0f));
  glm::vec3 cRot = glm::mat3_cast(qPitch * qYaw) * c;

  camPan_.x = -cRot.x;
  camPan_.y = -cRot.y;
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
    if (i == 0) glColor3f(0.9f, 0.6f, 0.2f); // top body highlighted in orange.
    else        glColor3f(0.8f, 0.8f, 0.9f);
    drawBoxWire(b.halfExtents);
    glPopMatrix();
  }

  glPointSize(6.0f);
  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.2f, 0.2f);
  for (int j = 0; j < (int)chain_.joints().size(); ++j) {
    glm::vec3 p = jointWorldPos(chain_, j);
    glVertex3fv(glm::value_ptr(p));
  }
  glEnd();

  if (showHierarchy_ && useLrc_) {
    const std::vector<lrc::MaxDistanceConstraint>& lrcs = chain_.lrcMax();
    const std::vector<int>& lb = chain_.lrcMaxLevelBegin();
    const std::vector<int>& le = chain_.lrcMaxLevelEnd();
    int nLevels = (int)std::min(lb.size(), le.size());
    if (nLevels > 0 && !lrcs.empty()) {
      glLineWidth(2.0f);
      glBegin(GL_LINES);
      for (int lvl = 0; lvl < nLevels; ++lvl) {
        if (hierarchyLevel_ >= 0 && lvl != hierarchyLevel_) continue;
        int begin = std::max(0, lb[lvl]);
        int end   = std::min((int)lrcs.size(), le[lvl]);

        float t = (nLevels > 1) ? ((float)lvl / (float)(nLevels - 1)) : 0.0f;
        glm::vec3 baseCol(0.2f + 0.6f * (1.0f - t), 0.6f + 0.3f * t, 1.0f - 0.5f * t);

        for (int i = begin; i < end; ++i) {
          const lrc::MaxDistanceConstraint& c = lrcs[i];
          if (c.a < 0 || c.b < 0) continue;
          if (c.a >= (int)chain_.bodies().size() || c.b >= (int)chain_.bodies().size()) continue;

          glm::vec3 pA = worldPoint(chain_.bodies()[c.a], c.la);
          glm::vec3 pB = worldPoint(chain_.bodies()[c.b], c.lb);
          float dist = glm::length(pB - pA);

          if (dist > c.maxDist + 1e-4f) glColor3f(1.0f, 0.15f, 0.15f);
          else glColor3fv(glm::value_ptr(baseCol));

          
// Draw as a "bulged" polyline to reduce overlap between hierarchy levels.
// The bulge direction is chosen perpendicular to both the segment direction and the view direction,
// so the arc tends to expand sideways in screen space.
glm::vec3 seg = pB - pA;
float segLen = glm::length(seg);
if (segLen < 1e-8f) {
  continue;
}
glm::vec3 segDir = seg / segLen;

// Approximate camera forward direction in world space from the simple yaw/pitch camera used in ChainSceneBase.
float cy = std::cos(camYaw_);
float sy = std::sin(camYaw_);
float cp = std::cos(camPitch_);
float sp = std::sin(camPitch_);
glm::vec3 viewDir(-sy * cp, -sp, -cy * cp);

glm::vec3 bulgeDir = glm::cross(segDir, viewDir);
float bLen = glm::length(bulgeDir);
if (bLen < 1e-6f) {
  // Fallback if segment is nearly parallel to the view direction.
  bulgeDir = glm::cross(segDir, glm::vec3(0.0f, 1.0f, 0.0f));
  bLen = glm::length(bulgeDir);
}
if (bLen < 1e-6f) {
  bulgeDir = glm::cross(segDir, glm::vec3(1.0f, 0.0f, 0.0f));
  bLen = glm::length(bulgeDir);
}
if (bLen < 1e-6f) {
  continue;
}
bulgeDir /= bLen;

// Alternate side per level to reduce overlap further.
float side = (lvl & 1) ? 1.0f : -1.0f;
bulgeDir *= side;

// Bulge amount per hierarchy level (debug-only visualization).
float amp = chain_.segmentLen() * (0.25f + 0.18f * (float)lvl);

// Sample a smooth arc using sin(pi * t) envelope.
const int slices = 8;
glm::vec3 prev = pA;
for (int s = 1; s <= slices; ++s) {
  float tSeg = (float)s / (float)slices;
  glm::vec3 cur = glm::mix(pA, pB, tSeg);
  float w = std::sin(lrc::kPi * tSeg);
  cur += bulgeDir * (amp * w);
  glVertex3fv(glm::value_ptr(prev));
  glVertex3fv(glm::value_ptr(cur));
  prev = cur;
}}
      }
      glEnd();
      glLineWidth(1.0f);
    }
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
  switch (key) {
    case ' ':
      paused_ = !paused_;
      break;
    case 'l': case 'L':
      useLrc_ = !useLrc_;
      updateWindowTitle();
      break;
    case 'v': case 'V':
      showHierarchy_ = !showHierarchy_;
      updateWindowTitle();
      break;
    case ',':
      if (!chain_.lrcMaxLevelBegin().empty()) {
        int n = (int)chain_.lrcMaxLevelBegin().size();
        if (hierarchyLevel_ < 0) hierarchyLevel_ = n - 1;
        else hierarchyLevel_ = std::max(-1, hierarchyLevel_ - 1);
      }
      updateWindowTitle();
      break;
    case '.':
      if (!chain_.lrcMaxLevelBegin().empty()) {
        int n = (int)chain_.lrcMaxLevelBegin().size();
        if (hierarchyLevel_ < 0) hierarchyLevel_ = 0;
        else {
          hierarchyLevel_ += 1;
          if (hierarchyLevel_ >= n) hierarchyLevel_ = -1;
        }
      }
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
    case 'c': case 'C':
      followCam_ = !followCam_;
      updateWindowTitle();
      break;
    case 'r': case 'R':
      reset();
      break;
    case '-':
      camDist_ *= 1.1f;
      if (camDist_ > 20.0f) camDist_ = 20.0f;
      break;
    case '=': case '+':
      camDist_ *= 0.9f;
      if (camDist_ < 0.5f) camDist_ = 0.5f;
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
