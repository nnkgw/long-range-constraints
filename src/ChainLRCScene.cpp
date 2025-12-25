#include "ChainLRCScene.h"

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

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/quaternion.hpp>

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <algorithm>

namespace {

static const float kPi = 3.14159265358979323846f;

static glm::quat integrateOrientation(const glm::quat& q, const glm::vec3& wWorld, float dt) {
  glm::quat wq(0.0f, wWorld.x, wWorld.y, wWorld.z);
  glm::quat dq = 0.5f * wq * q;
  glm::quat q2 = q + dq * dt;
  return glm::normalize(q2);
}

static void applySmallRotation(glm::quat& q, const glm::vec3& dthetaWorld) {
  glm::quat dq(0.0f, dthetaWorld.x, dthetaWorld.y, dthetaWorld.z);
  q = glm::normalize(q + 0.5f * dq * q);
}

static glm::vec3 quatToOmegaWorld(const glm::quat& q0, const glm::quat& q1, float dt) {
  glm::quat dq = q1 * glm::inverse(q0);
  if (dq.w < 0.0f) dq = -dq;

  float w = ChainLRCScene::clampf(dq.w, -1.0f, 1.0f);
  float angle = 2.0f * std::acos(w);
  if (angle < 1e-8f) return glm::vec3(0.0f);

  float s = std::sqrt(std::max(0.0f, 1.0f - w * w));
  glm::vec3 axis(dq.x, dq.y, dq.z);
  if (s > 1e-8f) axis /= s;
  return axis * (angle / dt);
}

static glm::mat3 boxInvInertiaLocal(float mass, const glm::vec3& halfExtents) {
  if (mass <= 0.0f) return glm::mat3(0.0f);

  float hx = halfExtents.x;
  float hy = halfExtents.y;
  float hz = halfExtents.z;

  float Ixx = (1.0f / 3.0f) * mass * (hy * hy + hz * hz);
  float Iyy = (1.0f / 3.0f) * mass * (hx * hx + hz * hz);
  float Izz = (1.0f / 3.0f) * mass * (hx * hx + hy * hy);

  glm::mat3 I(0.0f);
  I[0][0] = (Ixx > 0.0f) ? 1.0f / Ixx : 0.0f;
  I[1][1] = (Iyy > 0.0f) ? 1.0f / Iyy : 0.0f;
  I[2][2] = (Izz > 0.0f) ? 1.0f / Izz : 0.0f;
  return I;
}

} // namespace

struct ChainLRCScene::RigidBody {
  glm::vec3 x;
  glm::quat q;

  glm::vec3 v;
  glm::vec3 w;

  glm::vec3 xPred;
  glm::quat qPred;

  float invMass;
  glm::mat3 invInertiaLocal;

  glm::vec3 halfExtents;
};

static glm::mat3 invInertiaWorld(const ChainLRCScene::RigidBody& b) {
  glm::mat3 R = glm::toMat3(b.qPred);
  return R * b.invInertiaLocal * glm::transpose(R);
}

static glm::vec3 worldPoint(const ChainLRCScene::RigidBody& b, const glm::vec3& localP) {
  return b.xPred + glm::rotate(b.qPred, localP);
}

struct ChainLRCScene::JointPointConstraint {
  int a = -1;
  int b = -1;
  glm::vec3 la{0.0f};
  glm::vec3 lb{0.0f};
  glm::vec3 lambda{0.0f};
  float compliance = 0.0f;

  void solve(std::vector<RigidBody>& bodies, float dt) {
    if (a < 0 || b < 0) return;

    const glm::vec3 axes[3] = {
      glm::vec3(1, 0, 0),
      glm::vec3(0, 1, 0),
      glm::vec3(0, 0, 1)
    };

    for (int k = 0; k < 3; ++k) {
      const glm::vec3 axis = axes[k];

      RigidBody& A = bodies[a];
      RigidBody& B = bodies[b];

      glm::vec3 rA = glm::rotate(A.qPred, la);
      glm::vec3 rB = glm::rotate(B.qPred, lb);

      glm::vec3 pA = A.xPred + rA;
      glm::vec3 pB = B.xPred + rB;

      float C = glm::dot(pA - pB, axis);

      glm::vec3 gradAx = axis;
      glm::vec3 gradBx = -axis;

      glm::vec3 gradAq = glm::cross(rA, gradAx);
      glm::vec3 gradBq = glm::cross(rB, gradBx);

      float alpha = compliance / (dt * dt);

      float denom = 0.0f;
      denom += A.invMass * glm::dot(gradAx, gradAx);
      denom += B.invMass * glm::dot(gradBx, gradBx);

      if (A.invMass > 0.0f) {
        glm::mat3 Iw = invInertiaWorld(A);
        denom += glm::dot(gradAq, Iw * gradAq);
      }
      if (B.invMass > 0.0f) {
        glm::mat3 Iw = invInertiaWorld(B);
        denom += glm::dot(gradBq, Iw * gradBq);
      }

      denom += alpha;
      if (denom < 1e-12f) continue;

      float* lamPtr = (k == 0) ? &lambda.x : (k == 1) ? &lambda.y : &lambda.z;
      float& lam = *lamPtr;

      float dlam = -(C + alpha * lam) / denom;
      lam += dlam;

      A.xPred += A.invMass * dlam * gradAx;
      B.xPred += B.invMass * dlam * gradBx;

      if (A.invMass > 0.0f) {
        glm::mat3 Iw = invInertiaWorld(A);
        glm::vec3 dtheta = Iw * (dlam * gradAq);
        applySmallRotation(A.qPred, dtheta);
      }
      if (B.invMass > 0.0f) {
        glm::mat3 Iw = invInertiaWorld(B);
        glm::vec3 dtheta = Iw * (dlam * gradBq);
        applySmallRotation(B.qPred, dtheta);
      }
    }
  }
};

struct ChainLRCScene::MaxDistanceConstraint {
  int a = -1;
  int b = -1;
  glm::vec3 la{0.0f};
  glm::vec3 lb{0.0f};
  float maxDist = 0.0f;
  float lambda = 0.0f;
  float compliance = 0.0f;

  void solve(std::vector<RigidBody>& bodies, float dt) {
    if (a < 0 || b < 0) return;

    RigidBody& A = bodies[a];
    RigidBody& B = bodies[b];

    glm::vec3 rA = glm::rotate(A.qPred, la);
    glm::vec3 rB = glm::rotate(B.qPred, lb);

    glm::vec3 pA = A.xPred + rA;
    glm::vec3 pB = B.xPred + rB;

    glm::vec3 d = pB - pA;
    float dist = glm::length(d);
    if (dist < 1e-8f) return;
    if (dist <= maxDist) return;

    glm::vec3 n = d / dist;
    float C = dist - maxDist;

    glm::vec3 gradAx = -n;
    glm::vec3 gradBx =  n;

    glm::vec3 gradAq = glm::cross(rA, gradAx);
    glm::vec3 gradBq = glm::cross(rB, gradBx);

    float alpha = compliance / (dt * dt);

    float denom = 0.0f;
    denom += A.invMass * glm::dot(gradAx, gradAx);
    denom += B.invMass * glm::dot(gradBx, gradBx);

    if (A.invMass > 0.0f) {
      glm::mat3 Iw = invInertiaWorld(A);
      denom += glm::dot(gradAq, Iw * gradAq);
    }
    if (B.invMass > 0.0f) {
      glm::mat3 Iw = invInertiaWorld(B);
      denom += glm::dot(gradBq, Iw * gradBq);
    }

    denom += alpha;
    if (denom < 1e-12f) return;

    float dlam = -(C + alpha * lambda) / denom;
    lambda += dlam;

    A.xPred += A.invMass * dlam * gradAx;
    B.xPred += B.invMass * dlam * gradBx;

    if (A.invMass > 0.0f) {
      glm::mat3 Iw = invInertiaWorld(A);
      glm::vec3 dtheta = Iw * (dlam * gradAq);
      applySmallRotation(A.qPred, dtheta);
    }
    if (B.invMass > 0.0f) {
      glm::mat3 Iw = invInertiaWorld(B);
      glm::vec3 dtheta = Iw * (dlam * gradBq);
      applySmallRotation(B.qPred, dtheta);
    }
  }
};

ChainLRCScene::ChainLRCScene() {}
ChainLRCScene::~ChainLRCScene() {}

float ChainLRCScene::clampf(float x, float a, float b) {
  return std::max(a, std::min(b, x));
}

void ChainLRCScene::usage() {
  std::puts("=== Long Range Constraints (minimal) Controls ===");
  std::printf("Bodies=%d  dt=%.4f  iters=%d  LRC=%s\n",
              numBodies_, dt_, iters_, useLRC_ ? "ON" : "OFF");

  std::puts("\nMouse:");
  std::puts("  Left-drag   : rotate camera (yaw/pitch)");
  std::puts("  Right-drag  : pan");
  std::puts("  Wheel       : zoom");

  std::puts("\nKeyboard:");
  std::puts("  L           : toggle LRC ON/OFF");
  std::puts("  [ / ]       : decrement / increment iterations");
  std::puts("  R           : reset simulation");
  std::puts("  + / -       : zoom in / out");
  std::puts("  ESC         : quit");
  std::puts("");
}

void ChainLRCScene::updateWindowTitle() {
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "LRC demo | iters=%d | LRC=%s | sim=%.2f ms",
                iters_, useLRC_ ? "ON" : "OFF", lastSimMs_);
  glutSetWindowTitle(buf);
}

void ChainLRCScene::reset() {
  buildChain();
  updateWindowTitle();
}

void ChainLRCScene::buildChain() {
  bodies_.clear();
  joints_.clear();
  lrcs_.clear();

  bodies_.resize(numBodies_);

  float mass = 1.0f;
  float segLen = 2.0f * boxHalf_.y;

  glm::vec3 rootPos(0.0f, 1.2f, 0.0f);

  for (int i = 0; i < numBodies_; ++i) {
    RigidBody b;
    b.halfExtents = boxHalf_;
    b.x = rootPos - glm::vec3(0.0f, (float)i * segLen, 0.0f);
    b.q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);

    b.v = glm::vec3(0.0f);
    b.w = glm::vec3(0.0f);

    b.xPred = b.x;
    b.qPred = b.q;

    if (i == 0) {
      b.invMass = 0.0f;
      b.invInertiaLocal = glm::mat3(0.0f);
    } else {
      b.invMass = 1.0f / mass;
      b.invInertiaLocal = boxInvInertiaLocal(mass, b.halfExtents);
    }

    bodies_[i] = b;
  }

  joints_.reserve(numBodies_ - 1);
  for (int i = 0; i < numBodies_ - 1; ++i) {
    JointPointConstraint jc;
    jc.a = i;
    jc.b = i + 1;
    jc.la = glm::vec3(0.0f, -boxHalf_.y, 0.0f);
    jc.lb = glm::vec3(0.0f,  boxHalf_.y, 0.0f);
    jc.lambda = glm::vec3(0.0f);
    jc.compliance = jointCompliance_;
    joints_.push_back(jc);
  }

  glm::vec3 rootAnchorLocalA = glm::vec3(0.0f, -boxHalf_.y, 0.0f);

  std::vector<float> dMax;
  dMax.resize(numBodies_ - 1);
  dMax[0] = 0.0f;
  for (int i = 1; i < (int)dMax.size(); ++i) {
    dMax[i] = dMax[i - 1] + segLen;
  }

  lrcs_.reserve(std::max(0, numBodies_ - 2));
  for (int jointIdx = 1; jointIdx < numBodies_ - 1; ++jointIdx) {
    int bodyB = jointIdx + 1;

    MaxDistanceConstraint lc;
    lc.a = 0;
    lc.b = bodyB;
    lc.la = rootAnchorLocalA;
    lc.lb = glm::vec3(0.0f, boxHalf_.y, 0.0f);
    lc.maxDist = dMax[jointIdx];
    lc.lambda = 0.0f;
    lc.compliance = lrcCompliance_;
    lrcs_.push_back(lc);
  }

  // Cache the kinematic root base pose for the periodic drive.
  if (!bodies_.empty()) {
    rootBasePos_ = bodies_[0].x;
  }
  rootDriveT_ = 0.0f;

  // Reset idle accumulator (keeps post-reset behavior stable).
  lastTms_ = 0;
  acc_ = 0.0f;
}

void ChainLRCScene::simulateStep(float dt) {
  // Drive the root body kinematically in X to create a clear swing motion.
  // This is evaluated in simulation time (dt steps), so it stays deterministic.
  rootDriveT_ += dt;
  if (!bodies_.empty()) {
    const float omega = 2.0f * kPi * rootDriveHz_;
    const float offX  = rootDriveAmp_ * std::sin(omega * rootDriveT_);
    const float velX  = rootDriveAmp_ * omega * std::cos(omega * rootDriveT_);

    RigidBody& root = bodies_[0];
    root.x = rootBasePos_ + glm::vec3(offX, 0.0f, 0.0f);
    root.v = glm::vec3(velX, 0.0f, 0.0f);
    root.w = glm::vec3(0.0f);
    root.q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    root.xPred = root.x;
    root.qPred = root.q;
  }

  for (RigidBody& b : bodies_) {
    if (b.invMass == 0.0f) {
      b.xPred = b.x;
      b.qPred = b.q;
      continue;
    }
    b.v += dt * gravity_;
    b.xPred = b.x + dt * b.v;
    b.qPred = integrateOrientation(b.q, b.w, dt);
  }

  for (int it = 0; it < iters_; ++it) {
    for (JointPointConstraint& jc : joints_) {
      jc.solve(bodies_, dt);
    }
    if (useLRC_) {
      for (MaxDistanceConstraint& lc : lrcs_) {
        lc.solve(bodies_, dt);
      }
    }
  }

  for (RigidBody& b : bodies_) {
    if (b.invMass == 0.0f) continue;
    b.v = (b.xPred - b.x) / dt;
    b.w = quatToOmegaWorld(b.q, b.qPred, dt);
    b.x = b.xPred;
    b.q = b.qPred;
  }
}

static glm::vec3 jointWorldPos(const std::vector<ChainLRCScene::RigidBody>& bodies,
                               const std::vector<ChainLRCScene::JointPointConstraint>& joints,
                               int jointIdx) {
  if (jointIdx < 0 || jointIdx >= (int)joints.size()) return glm::vec3(0.0f);
  const ChainLRCScene::JointPointConstraint& jc = joints[jointIdx];

  const ChainLRCScene::RigidBody& A = bodies[jc.a];
  const ChainLRCScene::RigidBody& B = bodies[jc.b];

  glm::vec3 pA = worldPoint(A, jc.la);
  glm::vec3 pB = worldPoint(B, jc.lb);
  return 0.5f * (pA + pB);
}

void ChainLRCScene::render() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glTranslatef(camPan_.x, camPan_.y, -camDist_);
  glRotatef(camPitch_ * 180.0f / kPi, 1, 0, 0);
  glRotatef(camYaw_   * 180.0f / kPi, 0, 1, 0);

  glDisable(GL_LIGHTING);
  glLineWidth(1.0f);

  glColor3f(0.25f, 0.25f, 0.25f);
  drawGround(-0.8f);

  for (int i = 0; i < (int)bodies_.size(); ++i) {
    const RigidBody& b = bodies_[i];
    glPushMatrix();

    glm::mat4 T(1.0f);
    T = glm::translate(T, b.x);
    T *= glm::toMat4(b.q);

    glMultMatrixf(glm::value_ptr(T));

    if (i == 0) glColor3f(0.9f, 0.6f, 0.2f);
    else        glColor3f(0.8f, 0.8f, 0.9f);

    drawBoxWire(b.halfExtents.x, b.halfExtents.y, b.halfExtents.z);
    glPopMatrix();
  }

  glPointSize(6.0f);
  glBegin(GL_POINTS);
  glColor3f(1.0f, 0.2f, 0.2f);
  for (int j = 0; j < (int)joints_.size(); ++j) {
    glm::vec3 p = jointWorldPos(bodies_, joints_, j);
    glVertex3fv(glm::value_ptr(p));
  }
  glEnd();

  if (useLRC_) {
    glm::vec3 p0 = jointWorldPos(bodies_, joints_, 0);
    glLineWidth(2.0f);
    glBegin(GL_LINES);
    glColor3f(0.2f, 0.9f, 0.2f);
    for (int j = 1; j < (int)joints_.size(); ++j) {
      glm::vec3 pj = jointWorldPos(bodies_, joints_, j);
      glm::vec3 d = pj - p0;
      float dist = glm::length(d);
      float maxDist = (float)j * (2.0f * boxHalf_.y);
      if (dist > maxDist + 1e-4f) {
        glVertex3fv(glm::value_ptr(p0));
        glVertex3fv(glm::value_ptr(pj));
      }
    }
    glEnd();
    glLineWidth(1.0f);
  }

  glutSwapBuffers();
}

void ChainLRCScene::idle() {
  int now = glutGet(GLUT_ELAPSED_TIME);
  if (lastTms_ == 0) lastTms_ = now;
  int dtms = now - lastTms_;
  lastTms_ = now;

  acc_ += (float)dtms / 1000.0f;

  float simStart = (float)glutGet(GLUT_ELAPSED_TIME);

  int steps = 0;
  while (acc_ >= dt_ && steps < 4) {
    simulateStep(dt_);
    acc_ -= dt_;
    ++steps;
  }

  float simEnd = (float)glutGet(GLUT_ELAPSED_TIME);
  lastSimMs_ = (simEnd - simStart);

  updateWindowTitle();
  glutPostRedisplay();
}

void ChainLRCScene::reshape(int w, int h) {
  if (h < 1) h = 1;
  glViewport(0, 0, w, h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(45.0, (double)w / (double)h, 0.01, 50.0);
}

void ChainLRCScene::onMouseButton(int button, int state, int x, int y) {
  if (button == GLUT_LEFT_BUTTON)  lbtn_ = (state == GLUT_DOWN);
  if (button == GLUT_RIGHT_BUTTON) rbtn_ = (state == GLUT_DOWN);

#if defined(FREEGLUT)
  if (button == 3 && state == GLUT_DOWN) camDist_ *= 0.9f;
  if (button == 4 && state == GLUT_DOWN) camDist_ *= 1.1f;
#endif

  lastMouseX_ = x;
  lastMouseY_ = y;
}

void ChainLRCScene::onMouseMotion(int x, int y) {
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

void ChainLRCScene::onKeyboard(unsigned char key, int /*x*/, int /*y*/) {
  switch (key) {
    case 'l': case 'L':
      useLRC_ = !useLRC_;
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
    case '+':
      camDist_ *= 0.9f;
      if (camDist_ < 0.5f) camDist_ = 0.5f;
      break;
    case '-':
      camDist_ *= 1.1f;
      if (camDist_ > 20.0f) camDist_ = 20.0f;
      break;
    case 27:
#if defined(FREEGLUT)
      glutLeaveMainLoop();
#else
      std::exit(0);
#endif
      break;
  }
}

void ChainLRCScene::drawGround(float y) {
  glBegin(GL_LINES);
  for (int i = -20; i <= 20; ++i) {
    glVertex3f((float)i * 0.1f, y, -2.0f);
    glVertex3f((float)i * 0.1f, y,  2.0f);
    glVertex3f(-2.0f, y, (float)i * 0.1f);
    glVertex3f( 2.0f, y, (float)i * 0.1f);
  }
  glEnd();
}

void ChainLRCScene::drawBoxWire(float hx, float hy, float hz) {
  glm::vec3 v[8] = {
    glm::vec3(-hx, -hy, -hz),
    glm::vec3( hx, -hy, -hz),
    glm::vec3( hx,  hy, -hz),
    glm::vec3(-hx,  hy, -hz),
    glm::vec3(-hx, -hy,  hz),
    glm::vec3( hx, -hy,  hz),
    glm::vec3( hx,  hy,  hz),
    glm::vec3(-hx,  hy,  hz)
  };

  int e[12][2] = {
    {0,1},{1,2},{2,3},{3,0},
    {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7}
  };

  glBegin(GL_LINES);
  for (int i = 0; i < 12; ++i) {
    glVertex3fv(glm::value_ptr(v[e[i][0]]));
    glVertex3fv(glm::value_ptr(v[e[i][1]]));
  }
  glEnd();
}
