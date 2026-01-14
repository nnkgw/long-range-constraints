#include "ContactGraphScene.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <utility>

#include "DrawUtils.h"
#include "GlutCompat.h"

#if defined(__APPLE__)
  #include <OpenGL/gl.h>
  #include <OpenGL/glu.h>
#else
  #include <GL/gl.h>
  #include <GL/glu.h>
#endif

#include <glm/gtc/type_ptr.hpp>

static glm::vec3 aabbMin(const lrc::RigidBody& b) {
  return b.x - b.halfExtents;
}

static glm::vec3 aabbMax(const lrc::RigidBody& b) {
  return b.x + b.halfExtents;
}

ContactGraphScene::ContactGraphScene() {
  titleName_ = "Long Range Constraints - Phase D0";
  camPitch_ = 0.55f;
  camDist_ = 5.0f;
  useLrc_ = false;
  iters_ = 0;
}

void ContactGraphScene::usage() const {
  std::printf("\n");
  std::printf("Phase D1 (Contact graphs - Step0)\n");
  std::printf("  4 : switch to this scene\n");
  std::printf("  G : toggle contact graph edges\n");
  std::printf("  P : toggle contact points\n");
  std::printf("  SPACE : toggle pause\n");
  std::printf("  R : reset scene\n");
  std::printf("  ESC : quit\n");
  std::printf("  - Yellow points: dynamic (sliding) contacts (vt > eps).\n");
}

void ContactGraphScene::buildScene() {
  bodies_.clear();
  contacts_.clear();
  graphEdges_.clear();

  // Build a small stack with a deliberate overhang so both supporting (red) and
  // non-supporting (blue) contacts appear.
  // Body 0: bottom box on the floor.
  lrc::RigidBody b0;
  b0.halfExtents = boxHalf_;
  b0.invMass = 1.0f;
  b0.invInertiaLocal = lrc::boxInvInertiaLocal(1.0f, b0.halfExtents);
  b0.x = glm::vec3(0.0f, groundY_ + boxHalf_.y, 0.0f);
  bodies_.push_back(b0);

  // Body 1: centered on top of body 0.
  lrc::RigidBody b1 = b0;
  b1.x = glm::vec3(0.0f, b0.x.y + 2.0f * boxHalf_.y, 0.0f);
  bodies_.push_back(b1);

  // Body 2: overhanging on body 1 (will become "unsupported" in our test).
  lrc::RigidBody b2 = b0;
  b2.x = glm::vec3(0.25f, b1.x.y + 2.0f * boxHalf_.y, 0.0f);
  bodies_.push_back(b2);

  // Body 3: a side box touching the floor (supported).
  lrc::RigidBody b3 = b0;
  b3.x = glm::vec3(-0.65f, groundY_ + boxHalf_.y, 0.0f);
  bodies_.push_back(b3);

  // Keep orientations identity for this step (AABB contacts).
  for (lrc::RigidBody& b : bodies_) {
    b.q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    b.v = glm::vec3(0.0f);
    b.w = glm::vec3(0.0f);
  }

  camPan_ = glm::vec3(0.0f, 0.0f, 0.0f);
  lastTms_ = 0;
  acc_ = 0.0f;

  rebuildContacts();
}

void ContactGraphScene::reset() {
  buildScene();
  updateWindowTitle();
}

void ContactGraphScene::simulateStep(float /*dt*/) {
  // Step 0 is a static visualization: no dynamics.
  // The pause flag is kept so later steps can share the same UX.
  if (paused_) return;
}

void ContactGraphScene::rebuildContacts() {
  contacts_.clear();

  // Floor contacts.
  for (int i = 0; i < (int)bodies_.size(); ++i) {
    addBoxFloorContacts(i);
  }

  // Box-box contacts (AABB overlap). For this step, we assume identity orientation.
  for (int i = 0; i < (int)bodies_.size(); ++i) {
    for (int j = 0; j < (int)bodies_.size(); ++j) {
      if (i == j) continue;

      // Identify the upper and lower body by y.
      const lrc::RigidBody& A = bodies_[i];
      const lrc::RigidBody& B = bodies_[j];

      if (A.x.y <= B.x.y) continue;

      glm::vec3 aMin = aabbMin(A);
      glm::vec3 aMax = aabbMax(A);
      glm::vec3 bMin = aabbMin(B);
      glm::vec3 bMax = aabbMax(B);

      // Overlap in XZ.
      float ox0 = std::max(aMin.x, bMin.x);
      float ox1 = std::min(aMax.x, bMax.x);
      float oz0 = std::max(aMin.z, bMin.z);
      float oz1 = std::min(aMax.z, bMax.z);
      if (ox0 >= ox1 || oz0 >= oz1) continue;

      // Check vertical adjacency (top of lower near bottom of upper).
      float upperBottom = aMin.y;
      float lowerTop = bMax.y;
      float gap = upperBottom - lowerTop;
      if (std::fabs(gap) > 1e-4f) continue;

      addBoxBoxContacts(i, j);
    }
  }

  buildContactGraph();
  computeSupportingContacts();
}

void ContactGraphScene::buildContactGraph() {
  graphEdges_.clear();
  if (contacts_.empty()) return;

  // For each body, connect all contact nodes that act on it.
  for (int body = -1; body < (int)bodies_.size(); ++body) {
    std::vector<int> idx;
    for (int c = 0; c < (int)contacts_.size(); ++c) {
      const Contact& ct = contacts_[c];
      if (ct.upper == body || ct.lower == body) idx.push_back(c);
    }
    for (int i = 0; i < (int)idx.size(); ++i) {
      for (int j = i + 1; j < (int)idx.size(); ++j) {
        graphEdges_.push_back(std::make_pair(idx[i], idx[j]));
      }
    }
  }
}

void ContactGraphScene::computeSupportingContacts() {
  bodySupported_.assign(bodies_.size(), false);
  for (Contact& c : contacts_) c.supporting = false;

  // Iteratively mark bodies supported from the bottom up.
  // A body is supported if its COM projection lies within the convex hull of the
  // projections of its static contacts to supported "lower" bodies.
  bool changed = true;
  for (int iter = 0; iter < 32 && changed; ++iter) {
    changed = false;
    for (int bi = 0; bi < (int)bodies_.size(); ++bi) {
      if (bodySupported_[bi]) continue;

      std::vector<int> cidx;
      std::vector<glm::vec2> pts;

      for (int ci = 0; ci < (int)contacts_.size(); ++ci) {
        const Contact& c = contacts_[ci];
        if (c.upper != bi) continue;
        if (!c.isStatic) continue;
        if (c.lower >= 0 && !bodySupported_[c.lower]) continue;

        cidx.push_back(ci);
        pts.push_back(glm::vec2(c.p.x, c.p.z));
      }

      if (pts.empty()) continue;

      glm::vec2 comP(bodies_[bi].x.x, bodies_[bi].x.z);

      bool supported = false;
      if (pts.size() == 1) {
        supported = (glm::length(pts[0] - comP) < 1e-4f);
      } else if (pts.size() == 2) {
        supported = pointOnSegment2D(pts[0], pts[1], comP);
      } else {
        std::vector<glm::vec2> hull = convexHull2D(pts);
        if (hull.size() == 2) supported = pointOnSegment2D(hull[0], hull[1], comP);
        else if (hull.size() >= 3) supported = pointInConvexPolygon2D(hull, comP);
      }

      if (supported) {
        bodySupported_[bi] = true;
        changed = true;
        for (int idx : cidx) contacts_[idx].supporting = true;
      }
    }
  }
}

void ContactGraphScene::addBoxFloorContacts(int bodyIdx) {
  const lrc::RigidBody& b = bodies_[bodyIdx];
  glm::vec3 bMin = aabbMin(b);
  if (std::fabs(bMin.y - groundY_) > 1e-4f) return;

  float hx = b.halfExtents.x;
  float hz = b.halfExtents.z;

  glm::vec3 corners[4] = {
    glm::vec3(b.x.x - hx, groundY_, b.x.z - hz),
    glm::vec3(b.x.x + hx, groundY_, b.x.z - hz),
    glm::vec3(b.x.x + hx, groundY_, b.x.z + hz),
    glm::vec3(b.x.x - hx, groundY_, b.x.z + hz)
  };

  for (int i = 0; i < 4; ++i) {
    Contact c;
    c.upper = bodyIdx;
    c.lower = -1;
    c.p = corners[i];
    c.n = glm::vec3(0.0f, 1.0f, 0.0f);
    c.isStatic = isStaticContact(c.upper, c.lower, c.p, c.n);
    contacts_.push_back(c);
  }
}

bool ContactGraphScene::isStaticContact(int upperIdx, int lowerIdx, const glm::vec3& p, const glm::vec3& n) const {
  // Step 1: a very small "static vs dynamic" contact classification.
  // We classify a contact as "static" when the relative tangential velocity at the contact is small.
  // This is intentionally simple and primarily meant to support later steps (friction cone / impulses).
  const float kStaticVelEps = 0.02f;

  if (upperIdx < 0 || upperIdx >= (int)bodies_.size()) return true;

  const lrc::RigidBody& A = bodies_[upperIdx];
  glm::vec3 ra = p - A.x;
  glm::vec3 va = A.v + glm::cross(A.w, ra);

  glm::vec3 vb(0.0f);
  if (lowerIdx >= 0 && lowerIdx < (int)bodies_.size()) {
    const lrc::RigidBody& B = bodies_[lowerIdx];
    glm::vec3 rb = p - B.x;
    vb = B.v + glm::cross(B.w, rb);
  }

  glm::vec3 vrel = va - vb;
  glm::vec3 vt = vrel - glm::dot(vrel, n) * n;
  return glm::length(vt) < kStaticVelEps;
}

void ContactGraphScene::addBoxBoxContacts(int upperIdx, int lowerIdx) {
  const lrc::RigidBody& A = bodies_[upperIdx];
  const lrc::RigidBody& B = bodies_[lowerIdx];

  glm::vec3 aMin = aabbMin(A);
  glm::vec3 aMax = aabbMax(A);
  glm::vec3 bMin = aabbMin(B);
  glm::vec3 bMax = aabbMax(B);

  float ox0 = std::max(aMin.x, bMin.x);
  float ox1 = std::min(aMax.x, bMax.x);
  float oz0 = std::max(aMin.z, bMin.z);
  float oz1 = std::min(aMax.z, bMax.z);

  if (ox0 >= ox1 || oz0 >= oz1) return;

  float y = bMax.y;

  glm::vec3 corners[4] = {
    glm::vec3(ox0, y, oz0),
    glm::vec3(ox1, y, oz0),
    glm::vec3(ox1, y, oz1),
    glm::vec3(ox0, y, oz1)
  };

  for (int i = 0; i < 4; ++i) {
    Contact c;
    c.upper = upperIdx;
    c.lower = lowerIdx;
    c.p = corners[i];
    c.n = glm::vec3(0.0f, 1.0f, 0.0f);
    c.isStatic = isStaticContact(c.upper, c.lower, c.p, c.n);
    contacts_.push_back(c);
  }
}

float ContactGraphScene::cross2(const glm::vec2& a, const glm::vec2& b) {
  return a.x * b.y - a.y * b.x;
}

float ContactGraphScene::cross2(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c) {
  return cross2(b - a, c - a);
}

std::vector<glm::vec2> ContactGraphScene::convexHull2D(std::vector<glm::vec2> pts) {
  if (pts.size() <= 2) return pts;
  std::sort(pts.begin(), pts.end(), [](const glm::vec2& a, const glm::vec2& b) {
    if (a.x != b.x) return a.x < b.x;
    return a.y < b.y;
  });
  pts.erase(std::unique(pts.begin(), pts.end(), [](const glm::vec2& a, const glm::vec2& b) {
    return glm::length(a - b) < 1e-6f;
  }), pts.end());

  if (pts.size() <= 2) return pts;

  std::vector<glm::vec2> lower;
  for (const glm::vec2& p : pts) {
    while (lower.size() >= 2 && cross2(lower[lower.size() - 2], lower[lower.size() - 1], p) <= 0.0f) {
      lower.pop_back();
    }
    lower.push_back(p);
  }

  std::vector<glm::vec2> upper;
  for (int i = (int)pts.size() - 1; i >= 0; --i) {
    const glm::vec2& p = pts[i];
    while (upper.size() >= 2 && cross2(upper[upper.size() - 2], upper[upper.size() - 1], p) <= 0.0f) {
      upper.pop_back();
    }
    upper.push_back(p);
  }

  lower.pop_back();
  upper.pop_back();

  std::vector<glm::vec2> hull = lower;
  hull.insert(hull.end(), upper.begin(), upper.end());

  return hull;
}

bool ContactGraphScene::pointOnSegment2D(const glm::vec2& a, const glm::vec2& b, const glm::vec2& p) {
  glm::vec2 ab = b - a;
  glm::vec2 ap = p - a;
  float t = glm::dot(ap, ab) / std::max(1e-12f, glm::dot(ab, ab));
  if (t < 0.0f) t = 0.0f;
  if (t > 1.0f) t = 1.0f;
  glm::vec2 q = a + t * ab;
  return glm::length(q - p) < 1e-4f;
}

bool ContactGraphScene::pointInConvexPolygon2D(const std::vector<glm::vec2>& poly, const glm::vec2& p) {
  if (poly.size() < 3) return false;
  // Assumes CCW hull without self-intersections.
  bool hasPos = false;
  bool hasNeg = false;
  for (int i = 0; i < (int)poly.size(); ++i) {
    const glm::vec2& a = poly[i];
    const glm::vec2& b = poly[(i + 1) % poly.size()];
    float c = cross2(a, b, p);
    if (c > 1e-6f) hasPos = true;
    if (c < -1e-6f) hasNeg = true;
    if (hasPos && hasNeg) return false;
  }
  return true;
}

void ContactGraphScene::onFrameEnd() {
  // Keep the title stable with a short summary.
  int staticC = 0;
  int dynC = 0;
  int supp = 0;
  for (const Contact& c : contacts_) {
    if (c.isStatic) ++staticC;
    else ++dynC;
    if (c.supporting) ++supp;
  }
  char extra[160];
  std::snprintf(
    extra,
    sizeof(extra),
    "contacts=%d static=%d dyn=%d supporting=%d",
    (int)contacts_.size(),
    staticC,
    dynC,
    supp
  );
  updateWindowTitle(extra);
}

void ContactGraphScene::drawSceneContents() {
  glDisable(GL_LIGHTING);
  glLineWidth(1.0f);

  glColor3f(0.25f, 0.25f, 0.25f);
  drawGround(groundY_);

  // Draw boxes.
  for (int i = 0; i < (int)bodies_.size(); ++i) {
    const lrc::RigidBody& b = bodies_[i];
    glPushMatrix();
    glTranslatef(b.x.x, b.x.y, b.x.z);
    if (i >= 0 && i < (int)bodySupported_.size() && bodySupported_[i]) {
      glColor3f(0.9f, 0.6f, 0.2f); // supported bodies in orange.
    } else {
      glColor3f(0.8f, 0.8f, 0.9f);
    }
    drawBoxWire(b.halfExtents);
    glPopMatrix();
  }

  // Draw contact points.
  if (showContacts_) {
    glPointSize(6.0f);
    glBegin(GL_POINTS);
    for (const Contact& c : contacts_) {
      if (!c.isStatic) glColor3f(0.8f, 0.8f, 0.2f);
      else if (c.supporting) glColor3f(1.0f, 0.2f, 0.2f);
      else glColor3f(0.2f, 0.6f, 1.0f);
      glVertex3fv(glm::value_ptr(c.p));
    }
    glEnd();
  }

  // Draw contact graph edges.
  if (showGraph_) {
    glLineWidth(1.0f);
    glBegin(GL_LINES);
    glColor3f(0.55f, 0.55f, 0.55f);
    for (const std::pair<int, int>& e : graphEdges_) {
      int a = e.first;
      int b = e.second;
      if (a < 0 || b < 0 || a >= (int)contacts_.size() || b >= (int)contacts_.size()) continue;
      glm::vec3 pa = contacts_[a].p;
      glm::vec3 pb = contacts_[b].p;

      // Small lift to reduce z-fighting with the ground.
      pa.y += 1e-3f;
      pb.y += 1e-3f;

      glVertex3fv(glm::value_ptr(pa));
      glVertex3fv(glm::value_ptr(pb));
    }
    glEnd();
  }
}

void ContactGraphScene::display() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  applyCameraTransform();
  drawSceneContents();

  glutSwapBuffers();
}

void ContactGraphScene::keyboard(unsigned char key, int /*x*/, int /*y*/) {
  if (key == 27) std::exit(0);

  if (key == 'r' || key == 'R') {
    reset();
    return;
  }
  if (key == ' ') {
    paused_ = !paused_;
    return;
  }
  if (key == 'g' || key == 'G') {
    showGraph_ = !showGraph_;
    return;
  }
  if (key == 'p' || key == 'P') {
    showContacts_ = !showContacts_;
    return;
  }
}
