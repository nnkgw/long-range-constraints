// NOTE: Keep this file ASCII-only to avoid MSVC C4819 warnings on some code pages.
#include "ContactGraphScene.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <utility>
#include <limits>

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
#include <glm/gtx/quaternion.hpp>

namespace {
static float clampf(float x, float lo, float hi) {
  return (std::max)(lo, (std::min)(hi, x));
}

static glm::vec3 obbSupportPointWorld(const lrc::RigidBody& b, const glm::vec3& dirW) {
  const glm::mat3 R = glm::mat3_cast(b.q);
  const glm::vec3 A[3] = { glm::vec3(R[0]), glm::vec3(R[1]), glm::vec3(R[2]) };
  const float e[3] = { b.halfExtents.x, b.halfExtents.y, b.halfExtents.z };
  glm::vec3 p = b.x;
  for (int i = 0; i < 3; ++i) {
    const float s = (glm::dot(A[i], dirW) >= 0.0f) ? 1.0f : -1.0f;
    p += s * e[i] * A[i];
  }
  return p;
}

static bool obbAabbMtv(
  const lrc::RigidBody& obb,
  const lrc::RigidBody& aabb,
  glm::vec3* outN,
  float* outPen) {
  const glm::mat3 R = glm::mat3_cast(obb.q);
  const glm::vec3 A[3] = { glm::vec3(R[0]), glm::vec3(R[1]), glm::vec3(R[2]) };
  const glm::vec3 B[3] = { glm::vec3(1, 0, 0), glm::vec3(0, 1, 0), glm::vec3(0, 0, 1) };

  const float a[3] = { obb.halfExtents.x, obb.halfExtents.y, obb.halfExtents.z };
  const float b[3] = { aabb.halfExtents.x, aabb.halfExtents.y, aabb.halfExtents.z };

  // Rm[i][j] = dot(A[i], B[j]) == A[i][j] since B is the world basis.
  float Rm[3][3];
  float absR[3][3];
  for (int i = 0; i < 3; ++i) {
    Rm[i][0] = A[i].x;
    Rm[i][1] = A[i].y;
    Rm[i][2] = A[i].z;
    for (int j = 0; j < 3; ++j) {
      absR[i][j] = std::fabs(Rm[i][j]) + 1e-6f; // epsilon against parallel axes.
    }
  }

  const glm::vec3 tW = aabb.x - obb.x;
  const float tA[3] = { glm::dot(tW, A[0]), glm::dot(tW, A[1]), glm::dot(tW, A[2]) };
  const float tB[3] = { tW.x, tW.y, tW.z };

  float bestPen = std::numeric_limits<float>::infinity();
  glm::vec3 bestN(0);

  auto tryAxis = [&](const glm::vec3& axisUnit, float proj, float ra, float rb) -> bool {
    const float pen = (ra + rb) - std::fabs(proj);
    if (pen < 0.0f) return false; // separating axis
    if (pen < bestPen) {
      bestPen = pen;
      // Move OBB away from AABB (translate OBB by bestN * bestPen).
      bestN = (proj < 0.0f) ? axisUnit : -axisUnit;
    }
    return true;
  };

  // Axes A0, A1, A2 (OBB local axes).
  for (int i = 0; i < 3; ++i) {
    const float ra = a[i];
    const float rb = b[0] * absR[i][0] + b[1] * absR[i][1] + b[2] * absR[i][2];
    if (!tryAxis(A[i], tA[i], ra, rb)) return false;
  }

  // Axes B0, B1, B2 (world axes / AABB axes).
  for (int j = 0; j < 3; ++j) {
    const float ra = a[0] * absR[0][j] + a[1] * absR[1][j] + a[2] * absR[2][j];
    const float rb = b[j];
    if (!tryAxis(B[j], tB[j], ra, rb)) return false;
  }

  // Axes Ai x Bj (9 cross-product axes).
  for (int i = 0; i < 3; ++i) {
    const int i1 = (i + 1) % 3;
    const int i2 = (i + 2) % 3;
    for (int j = 0; j < 3; ++j) {
      const int j1 = (j + 1) % 3;
      const int j2 = (j + 2) % 3;
      const glm::vec3 L = glm::cross(A[i], B[j]);
      const float len2 = glm::dot(L, L);
      if (len2 < 1e-12f) continue; // nearly parallel => skip
      const glm::vec3 axisUnit = L * (1.0f / std::sqrt(len2));

      const float ra = a[i1] * absR[i2][j] + a[i2] * absR[i1][j];
      const float rb = b[j1] * absR[i][j2] + b[j2] * absR[i][j1];
      const float proj = tA[i2] * Rm[i1][j] - tA[i1] * Rm[i2][j];
      if (!tryAxis(axisUnit, proj, ra, rb)) return false;
    }
  }

  if (!std::isfinite(bestPen)) return false;
  if (outN) *outN = bestN;
  if (outPen) *outPen = bestPen;
  return true;
}

static glm::vec2 closestPointOnSegment2D(const glm::vec2& a, const glm::vec2& b, const glm::vec2& p) {
  const glm::vec2 ab = b - a;
  const float denom = glm::dot(ab, ab);
  float t = 0.0f;
  if (denom > 0.0f) {
    t = glm::dot(p - a, ab) / denom;
  }
  t = clampf(t, 0.0f, 1.0f);
  return a + t * ab;
}

static glm::vec2 closestPointOnPolyBoundary2D(const std::vector<glm::vec2>& poly, const glm::vec2& p) {
  if (poly.empty()) return p;
  if (poly.size() == 1) return poly[0];

  glm::vec2 best = poly[0];
  float bestD2 = std::numeric_limits<float>::infinity();
  const int n = (int)poly.size();
  for (int i = 0; i < n; ++i) {
    const glm::vec2 a = poly[i];
    const glm::vec2 b = poly[(i + 1) % n];
    const glm::vec2 q = closestPointOnSegment2D(a, b, p);
    const glm::vec2 d = q - p;
    const float d2 = glm::dot(d, d);
    if (d2 < bestD2) {
      bestD2 = d2;
      best = q;
    }
  }
  return best;
}
} // namespace

static glm::vec3 worldFromLocal(const lrc::RigidBody& b, const glm::vec3& pLocal) {
  // Transform a point from the body local frame to world frame.
  return b.x + (b.q * pLocal);
}

static glm::vec3 aabbExtentsWorld(const lrc::RigidBody& b) {
  // World-space AABB half extents of an OBB (b.x, b.q, b.halfExtents).
  // ext = |R| * h
  const glm::mat3 R = glm::toMat3(b.q);
  const glm::vec3 h = b.halfExtents;
  return glm::vec3(
    std::abs(R[0][0]) * h.x + std::abs(R[1][0]) * h.y + std::abs(R[2][0]) * h.z,
    std::abs(R[0][1]) * h.x + std::abs(R[1][1]) * h.y + std::abs(R[2][1]) * h.z,
    std::abs(R[0][2]) * h.x + std::abs(R[1][2]) * h.y + std::abs(R[2][2]) * h.z
  );
}

static glm::vec3 aabbMin(const lrc::RigidBody& b) {
  return b.x - aabbExtentsWorld(b);
}

static glm::vec3 aabbMax(const lrc::RigidBody& b) {
  return b.x + aabbExtentsWorld(b);
}

static void applySmallRotation(glm::quat& q, const glm::vec3& dthetaWorld) {
  // Small-angle approximation: q <- normalize( q + 0.5 * [0, dtheta] * q )
  glm::quat dq(0.0f, dthetaWorld.x, dthetaWorld.y, dthetaWorld.z);
  q += 0.5f * dq * q;
  q = glm::normalize(q);
}

static glm::mat3 invInertiaWorld(const lrc::RigidBody& b) {
  const glm::mat3 R = glm::toMat3(b.q);
  return R * b.invInertiaLocal * glm::transpose(R);
}

static void solvePointPenetration(lrc::RigidBody& b,
                                  const glm::vec3& pWorld,
                                  const glm::vec3& nWorld,
                                  float penetration) {
  // Unilateral point-plane / point-AABB-face style positional correction.
  // Treat the other object as static (infinite mass).
  if (penetration <= 0.0f) return;

  const glm::vec3 r = pWorld - b.x;
  const glm::vec3 rn = glm::cross(r, nWorld);
  const glm::mat3 IinvW = invInertiaWorld(b);

  const float denom = b.invMass + glm::dot(rn, IinvW * rn);
  if (denom <= 0.0f) return;

  const float lambda = penetration / denom;

  b.x += b.invMass * lambda * nWorld;
  const glm::vec3 dtheta = IinvW * (lambda * rn);
  applySmallRotation(b.q, dtheta);
}

static void obbCornersWorld(const lrc::RigidBody& b, glm::vec3 corners[8]) {
  const glm::vec3 h = b.halfExtents;
  int idx = 0;
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        const glm::vec3 local((float)sx * h.x, (float)sy * h.y, (float)sz * h.z);
        corners[idx++] = b.x + glm::rotate(b.q, local);
      }
    }
  }
}


static void aabbCornersWorld(const lrc::RigidBody& b, glm::vec3 corners[8]) {
  const glm::vec3 h = b.halfExtents;
  int i = 0;
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        corners[i++] = b.x + glm::vec3((float)sx * h.x, (float)sy * h.y, (float)sz * h.z);
      }
    }
  }
}

static constexpr int kObbSamplePointCount = 26;

static void obbSamplePointsWorld(const lrc::RigidBody& b, glm::vec3 out[kObbSamplePointCount]) {
  const glm::mat3 R = glm::toMat3(b.q);
  const glm::vec3 h = b.halfExtents;

  int idx = 0;
  auto emitLocal = [&](const glm::vec3& pLocal) {
    out[idx++] = b.x + R * pLocal;
  };

  // 8 corners
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      for (int sz = -1; sz <= 1; sz += 2) {
        emitLocal(glm::vec3((float)sx * h.x, (float)sy * h.y, (float)sz * h.z));
      }
    }
  }

  // 12 edge midpoints
  // X edges: x=0, y= +-h.y, z= +-h.z
  for (int sy = -1; sy <= 1; sy += 2) {
    for (int sz = -1; sz <= 1; sz += 2) {
      emitLocal(glm::vec3(0.0f, (float)sy * h.y, (float)sz * h.z));
    }
  }
  // Y edges: y=0, x=+-h.x, z=+-h.z
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sz = -1; sz <= 1; sz += 2) {
      emitLocal(glm::vec3((float)sx * h.x, 0.0f, (float)sz * h.z));
    }
  }
  // Z edges: z=0, x=+-h.x, y=+-h.y
  for (int sx = -1; sx <= 1; sx += 2) {
    for (int sy = -1; sy <= 1; sy += 2) {
      emitLocal(glm::vec3((float)sx * h.x, (float)sy * h.y, 0.0f));
    }
  }

  // 6 face centers
  emitLocal(glm::vec3(+h.x, 0.0f, 0.0f));
  emitLocal(glm::vec3(-h.x, 0.0f, 0.0f));
  emitLocal(glm::vec3(0.0f, +h.y, 0.0f));
  emitLocal(glm::vec3(0.0f, -h.y, 0.0f));
  emitLocal(glm::vec3(0.0f, 0.0f, +h.z));
  emitLocal(glm::vec3(0.0f, 0.0f, -h.z));

  // Safety: keep the list size stable.
  if (idx != kObbSamplePointCount) {
    // Do nothing; idx mismatch indicates a logic error in sampling.
  }
}

static bool pointInsideObb(const lrc::RigidBody& b,
                           const glm::vec3& pWorld,
                           glm::vec3* outNWorld,
                           float* outPenetration) {
  const glm::mat3 R = glm::toMat3(b.q);
  const glm::vec3 pLocal = glm::transpose(R) * (pWorld - b.x);
  const glm::vec3 h = b.halfExtents;

  if (std::fabs(pLocal.x) > h.x || std::fabs(pLocal.y) > h.y || std::fabs(pLocal.z) > h.z) {
    return false;
  }

  const float dx = h.x - std::fabs(pLocal.x);
  const float dy = h.y - std::fabs(pLocal.y);
  const float dz = h.z - std::fabs(pLocal.z);

  float best = dx;
  glm::vec3 nLocal((pLocal.x >= 0.0f) ? 1.0f : -1.0f, 0.0f, 0.0f);

  if (dy < best) {
    best = dy;
    nLocal = glm::vec3(0.0f, (pLocal.y >= 0.0f) ? 1.0f : -1.0f, 0.0f);
  }
  if (dz < best) {
    best = dz;
    nLocal = glm::vec3(0.0f, 0.0f, (pLocal.z >= 0.0f) ? 1.0f : -1.0f);
  }

  if (outNWorld) *outNWorld = R * nLocal;
  if (outPenetration) *outPenetration = best;
  return true;
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
  std::printf("  T : toggle top drive\n");
  std::printf("  [ / ] : adjust drive amplitude\n");
  std::printf("  ; / ' : adjust drive frequency\n");
  std::printf("  F : toggle falling when unsupported\n");
  std::printf("  M : cycle fall rotation mode (A: kick, B: torque)\n");
  std::printf("  ESC : quit\n");
  std::printf("  - Yellow points: dynamic (sliding) contacts (vt > eps).\n");
  std::printf("  - A small horizontal drive moves the top body to make dyn contacts visible.");
}

void ContactGraphScene::buildScene() {
  bodies_.clear();
  contacts_.clear();
  graphEdges_.clear();

  driveTime_ = 0.0f;
  topFreeFall_ = false;
  topOnFloor_ = false;
  fallHasPivot_ = false;
  fallSupportLower_ = -2;
  fallPivot_ = glm::vec3(0.0f);
  fallAxis_ = glm::vec3(0.0f, 0.0f, 1.0f);

  // Build a small blocky stack (Figure 12-style) that stays in view and
  // generates a richer contact graph (several bodies have multiple supporting contacts).
  //
  // Index convention:
  //  - bodies_[2] is the top body and is driven horizontally (dyn contact visualization).

  float y0 = groundY_ + boxHalf_.y;
  float y1 = y0 + 2.0f * boxHalf_.y;
  float y2 = y1 + 2.0f * boxHalf_.y;

  // Body 0: bottom center.
  lrc::RigidBody b0;
  b0.halfExtents = boxHalf_;
  b0.invMass = 1.0f;
  b0.invInertiaLocal = lrc::boxInvInertiaLocal(1.0f, b0.halfExtents);
  b0.x = glm::vec3(0.0f, y0, 0.0f);
  bodies_.push_back(b0);

  // Body 1: middle center.
  lrc::RigidBody b1 = b0;
  b1.x = glm::vec3(0.0f, y1, 0.0f);
  bodies_.push_back(b1);

  // Body 2: top (slightly offset to avoid perfect symmetry).
  lrc::RigidBody b2 = b0;
  b2.x = glm::vec3(0.12f, y2, 0.0f);
  bodies_.push_back(b2);

  // Additional blocks to form a "stacked bricks" layout.
  // Bottom layer (left / right).
  lrc::RigidBody b3 = b0;
  b3.x = glm::vec3(-2.0f * boxHalf_.x, y0, 0.0f);
  bodies_.push_back(b3);

  lrc::RigidBody b4 = b0;
  b4.x = glm::vec3(2.0f * boxHalf_.x, y0, 0.0f);
  bodies_.push_back(b4);

  // Middle layer bridges (each overlaps two bottom blocks).
  lrc::RigidBody b5 = b0;
  b5.x = glm::vec3(-boxHalf_.x, y1, 0.0f);
  bodies_.push_back(b5);

  lrc::RigidBody b6 = b0;
  b6.x = glm::vec3(boxHalf_.x, y1, 0.0f);
  bodies_.push_back(b6);

  // Keep orientations identity for this step (AABB contacts).
  for (lrc::RigidBody& b : bodies_) {
    b.q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    b.v = glm::vec3(0.0f);
    b.w = glm::vec3(0.0f);
  }

  // Remember the initial top-body pose for the kinematic drive.
  if (bodies_.size() > 2) {
    driveBasePos_ = bodies_[2].x;
    bodies_[2].v = glm::vec3(0.0f);
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

void ContactGraphScene::simulateStep(float dt) {
  // This scene is intentionally kept "simple":
  // - The lower stack is kinematic (fixed pose).
  // - The top box can be driven horizontally.
  // - When "F: toggle falling when unsupported" is enabled, the top box can switch to
  //   a dynamic rigid body and free-fall with collisions (including rotation).

  if (paused_) return;
  if (dt <= 0.0f) return;

  driveTime_ += dt;

  // Keep the stack kinematic (except the top box when in free-fall mode).
  for (int i = 0; i < (int)bodies_.size(); ++i) {
    if (i == 2) continue; // top box
    lrc::RigidBody& b = bodies_[i];
    b.v = glm::vec3(0.0f);
    b.w = glm::vec3(0.0f);
    b.q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
  }

  lrc::RigidBody& top = bodies_[2];

  // Horizontal drive (weak PD velocity injection) - matches the pre-fall behavior.
  float vxCmd = 0.0f;
  if (driveEnabled_) {
    float omega = 2.0f * 3.1415926535f * driveHz_;
    float targetX = driveBasePos_.x + drivePosAmp_ * std::sin(omega * driveTime_);
    float errorX = targetX - top.x.x;
    vxCmd = clampf(driveKp_ * errorX, -driveVelAmp_, driveVelAmp_);
  }
  if (topFreeFall_ && topOnFloor_) vxCmd = 0.0f;
  if (topFreeFall_ && topOnFloor_) {
    top.v.x = 0.0f;
    top.v.z = 0.0f;
  } else {
    top.v.x = vxCmd;
    top.v.z = 0.0f;
  }

  // If falling toggle is off, force the top back to kinematic mode.
  if (!driveAllowFall_) {
    topFreeFall_ = false;
    topOnFloor_ = false;
  fallHasPivot_ = false;
  fallSupportLower_ = -2;
  fallPivot_ = glm::vec3(0.0f);
  fallAxis_ = glm::vec3(0.0f, 0.0f, 1.0f);
  }

  // --- Kinematic mode (top is supported / hasn't started free-fall yet)
  if (!topFreeFall_) {
    // Keep the top upright while it is part of the stack.
    top.q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    top.w = glm::vec3(0.0f);
    top.v.y = 0.0f;

    // Apply horizontal motion kinematically.
    top.x.x += top.v.x * dt;

    // Rebuild contacts in this (driven) configuration so we can test support.
    rebuildContacts();

    if (driveAllowFall_) {
      // Collect support contact points under the top.
      std::vector<glm::vec2> supportXZ;
    std::vector<int> supportLower;
      float minSupportY = 1e9f;

      float topBottomY = top.x.y - boxHalf_.y;

      for (const Contact& c : contacts_) {
        if (c.upper != 2) continue;
        if (c.lower == -1) continue; // ground doesn't directly support the top in this setup

        // Only consider contacts close to the bottom face of the top.
        if (std::fabs(c.p.y - topBottomY) > 0.05f) continue;

        supportXZ.push_back(glm::vec2(c.p.x, c.p.z));
        supportLower.push_back(c.lower);
        minSupportY = std::min(minSupportY, c.p.y);
      }

      bool supportedForFall = false;
      if (supportXZ.size() >= 3) {
        std::vector<glm::vec2> hull = convexHull2D(supportXZ);
        supportedForFall = pointInConvexPolygon2D(hull, glm::vec2(top.x.x, top.x.z));
      }

      // If supported, keep the top resting on the supports; otherwise start free-fall.
      if (supportedForFall) {
        top.x.y = minSupportY + boxHalf_.y;
        top.v.y = 0.0f;

        rebuildContacts();
        return;
      }

      // Transition to dynamic free-fall (continue below in the same step).
      topFreeFall_ = true;
      top.v.y = 0.0f;

      // Cache the most relevant supporting surface to decide a fall rotation direction.
      fallHasPivot_ = false;
      fallSupportLower_ = -2;
      fallPivot_ = glm::vec3(0.0f);
      fallAxis_ = glm::vec3(0.0f, 0.0f, 1.0f);

      if (minSupportY < 1e8f && !supportXZ.empty()) {
        int bestLower = -1;
        int bestCount = 0;
        std::vector<int> counts((int)bodies_.size(), 0);
        for (int li : supportLower) {
          if (li >= 0 && li < (int)counts.size()) counts[li]++;
        }
        for (int i = 0; i < (int)counts.size(); ++i) {
          if (counts[i] > bestCount) {
            bestCount = counts[i];
            bestLower = i;
          }
        }

        std::vector<glm::vec2> pts = supportXZ;
        if (bestLower >= 0 && bestCount > 0) {
          pts.clear();
          for (size_t i = 0; i < supportXZ.size(); ++i) {
            if (supportLower[i] == bestLower) pts.push_back(supportXZ[i]);
          }
        }

        std::vector<glm::vec2> hull = pts;
        if (pts.size() >= 3) hull = convexHull2D(pts);

        const glm::vec2 comXZ(top.x.x, top.x.z);
        const glm::vec2 pivotXZ = closestPointOnPolyBoundary2D(hull, comXZ);
        fallPivot_ = glm::vec3(pivotXZ.x, minSupportY, pivotXZ.y);
        fallSupportLower_ = bestLower;

        const glm::mat3 IinvW = invInertiaWorld(top);
        const glm::vec3 tauA = glm::cross(top.x - fallPivot_, gravity_);
        const glm::vec3 wDir = IinvW * tauA;
        const float wl = glm::length(wDir);
        if (wl > 1e-6f) {
          fallAxis_ = wDir / wl;
          fallHasPivot_ = true;
        }
      }

      if (fallRotMode_ == 1) {
        // Mode A: tilt and kick around the gravity torque direction induced by the supporting surface.
        const float tilt = glm::radians(fallKickDeg_);
        const glm::vec3 axis = fallHasPivot_ ? fallAxis_ : glm::vec3(0.0f, 0.0f, 1.0f);
        top.q = glm::angleAxis(tilt, axis);
        top.w = axis * fallKickOmega_;
      } else {
        top.q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
        top.w = glm::vec3(0.0f);
      }
    } else {
      rebuildContacts();
      return;
    }
  }

  // --- Dynamic mode (top free-falls with collisions)
  const glm::vec3 xPrev = top.x;
  const glm::quat qPrev = top.q;

  if (fallRotMode_ == 2 && driveEnabled_) {
    // Mode B: bias rotation using the last supporting surface, so the direction matches the lower box.
    if (fallHasPivot_ && fallSupportLower_ >= 0) {
      bool hasSupportContact = false;
      const float topBottomY = top.x.y - boxHalf_.y;
      for (const Contact& c : contacts_) {
        if (c.upper == 2 && c.lower == fallSupportLower_) {
          if (std::fabs(c.p.y - topBottomY) < 0.08f) {
            hasSupportContact = true;
            break;
          }
        }
      }
      if (hasSupportContact) {
        const glm::mat3 IinvW = invInertiaWorld(top);
        const glm::vec3 tauA = glm::cross(top.x - fallPivot_, gravity_);
        top.w += (fallTorqueGain_ * dt) * (IinvW * tauA);
      }
    } else {
      // Fallback: the previous heuristic around a top corner.
      const glm::mat3 IinvW = invInertiaWorld(top);
      const glm::vec3 rLocal = glm::vec3(+boxHalf_.x, +boxHalf_.y, +boxHalf_.z);
      const glm::vec3 r = glm::mat3_cast(top.q) * rLocal;
      const glm::vec3 J = glm::vec3(vxCmd, 0.0f, 0.0f);
      const glm::vec3 tau = glm::cross(r, J);
      top.w += (fallTorqueGain_ * dt) * (IinvW * tau);
    }
  }

  // Integrate (semi-implicit Euler).
  top.v += gravity_ * dt;
  top.x += top.v * dt;
  top.q = lrc::integrateOrientation(top.q, top.w, dt);

  // Position-based collision resolution against:
  // - ground plane y = groundY_
  // - other (kinematic) axis-aligned boxes
  constexpr int kIters = 8;
  constexpr float kSlop = 1e-4f;

  for (int it = 0; it < kIters; ++it) {
    glm::vec3 samples[kObbSamplePointCount];
    obbSamplePointsWorld(top, samples);

    // Ground
    for (int i = 0; i < kObbSamplePointCount; ++i) {
      const glm::vec3 p = samples[i];
      float pen = groundY_ - p.y;
      if (pen > 0.0f) {
        solvePointPenetration(top, p, glm::vec3(0.0f, 1.0f, 0.0f), pen + kSlop);
      }
    }

    // Other boxes (treated as static AABBs)
    for (int bi = 0; bi < (int)bodies_.size(); ++bi) {
      if (bi == 2) continue;
      const lrc::RigidBody& s = bodies_[bi];
      const glm::vec3 bmin = s.x - s.halfExtents;
      const glm::vec3 bmax = s.x + s.halfExtents;
      bool didPush = false;

      // A) Push out top samples that end up inside the AABB.
      for (int i = 0; i < kObbSamplePointCount; ++i) {
        const glm::vec3 p = samples[i];
        if (p.x <= bmin.x || p.x >= bmax.x ||
            p.y <= bmin.y || p.y >= bmax.y ||
            p.z <= bmin.z || p.z >= bmax.z) {
          continue;
        }

        // Point is inside the AABB -> push it out along the closest face normal.
        float dx0 = p.x - bmin.x;
        float dx1 = bmax.x - p.x;
        float dy0 = p.y - bmin.y;
        float dy1 = bmax.y - p.y;
        float dz0 = p.z - bmin.z;
        float dz1 = bmax.z - p.z;

        float best = dx0;
        glm::vec3 n(-1.0f, 0.0f, 0.0f);

        if (dx1 < best) { best = dx1; n = glm::vec3( 1.0f, 0.0f, 0.0f); }
        if (dy0 < best) { best = dy0; n = glm::vec3( 0.0f,-1.0f, 0.0f); }
        if (dy1 < best) { best = dy1; n = glm::vec3( 0.0f, 1.0f, 0.0f); }
        if (dz0 < best) { best = dz0; n = glm::vec3( 0.0f, 0.0f,-1.0f); }
        if (dz1 < best) { best = dz1; n = glm::vec3( 0.0f, 0.0f, 1.0f); }

        solvePointPenetration(top, p, n, best + kSlop);
        didPush = true;
      }

      // B) Also handle the opposite containment case (static corners inside the OBB).
      // This is important when the OBB overlaps an AABB edge/edge region without any OBB sample
      // falling strictly inside the AABB.
      glm::vec3 sc[8];
      aabbCornersWorld(s, sc);
      for (int i = 0; i < 8; ++i) {
        glm::vec3 nWorld(0.0f);
        float pen = 0.0f;
        if (pointInsideObb(top, sc[i], &nWorld, &pen)) {
          // For a fixed point inside the OBB, moving the OBB opposite to the outward normal expels it.
          solvePointPenetration(top, sc[i], -nWorld, pen + kSlop);
          didPush = true;
        }
      }

      // C) Face-level resolution (OBB vs AABB) for the common "face-face" overlap case where
      // no corners or sample points are strictly contained.
      if (/*!didPush*/1) {
        glm::vec3 nSat(0.0f);
        float penSat = 0.0f;
        if (obbAabbMtv(top, s, &nSat, &penSat) && penSat > 1e-6f) {
          const glm::vec3 p = obbSupportPointWorld(top, -nSat);
          solvePointPenetration(top, p, nSat, penSat + kSlop);
        }
        top.v.x *= 0.0001f;
        top.v.z *= 0.0001f;
        top.v.y *= 0.00001f;
        top.w   *= 0.0001f;
      }
    }

  }

  // Update velocities from the corrected pose.
  top.v = (top.x - xPrev) / dt;
  top.w = lrc::quatToOmegaWorld(qPrev, top.q, dt);

  // Once the top has fallen and settled onto the floor, stop horizontal drifting.
  if (topFreeFall_) {
    bool onFloor = false;
    {
      glm::vec3 samples[kObbSamplePointCount];
      obbSamplePointsWorld(top, samples);
      float minY = samples[0].y;
      for (int i = 1; i < kObbSamplePointCount; ++i) {
        if (samples[i].y < minY) minY = samples[i].y;
      }
      onFloor = (minY <= groundY_ + 2e-3f);
    }

    if (onFloor && std::fabs(top.v.y) < 0.2f) {
      topOnFloor_ = true;
    }

    if (topOnFloor_) {
      top.v.x *= 0.1f;
      top.v.z *= 0.1f;
      top.v.y *= 0.01f;
      top.w   *= 0.1f;
      if (std::fabs(top.v.y) < 0.2f/*0.05f*/) top.v.y = 0.0f;
      if (glm::length(top.w) < 0.2f) top.w = glm::vec3(0.0f);
    }
  }

  rebuildContacts();
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
    const glm::mat4 R = glm::toMat4(b.q);
    glMultMatrixf(glm::value_ptr(R));
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

  if (key == 't' || key == 'T') {
    driveEnabled_ = !driveEnabled_;
    return;
  }
  if (key == 'f' || key == 'F') {
    driveAllowFall_ = !driveAllowFall_;
    return;
  }
  if (key == 'm' || key == 'M') {
    fallRotMode_ = (fallRotMode_ + 1) % 3;
    return;
  }
  if (key == '[') {
    drivePosAmp_ = std::max(0.0f, drivePosAmp_ - 0.02f);
    return;
  }
  if (key == ']') {
    drivePosAmp_ += 0.02f;
    return;
  }
  if (key == ';') {
    driveHz_ = std::max(0.05f, driveHz_ - 0.05f);
    return;
  }
  if (key == '\'') {
    driveHz_ += 0.05f;
    return;
  }
}

void ContactGraphScene::rebuildContacts() {
  contacts_.clear();
  graphEdges_.clear();
  bodySupported_.assign(bodies_.size(), false);

  // Floor contacts.
  for (int i = 0; i < (int)bodies_.size(); ++i) {
    addBoxFloorContacts(i);
  }

  // Box-box contacts (broad phase via world AABB).
  for (int i = 0; i < (int)bodies_.size(); ++i) {
    for (int j = i + 1; j < (int)bodies_.size(); ++j) {
      const glm::vec3 aMin = aabbMin(bodies_[i]);
      const glm::vec3 aMax = aabbMax(bodies_[i]);
      const glm::vec3 bMin = aabbMin(bodies_[j]);
      const glm::vec3 bMax = aabbMax(bodies_[j]);

      const bool overlapX = (aMin.x <= bMax.x) && (aMax.x >= bMin.x);
      const bool overlapY = (aMin.y <= bMax.y) && (aMax.y >= bMin.y);
      const bool overlapZ = (aMin.z <= bMax.z) && (aMax.z >= bMin.z);
      if (!overlapX || !overlapY || !overlapZ) continue;

      // Determine a plausible "upper" / "lower" ordering by center height.
      const bool iAbove = bodies_[i].x.y >= bodies_[j].x.y;
      const int upper = iAbove ? i : j;
      const int lower = iAbove ? j : i;
      addBoxBoxContacts(upper, lower);
    }
  }

  buildContactGraph();
  computeSupportingContacts();
}

void ContactGraphScene::buildContactGraph() {
  graphEdges_.clear();
  const int n = (int)contacts_.size();
  if (n <= 1) return;

  // Create undirected edges between contact nodes that share a body.
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      const Contact& a = contacts_[i];
      const Contact& b = contacts_[j];
      const bool shareUpper = (a.upper == b.upper) && (a.upper >= 0);
      const bool shareLower = (a.lower == b.lower) && (a.lower >= 0);
      const bool share = shareUpper || shareLower;
      if (!share) continue;
      graphEdges_.push_back(std::make_pair(i, j));
    }
  }
}

void ContactGraphScene::computeSupportingContacts() {
  for (Contact& c : contacts_) c.supporting = false;
  bodySupported_.assign(bodies_.size(), false);

  // For each body, check whether its COM projection is inside the convex hull
  // of "static" contact points supporting it from below (incl. floor).
  for (int bi = 0; bi < (int)bodies_.size(); ++bi) {
    std::vector<int> cand;
    cand.reserve(16);

    for (int ci = 0; ci < (int)contacts_.size(); ++ci) {
      const Contact& c = contacts_[ci];
      if (c.upper != bi) continue;
      if (!c.isStatic) continue;
      // Accept floor (lower == -1) and any lower body.
      cand.push_back(ci);
    }

    if ((int)cand.size() < 3) continue;

    std::vector<glm::vec2> pts;
    pts.reserve(cand.size());
    for (int idx : cand) {
      const glm::vec3& p = contacts_[idx].p;
      pts.push_back(glm::vec2(p.x, p.z));
    }

    const std::vector<glm::vec2> hull = convexHull2D(std::move(pts));
    if (hull.size() < 3) continue;

    const glm::vec2 c2(bodies_[bi].x.x, bodies_[bi].x.z);
    if (!pointInConvexPolygon2D(hull, c2)) continue;

    bodySupported_[bi] = true;

    // Mark supporting contacts for visualization.
    for (int idx : cand) {
      contacts_[idx].supporting = true;
    }
  }
}

void ContactGraphScene::addBoxFloorContacts(int bodyIdx) {
  const lrc::RigidBody& b = bodies_[bodyIdx];
  const glm::vec3 aMin = aabbMin(b);
  if (aMin.y > groundY_ + 1e-3f) return;

  // Use the four bottom corners in the box local frame, transformed to world.
  const float hx = b.halfExtents.x;
  const float hy = b.halfExtents.y;
  const float hz = b.halfExtents.z;

  const glm::vec3 localCorners[4] = {
    glm::vec3(-hx, -hy, -hz),
    glm::vec3(+hx, -hy, -hz),
    glm::vec3(+hx, -hy, +hz),
    glm::vec3(-hx, -hy, +hz)
  };

  for (int i = 0; i < 4; ++i) {
    const glm::vec3 pw = worldFromLocal(b, localCorners[i]);
    if (pw.y > groundY_ + 0.01f) continue;

    Contact c;
    c.upper = bodyIdx;
    c.lower = -1;
    c.p = glm::vec3(pw.x, groundY_, pw.z);
    c.n = glm::vec3(0.0f, 1.0f, 0.0f);
    c.isStatic = isStaticContact(bodyIdx, -1, c.p, c.n);
    c.supporting = false;
    contacts_.push_back(c);
  }
}

void ContactGraphScene::addBoxBoxContacts(int upperIdx, int lowerIdx) {
  if (upperIdx < 0 || lowerIdx < 0) return;
  const lrc::RigidBody& upper = bodies_[upperIdx];
  const lrc::RigidBody& lower = bodies_[lowerIdx];

  const glm::vec3 uMin = aabbMin(upper);
  const glm::vec3 uMax = aabbMax(upper);
  const glm::vec3 lMin = aabbMin(lower);
  const glm::vec3 lMax = aabbMax(lower);

  const bool overlapX = (uMin.x <= lMax.x) && (uMax.x >= lMin.x);
  const bool overlapZ = (uMin.z <= lMax.z) && (uMax.z >= lMin.z);
  if (!overlapX || !overlapZ) return;

  // Contact plane at the lower AABB top (works well for axis-aligned stacks).
  const float yPlane = lMax.y;
  const float gap = uMin.y - yPlane;
  if (gap > 0.02f) return;

  const float x0 = std::max(uMin.x, lMin.x);
  const float x1 = std::min(uMax.x, lMax.x);
  const float z0 = std::max(uMin.z, lMin.z);
  const float z1 = std::min(uMax.z, lMax.z);
  if (x1 - x0 < 1e-5f || z1 - z0 < 1e-5f) return;

  const glm::vec3 corners[4] = {
    glm::vec3(x0, yPlane, z0),
    glm::vec3(x1, yPlane, z0),
    glm::vec3(x1, yPlane, z1),
    glm::vec3(x0, yPlane, z1)
  };

  for (int i = 0; i < 4; ++i) {
    Contact c;
    c.upper = upperIdx;
    c.lower = lowerIdx;
    c.p = corners[i];
    c.n = glm::vec3(0.0f, 1.0f, 0.0f);
    c.isStatic = isStaticContact(upperIdx, lowerIdx, c.p, c.n);
    c.supporting = false;
    contacts_.push_back(c);
  }
}

bool ContactGraphScene::isStaticContact(int upperIdx, int lowerIdx, const glm::vec3& p, const glm::vec3& n) const {
  const lrc::RigidBody& a = bodies_[upperIdx];
  const glm::vec3 va = a.v + glm::cross(a.w, p - a.x);

  glm::vec3 vb(0.0f);
  if (lowerIdx >= 0) {
    const lrc::RigidBody& b = bodies_[lowerIdx];
    vb = b.v + glm::cross(b.w, p - b.x);
  }

  const glm::vec3 rel = va - vb;
  const float speed = glm::length(rel);
  // Keep consistent with the earlier "static" split for contact graphs.
  (void)n;
  return speed < 0.05f;
}

float ContactGraphScene::cross2(const glm::vec2& a, const glm::vec2& b) {
  return a.x * b.y - a.y * b.x;
}

float ContactGraphScene::cross2(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c) {
  return cross2(b - a, c - a);
}

bool ContactGraphScene::pointOnSegment2D(
  const glm::vec2& a,
  const glm::vec2& b,
  const glm::vec2& p
) {
  const float eps = 1e-5f;
  const glm::vec2 ab = b - a;
  const glm::vec2 ap = p - a;
  const float area = std::abs(cross2(ab, ap));
  if (area > eps) return false;
  const float d = glm::dot(ap, ab);
  if (d < -eps) return false;
  if (d > glm::dot(ab, ab) + eps) return false;
  return true;
}

std::vector<glm::vec2> ContactGraphScene::convexHull2D(std::vector<glm::vec2> pts) {
  if (pts.size() <= 1) return pts;

  std::sort(pts.begin(), pts.end(), [](const glm::vec2& a, const glm::vec2& b) {
    if (a.x != b.x) return a.x < b.x;
    return a.y < b.y;
  });

  std::vector<glm::vec2> hull;
  hull.reserve(pts.size() * 2);

  // Lower hull.
  for (const glm::vec2& p : pts) {
    while (hull.size() >= 2) {
      const glm::vec2& a = hull[hull.size() - 2];
      const glm::vec2& b = hull[hull.size() - 1];
      if (cross2(a, b, p) <= 0.0f) hull.pop_back();
      else break;
    }
    hull.push_back(p);
  }

  // Upper hull.
  const size_t lowerSize = hull.size();
  for (int i = (int)pts.size() - 2; i >= 0; --i) {
    const glm::vec2& p = pts[(size_t)i];
    while (hull.size() > lowerSize) {
      const glm::vec2& a = hull[hull.size() - 2];
      const glm::vec2& b = hull[hull.size() - 1];
      if (cross2(a, b, p) <= 0.0f) hull.pop_back();
      else break;
    }
    hull.push_back(p);
  }

  if (!hull.empty()) hull.pop_back();
  return hull;
}

bool ContactGraphScene::pointInConvexPolygon2D(
  const std::vector<glm::vec2>& poly,
  const glm::vec2& p
) {
  const int n = (int)poly.size();
  if (n < 3) return false;

  // Allow boundary as inside.
  for (int i = 0; i < n; ++i) {
    const glm::vec2 a = poly[i];
    const glm::vec2 b = poly[(i + 1) % n];
    if (pointOnSegment2D(a, b, p)) return true;
  }

  // Check that p is on the same side of all edges.
  float sign = 0.0f;
  for (int i = 0; i < n; ++i) {
    const glm::vec2 a = poly[i];
    const glm::vec2 b = poly[(i + 1) % n];
    const float c = cross2(a, b, p);
    if (std::abs(c) < 1e-6f) continue;
    if (sign == 0.0f) sign = c;
    else if ((sign > 0.0f) != (c > 0.0f)) return false;
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

  const char* rot = "none";


  if (fallRotMode_ == 1) rot = "A";


  else if (fallRotMode_ == 2) rot = "B";



  char extra[240];


  std::snprintf(


    extra,


    sizeof(extra),


    "contacts=%d static=%d dyn=%d supporting=%d | drive=%s A=%.2f f=%.2f fall=%s rot=%s",
    (int)contacts_.size(),
    staticC,
    dynC,
    supp,
    driveEnabled_ ? "ON" : "OFF",
    drivePosAmp_,
    driveHz_,
    driveAllowFall_ ? "ON" : "OFF",
    rot
  );
  updateWindowTitle(extra);
}
