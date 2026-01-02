#include "RigidChainSim.h"

#include <algorithm>
#include <cmath>

namespace lrc {

static float clampf(float x, float lo, float hi) {
  return std::max(lo, std::min(hi, x));
}

static void applySmallRotation(glm::quat& q, const glm::vec3& dthetaWorld) {
  glm::quat dq(0.0f, dthetaWorld.x, dthetaWorld.y, dthetaWorld.z);
  q = glm::normalize(q + 0.5f * dq * q);
}

glm::quat integrateOrientation(const glm::quat& q, const glm::vec3& wWorld, float dt) {
  glm::quat wq(0.0f, wWorld.x, wWorld.y, wWorld.z);
  glm::quat dq = 0.5f * wq * q;
  glm::quat q2 = q + dq * dt;
  return glm::normalize(q2);
}

glm::vec3 quatToOmegaWorld(const glm::quat& q0, const glm::quat& q1, float dt) {
  glm::quat dq = q1 * glm::inverse(q0);
  if (dq.w < 0.0f) dq = -dq;

  float w = clampf(dq.w, -1.0f, 1.0f);
  float angle = 2.0f * std::acos(w);
  if (angle < 1e-8f) return glm::vec3(0.0f);

  float s = std::sqrt(std::max(0.0f, 1.0f - w * w));
  glm::vec3 axis(dq.x, dq.y, dq.z);
  if (s > 1e-8f) axis /= s;

  return axis * (angle / dt);
}

glm::mat3 boxInvInertiaLocal(float mass, const glm::vec3& halfExtents) {
  if (mass <= 0.0f) return glm::mat3(0.0f);

  float hx = halfExtents.x;
  float hy = halfExtents.y;
  float hz = halfExtents.z;

  float Ixx = (1.0f / 3.0f) * mass * (hy * hy + hz * hz);
  float Iyy = (1.0f / 3.0f) * mass * (hx * hx + hz * hz);
  float Izz = (1.0f / 3.0f) * mass * (hx * hx + hy * hy);

  glm::mat3 I(0.0f);
  I[0][0] = Ixx;
  I[1][1] = Iyy;
  I[2][2] = Izz;

  glm::mat3 inv(0.0f);
  inv[0][0] = (Ixx > 0.0f) ? (1.0f / Ixx) : 0.0f;
  inv[1][1] = (Iyy > 0.0f) ? (1.0f / Iyy) : 0.0f;
  inv[2][2] = (Izz > 0.0f) ? (1.0f / Izz) : 0.0f;
  return inv;
}

static glm::mat3 invInertiaWorld(const RigidBody& b) {
  glm::mat3 R = glm::toMat3(b.qPred);
  return R * b.invInertiaLocal * glm::transpose(R);
}

glm::vec3 worldPointPred(const RigidBody& b, const glm::vec3& localP) {
  return b.xPred + glm::rotate(b.qPred, localP);
}

void JointPointConstraint::solve(std::vector<RigidBody>& bodies, float dt) {
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

    if (denom < 1e-12f) continue;

    float dlam = (-C - alpha * lambda[k]) / (denom + alpha);
    lambda[k] += dlam;

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

void MaxDistanceConstraint::solve(std::vector<RigidBody>& bodies, float dt) {
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
  if (dist <= maxDist) {
    lambda = 0.0f;
    return;
  }

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

  if (denom < 1e-12f) return;

  float dlam = (-C - alpha * lambda) / (denom + alpha);
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

void DistanceBoundsConstraint::solve(std::vector<RigidBody>& bodies, float dt) {
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

  float target = dist;
  if (dist > maxDist) target = maxDist;
  if (dist < minDist) target = minDist;

  if (target == dist) {
    lambda = 0.0f;
    return;
  }

  glm::vec3 n = d / dist;
  float C = dist - target;

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

  if (denom < 1e-12f) return;

  float dlam = (-C - alpha * lambda) / (denom + alpha);
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

void ChainSystem::clear() {
  bodies_.clear();
  joints_.clear();
  lrcMax_.clear();
  lrcBounds_.clear();
}

void ChainSystem::buildVerticalChain(int numBodies,
                                     const glm::vec3& boxHalf,
                                     float mass,
                                     const glm::vec3& rootPos,
                                     bool kinematicRoot) {
  clear();
  if (numBodies < 2) return;

  bodies_.resize(numBodies);

  segLen_ = 2.0f * boxHalf.y;
  rootAnchorLocal_ = glm::vec3(0.0f, -boxHalf.y, 0.0f);
  childAnchorLocal_ = glm::vec3(0.0f,  boxHalf.y, 0.0f);

  for (int i = 0; i < numBodies; ++i) {
    RigidBody b;
    b.halfExtents = boxHalf;
    b.x = rootPos - glm::vec3(0.0f, (float)i * segLen_, 0.0f);
    b.q = glm::quat(1.0f, 0.0f, 0.0f, 0.0f);
    b.v = glm::vec3(0.0f);
    b.w = glm::vec3(0.0f);
    b.xPred = b.x;
    b.qPred = b.q;

    if (i == 0 && kinematicRoot) {
      b.invMass = 0.0f;
      b.invInertiaLocal = glm::mat3(0.0f);
    } else {
      b.invMass = 1.0f / mass;
      b.invInertiaLocal = boxInvInertiaLocal(mass, boxHalf);
    }

    bodies_[i] = b;
  }

  joints_.resize(numBodies - 1);
  for (int i = 0; i < numBodies - 1; ++i) {
    JointPointConstraint jc;
    jc.a = i;
    jc.b = i + 1;
    jc.la = rootAnchorLocal_;
    jc.lb = childAnchorLocal_;
    jc.lambda = glm::vec3(0.0f);
    joints_[i] = jc;
  }
}

void ChainSystem::buildLrcMax(float lrcCompliance) {
  lrcMax_.clear();
  lrcBounds_.clear();

  int nb = (int)bodies_.size();
  if (nb < 3) return;

  std::vector<float> dMax;
  dMax.resize(nb - 1);
  dMax[0] = 0.0f;
  for (int i = 1; i < (int)dMax.size(); ++i) {
    dMax[i] = dMax[i - 1] + segLen_;
  }

  lrcMax_.reserve(std::max(0, nb - 2));
  for (int jointIdx = 1; jointIdx < nb - 1; ++jointIdx) {
    int bodyB = jointIdx + 1;

    MaxDistanceConstraint lc;
    lc.a = 0;
    lc.b = bodyB;
    lc.la = rootAnchorLocal_;
    lc.lb = childAnchorLocal_;
    lc.maxDist = dMax[jointIdx];
    lc.lambda = 0.0f;
    lc.compliance = lrcCompliance;
    lrcMax_.push_back(lc);
  }
}

DistanceBoundSequence computeDistanceBoundsFromAngleLimits(int numSegments,
                                                          float segLen,
                                                          float restAngleRad,
                                                          float jointLimitRad) {
  DistanceBoundSequence out;
  out.dMin.assign(numSegments + 1, 0.0f);
  out.dMax.assign(numSegments + 1, 0.0f);

  if (numSegments <= 0) return out;

  out.dMin[0] = 0.0f;
  out.dMax[0] = 0.0f;

  // Base case: first joint sits at distance segLen from the attachment.
  out.dMin[1] = segLen;
  out.dMax[1] = segLen;

  // alpha is the EXTERIOR angle between the last segment and the line to the attachment.
  // For the first joint, the segment points away from the attachment, so alpha = pi.
  float alphaMin = kPi;
  float alphaMax = kPi;

  for (int i = 1; i < numSegments; ++i) {
    float diMin = out.dMin[i];
    float diMax = out.dMax[i];

    float li = segLen;

    float rot = restAngleRad + jointLimitRad;

    // Lower bound: rotate toward the attachment as much as possible.
    float thetaMin = std::max(alphaMin - rot, 0.0f);
    float di1Min = std::sqrt(std::max(0.0f, diMin * diMin + li * li - 2.0f * diMin * li * std::cos(thetaMin)));
    float betaMin = 0.0f;
    if (diMin > 1e-6f && di1Min > 1e-6f) {
      float c = (diMin * diMin + di1Min * di1Min - li * li) / (2.0f * diMin * di1Min);
      betaMin = std::acos(clampf(c, -1.0f, 1.0f));
    }
    alphaMin = thetaMin + betaMin;
    out.dMin[i + 1] = di1Min;

    // Upper bound: rotate away from the attachment as much as possible.
    float thetaMax = std::min(alphaMax + rot, kPi);
    float di1Max = std::sqrt(std::max(0.0f, diMax * diMax + li * li - 2.0f * diMax * li * std::cos(thetaMax)));
    float betaMax = 0.0f;
    if (diMax > 1e-6f && di1Max > 1e-6f) {
      float c = (diMax * diMax + di1Max * di1Max - li * li) / (2.0f * diMax * di1Max);
      betaMax = std::acos(clampf(c, -1.0f, 1.0f));
    }
    alphaMax = thetaMax + betaMax;
    out.dMax[i + 1] = di1Max;
  }

  return out;
}

void ChainSystem::buildLrcBounds(float jointLimitRad, float lrcCompliance) {
  lrcMax_.clear();
  lrcBounds_.clear();

  int nb = (int)bodies_.size();
  if (nb < 3) return;

  // Number of segments from root anchor to the attachment point on body (jointIdx+1).
  int numSegments = nb - 1;

  DistanceBoundSequence seq = computeDistanceBoundsFromAngleLimits(numSegments,
                                                                   segLen_,
                                                                   0.0f,
                                                                   jointLimitRad);

  lrcBounds_.reserve(std::max(0, nb - 2));
  for (int jointIdx = 1; jointIdx < nb - 1; ++jointIdx) {
    int bodyB = jointIdx + 1;

    DistanceBoundsConstraint lc;
    lc.a = 0;
    lc.b = bodyB;
    lc.la = rootAnchorLocal_;
    lc.lb = childAnchorLocal_;
    lc.minDist = seq.dMin[jointIdx];
    lc.maxDist = seq.dMax[jointIdx];
    lc.lambda = 0.0f;
    lc.compliance = lrcCompliance;
    lrcBounds_.push_back(lc);
  }
}


void ChainSystem::buildLrcFreeMaxHierarchy(float lrcCompliance) {
  lrcMax_.clear();
  lrcBounds_.clear();

  int nb = (int)bodies_.size();
  int nj = nb - 1; // number of joint points
  if (nj < 3) return;

  // Joint k is the point between bodies k and k+1.
  // We represent it on body (k+1) at local childAnchorLocal_.
  // For a span of s segments, the straight-chain max distance is s * segLen_.
  for (int stride = 2; stride <= nj - 1; stride *= 2) {
    for (int j0 = 0; j0 + stride < nj; ++j0) {
      int j1 = j0 + stride;

      MaxDistanceConstraint lc;
      lc.a = j0 + 1;
      lc.b = j1 + 1;
      lc.la = childAnchorLocal_;
      lc.lb = childAnchorLocal_;
      lc.maxDist = (float)stride * segLen_;
      lc.lambda = 0.0f;
      lc.compliance = lrcCompliance;
      lrcMax_.push_back(lc);
    }
  }
}

void ChainSystem::step(float dt, int iters, bool useLrc, bool useBounds, const glm::vec3& gravity) {
  for (RigidBody& b : bodies_) {
    if (b.invMass == 0.0f) {
      b.xPred = b.x;
      b.qPred = b.q;
      continue;
    }
    b.v += dt * gravity;
    b.xPred = b.x + dt * b.v;
    b.qPred = integrateOrientation(b.q, b.w, dt);
  }

  for (int it = 0; it < iters; ++it) {
    for (JointPointConstraint& jc : joints_) {
      jc.solve(bodies_, dt);
    }
    if (useLrc) {
      if (useBounds) {
        for (DistanceBoundsConstraint& lc : lrcBounds_) {
          lc.solve(bodies_, dt);
        }
      } else {
        for (MaxDistanceConstraint& lc : lrcMax_) {
          lc.solve(bodies_, dt);
        }
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

} // namespace lrc
