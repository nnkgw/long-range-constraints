#pragma once
#include <vector>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/quaternion.hpp>

namespace lrc {

static constexpr float kPi = 3.14159265358979323846f;

struct RigidBody {
  glm::vec3 x{0.0f};
  glm::quat q{1.0f, 0.0f, 0.0f, 0.0f};

  glm::vec3 v{0.0f};
  glm::vec3 w{0.0f};

  glm::vec3 xPred{0.0f};
  glm::quat qPred{1.0f, 0.0f, 0.0f, 0.0f};

  float invMass = 0.0f;
  glm::mat3 invInertiaLocal{0.0f};

  glm::vec3 halfExtents{0.05f, 0.10f, 0.05f};
};

struct JointPointConstraint {
  int a = -1;
  int b = -1;
  glm::vec3 la{0.0f};
  glm::vec3 lb{0.0f};
  glm::vec3 lambda{0.0f};
  float compliance = 0.0f;

  void solve(std::vector<RigidBody>& bodies, float dt);
};

struct MaxDistanceConstraint {
  int a = -1;
  int b = -1;
  glm::vec3 la{0.0f};
  glm::vec3 lb{0.0f};
  float maxDist = 0.0f;
  float lambda = 0.0f;
  float compliance = 0.0f;

  void solve(std::vector<RigidBody>& bodies, float dt);
};

struct DistanceBoundsConstraint {
  int a = -1;
  int b = -1;
  glm::vec3 la{0.0f};
  glm::vec3 lb{0.0f};
  float minDist = 0.0f;
  float maxDist = 0.0f;
  float lambda = 0.0f;
  float compliance = 0.0f;

  void solve(std::vector<RigidBody>& bodies, float dt);
};

glm::mat3 boxInvInertiaLocal(float mass, const glm::vec3& halfExtents);

glm::vec3 worldPointPred(const RigidBody& b, const glm::vec3& localP);

glm::quat integrateOrientation(const glm::quat& q, const glm::vec3& wWorld, float dt);
glm::vec3 quatToOmegaWorld(const glm::quat& q0, const glm::quat& q1, float dt);

class ChainSystem {
public:
  void clear();

  void buildVerticalChain(int numBodies,
                          const glm::vec3& boxHalf,
                          float mass,
                          const glm::vec3& rootPos,
                          bool kinematicRoot = true);

  void buildLrcMax(float lrcCompliance);
  void buildLrcFreeMaxHierarchy(float lrcCompliance, int minSpan = 2);

  void buildLrcBounds(float jointLimitRad, float lrcCompliance);

  void step(float dt, int iters, bool useLrc, bool useBounds, const glm::vec3& gravity);

  std::vector<RigidBody>& bodies() { return bodies_; }
  const std::vector<RigidBody>& bodies() const { return bodies_; }

  std::vector<JointPointConstraint>& joints() { return joints_; }
  const std::vector<JointPointConstraint>& joints() const { return joints_; }

  const std::vector<MaxDistanceConstraint>& lrcMax() const { return lrcMax_; }
  const std::vector<DistanceBoundsConstraint>& lrcBounds() const { return lrcBounds_; }

  glm::vec3 rootAnchorLocal() const { return rootAnchorLocal_; }
  glm::vec3 childAnchorLocal() const { return childAnchorLocal_; }
  float segmentLen() const { return segLen_; }
  int numBodies() const { return (int)bodies_.size(); }

private:
  std::vector<RigidBody> bodies_;
  std::vector<JointPointConstraint> joints_;
  std::vector<MaxDistanceConstraint> lrcMax_;
  std::vector<DistanceBoundsConstraint> lrcBounds_;

  glm::vec3 rootAnchorLocal_{0.0f, -0.10f, 0.0f};
  glm::vec3 childAnchorLocal_{0.0f,  0.10f, 0.0f};
  float segLen_ = 0.20f;
};

struct DistanceBoundSequence {
  std::vector<float> dMin;
  std::vector<float> dMax;
};

DistanceBoundSequence computeDistanceBoundsFromAngleLimits(int numSegments,
                                                           float segLen,
                                                           float restAngleRad,
                                                           float jointLimitRad);

} // namespace lrc
