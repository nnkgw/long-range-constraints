#pragma once

#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

#include "IScene.h"

class ChainLRCScene final : public IScene {
public:
  struct RigidBody;
  struct JointPointConstraint;

  ChainLRCScene();
  ~ChainLRCScene() override;

  void reset() override;
  void display() override;
  void idle() override;

  void reshape(int w, int h) override;
  void mouse(int button, int state, int x, int y) override;
  void motion(int x, int y) override;
  void keyboard(unsigned char key, int x, int y) override;

  void usage() override;

  void updateWindowTitle();
  void buildChain();
  void simulateStep(float dt);

  static float clampf(float x, float a, float b);
private:
  struct MaxDistanceConstraint;

  static void drawGround(float y);
  static void drawBoxWire(float hx, float hy, float hz);

private:
  // Simulation parameters.
  float dt_ = 1.0f / 60.0f;
  glm::vec3 gravity_ = glm::vec3(0.0f, -9.8f, 0.0f);

  int numBodies_ = 16;
  int iters_ = 8;

  bool useLRC_ = true;

  float jointCompliance_ = 0.0f;
  float lrcCompliance_ = 0.0f;

  glm::vec3 boxHalf_ = glm::vec3(0.05f, 0.12f, 0.05f);

  // Kinematic root drive (swing motion).
  float rootDriveAmp_ = 0.25f;  // meters
  float rootDriveHz_ = 0.50f;   // cycles per second
  float rootDriveT_ = 0.0f;     // seconds
  glm::vec3 rootBasePos_ = glm::vec3(0.0f);

  // Camera and input.
  float camDist_ = 2.5f;
  float camYaw_ = 0.0f;
  float camPitch_ = 0.3f;
  glm::vec2 camPan_ = glm::vec2(0.0f);

  int lastMouseX_ = 0;
  int lastMouseY_ = 0;
  bool lbtn_ = false;
  bool rbtn_ = false;

  // Timing (idle accumulator).
  float lastSimMs_ = 0.0f;
  int lastTms_ = 0;
  float acc_ = 0.0f;

  // World state.
  std::vector<RigidBody> bodies_;
  std::vector<JointPointConstraint> joints_;
  std::vector<MaxDistanceConstraint> lrcs_;
};
