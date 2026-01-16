#pragma once
#include <string>

#include <glm/glm.hpp>

#include "IScene.h"
#include "RigidChainSim.h"

class ChainSceneBase : public IScene {
public:
  ChainSceneBase();
  ~ChainSceneBase() override = default;

  void reshape(int w, int h) override;
  void idle() override;
  void mouse(int button, int state, int x, int y) override;
  void motion(int x, int y) override;

protected:
  void applyCameraTransform();
  void updateWindowTitle(const char* extra = nullptr) const;

  virtual void simulateStep(float dt) = 0;
  virtual void drawSceneContents() = 0;

  // Optional hook called once per rendered frame after sub-steps.
  virtual void onFrameEnd() {}

  // Common camera state (matches the original single-file demo style).
  float camYaw_ = 0.0f;
  float camPitch_ = 0.6f;
  float camDist_ = 4.0f;
  glm::vec3 camPan_{0.0f, 0.0f, 0.0f};

  bool lbtn_ = false;
  bool rbtn_ = false;
  int lastMouseX_ = 0;
  int lastMouseY_ = 0;

  // Fixed-step accumulator (keeps behavior deterministic with variable frame rate).
  float dt_ = 1.0f / 60.0f;
  int lastTms_ = 0;
  float acc_ = 0.0f;
  float lastSimMs_ = 0.0f;

  // If you want "slow PC -> slower animation", keep the 4-step cap.
  int maxSubStepsPerFrame_ = 4;

  // Common physics and presentation.
  lrc::ChainSystem chain_;
  glm::vec3 gravity_{0.0f, -9.8f, 0.0f};

  // UI knobs.
  int iters_ = 20;
  bool useLrc_ = true;

  // Root kinematic drive (for the swinging motion).
  glm::vec3 rootBasePos_{0.0f};
  float rootDriveT_ = 0.0f;
  float rootDriveAmp_ = 0.35f;
  float rootDriveHz_ = 0.5f;

  // Scene name for window title.
  std::string titleName_ = "Chain";

  // Cached extra string for the window title.
  // This prevents one-shot updates (e.g., from a key press) from being
  // immediately overwritten by the per-frame title refresh.
  mutable std::string titleExtra_;
  mutable bool hasTitleExtra_ = false;
};
