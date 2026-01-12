#pragma once
#include "ChainSceneBase.h"

class ChainFreeScene final : public ChainSceneBase {
public:
  ChainFreeScene();
  ~ChainFreeScene() override = default;

  void reset() override;
  void display() override;
  void keyboard(unsigned char key, int x, int y) override;
  const char* name() const override { return "Phase C1 (Free chain + Follow Cam)"; }
  void usage() const override;

private:
  void simulateStep(float dt) override;
  void drawSceneContents() override;
  void onFrameEnd() override;

  void buildScene();

  int numBodies_ = 14;
  glm::vec3 boxHalf_{0.05f, 0.10f, 0.05f};
  float mass_ = 1.0f;

  float jointCompliance_ = 0.0f;
  float lrcCompliance_ = 0.0f;

  bool followCam_ = true;

  bool paused_ = false;

  bool showHierarchy_ = true;
  int hierarchyLevel_ = -1; // -1 means all levels.
};
