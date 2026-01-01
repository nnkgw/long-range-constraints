#pragma once
#include "ChainSceneBase.h"

class ChainAngleBoundsScene final : public ChainSceneBase {
public:
  ChainAngleBoundsScene();
  ~ChainAngleBoundsScene() override = default;

  void reset() override;
  void display() override;
  void keyboard(unsigned char key, int x, int y) override;
  const char* name() const override { return "Phase B (Angle->Distance Bounds)"; }
  void usage() const override;

private:
  void simulateStep(float dt) override;
  void drawSceneContents() override;

  void buildScene();

  int numBodies_ = 14;
  glm::vec3 boxHalf_{0.05f, 0.10f, 0.05f};
  float mass_ = 1.0f;

  float jointCompliance_ = 0.0f;
  float lrcCompliance_ = 0.0f;

  float jointLimitDeg_ = 25.0f;
};
