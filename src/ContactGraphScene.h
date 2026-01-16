#pragma once

#include <vector>

#include "ChainSceneBase.h"

// Step 1 for Section 3.6 (Contact graphs):
// - Build a small stack of axis-aligned boxes.
// - Generate simple contacts (box-floor and box-box) using AABB overlap.
// - Build a contact graph (contacts are nodes; edges connect contacts acting on the same body).
// - Classify "supporting" contacts by iteratively checking whether a body's center-of-mass projection
//   lies inside the convex hull of the projections of its static contacts below it.
//
// This step adds a simple static-vs-dynamic contact classification (friction cone test).
class ContactGraphScene final : public ChainSceneBase {
public:
  ContactGraphScene();
  ~ContactGraphScene() override = default;

  void reset() override;
  void display() override;
  void keyboard(unsigned char key, int x, int y) override;
  const char* name() const override { return "Phase D1 (Contact graphs - Step1)"; }
  void usage() const override;

private:
  void simulateStep(float dt) override;
  void drawSceneContents() override;
  void onFrameEnd() override;

  struct Contact {
    int upper = -1;       // Body index above (dynamic body).
    int lower = -1;       // Body index below, or -1 for floor.
    glm::vec3 p{0.0f};    // World contact point.
    glm::vec3 n{0.0f};    // Contact normal pointing from lower to upper.
    bool isStatic = true; // Static friction criterion (simplified in this step).
    bool supporting = false;
  };

  void buildScene();
  void rebuildContacts();
  void buildContactGraph();
  void computeSupportingContacts();

  void addBoxFloorContacts(int bodyIdx);
  void addBoxBoxContacts(int upperIdx, int lowerIdx);

  bool isStaticContact(int upperIdx, int lowerIdx, const glm::vec3& p, const glm::vec3& n) const;


  // 2D helpers for the convex hull test in the plane orthogonal to gravity (XZ plane here).
  static float cross2(const glm::vec2& a, const glm::vec2& b);
  static float cross2(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c);
  static std::vector<glm::vec2> convexHull2D(std::vector<glm::vec2> pts);
  static bool pointInConvexPolygon2D(const std::vector<glm::vec2>& poly, const glm::vec2& p);
  static bool pointOnSegment2D(const glm::vec2& a, const glm::vec2& b, const glm::vec2& p);

  float groundY_ = -0.8f;
  glm::vec3 boxHalf_{0.18f, 0.10f, 0.12f};
  float frictionMu_ = 0.8f;

  std::vector<lrc::RigidBody> bodies_;
  std::vector<Contact> contacts_;

  // Contact graph edges (pairs of contact indices) for visualization.
  std::vector<std::pair<int, int>> graphEdges_;

  std::vector<bool> bodySupported_;

  bool paused_ = false;
  bool showContacts_ = true;
  bool showGraph_ = true;

  // A tiny horizontal drive on the top body. This was originally used only to create
  // sliding (dynamic) contacts. In this step, the drive also moves the top body so the
  // stack can lose support and the top block can fall (easier to validate the graph).
  bool driveEnabled_ = true;
  bool driveAllowFall_ = true;
  float driveTime_ = 0.0f;
  bool topFreeFall_ = false;
  bool topOnFloor_ = false;

  // Target position drive: x_target = base_x + A * sin(2*pi*f*t).
  float drivePosAmp_ = 0.22f; // m
  float driveKp_ = 10.0f;     // 1/s

  // Velocity clamp (prevents teleporting and keeps the motion slow).
  float driveVelAmp_ = 0.10f; // m/s (small)
  float driveHz_ = 0.7f;

  glm::vec3 driveBasePos_{0.0f};

  // Fall rotation experiments:
  //  0: none
  //  1: A (kick)     - apply a small tilt + initial angular velocity when free-fall starts.
  //  2: B (torque)   - inject a weak off-center drive torque during free-fall.
  int fallRotMode_ = 1;
  float fallKickDeg_ = 8.0f;
  float fallKickOmega_ = 0.6f;
  float fallTorqueGain_ = 2.5f;
  // Cached support info used to pick a more physical fall rotation direction.
  bool fallHasPivot_ = false;
  int fallSupportLower_ = -2;
  glm::vec3 fallPivot_{0.0f};
  glm::vec3 fallAxis_{0.0f, 0.0f, 1.0f};

};
