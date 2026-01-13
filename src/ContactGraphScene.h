#pragma once

#include <vector>

#include "ChainSceneBase.h"

// Step 0 for Section 3.6 (Contact graphs):
// - Build a small stack of axis-aligned boxes.
// - Generate simple contacts (box-floor and box-box) using AABB overlap.
// - Build a contact graph (contacts are nodes; edges connect contacts acting on the same body).
// - Classify "supporting" contacts by iteratively checking whether a body's center-of-mass projection
//   lies inside the convex hull of the projections of its static contacts below it.
//
// This is an intentionally small step meant to be buildable and easy to validate.
class ContactGraphScene final : public ChainSceneBase {
public:
  ContactGraphScene();
  ~ContactGraphScene() override = default;

  void reset() override;
  void display() override;
  void keyboard(unsigned char key, int x, int y) override;
  const char* name() const override { return "Phase D0 (Contact graphs - Step0)"; }
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

  // 2D helpers for the convex hull test in the plane orthogonal to gravity (XZ plane here).
  static float cross2(const glm::vec2& a, const glm::vec2& b);
  static float cross2(const glm::vec2& a, const glm::vec2& b, const glm::vec2& c);
  static std::vector<glm::vec2> convexHull2D(std::vector<glm::vec2> pts);
  static bool pointInConvexPolygon2D(const std::vector<glm::vec2>& poly, const glm::vec2& p);
  static bool pointOnSegment2D(const glm::vec2& a, const glm::vec2& b, const glm::vec2& p);

  float groundY_ = -0.8f;
  glm::vec3 boxHalf_{0.18f, 0.10f, 0.12f};

  std::vector<lrc::RigidBody> bodies_;
  std::vector<Contact> contacts_;

  // Contact graph edges (pairs of contact indices) for visualization.
  std::vector<std::pair<int, int>> graphEdges_;

  std::vector<bool> bodySupported_;

  bool paused_ = false;
  bool showContacts_ = true;
  bool showGraph_ = true;
};
