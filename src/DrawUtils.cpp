#include "DrawUtils.h"

#include "GlutCompat.h"
#include <glm/gtc/type_ptr.hpp>

void drawGround(float y) {
  const float s = 5.0f;
  glBegin(GL_LINES);
  for (int i = -10; i <= 10; ++i) {
    float x = (float)i * 0.5f;
    glVertex3f(x, y, -s);
    glVertex3f(x, y,  s);
    glVertex3f(-s, y, x);
    glVertex3f( s, y, x);
  }
  glEnd();
}

void drawBoxWire(const glm::vec3& halfExtents) {
  float hx = halfExtents.x;
  float hy = halfExtents.y;
  float hz = halfExtents.z;

  glm::vec3 v[8] = {
    glm::vec3(-hx, -hy, -hz),
    glm::vec3( hx, -hy, -hz),
    glm::vec3( hx,  hy, -hz),
    glm::vec3(-hx,  hy, -hz),
    glm::vec3(-hx, -hy,  hz),
    glm::vec3( hx, -hy,  hz),
    glm::vec3( hx,  hy,  hz),
    glm::vec3(-hx,  hy,  hz)
  };

  int e[12][2] = {
    {0,1},{1,2},{2,3},{3,0},
    {4,5},{5,6},{6,7},{7,4},
    {0,4},{1,5},{2,6},{3,7}
  };

  glBegin(GL_LINES);
  for (int i = 0; i < 12; ++i) {
    glVertex3fv(glm::value_ptr(v[e[i][0]]));
    glVertex3fv(glm::value_ptr(v[e[i][1]]));
  }
  glEnd();
}
