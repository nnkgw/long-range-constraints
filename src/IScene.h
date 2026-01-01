#pragma once

class IScene {
public:
  virtual ~IScene() = default;

  virtual void reset() = 0;
  virtual void display() = 0;
  virtual void reshape(int w, int h) = 0;
  virtual void idle() = 0;
  virtual void keyboard(unsigned char key, int x, int y) = 0;
  virtual void mouse(int button, int state, int x, int y) = 0;
  virtual void motion(int x, int y) = 0;
  virtual const char* name() const = 0;
  virtual void usage() const = 0;
};
