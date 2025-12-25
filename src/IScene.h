#pragma once

class IScene {
public:
  virtual ~IScene() {}

  virtual void reset() = 0;
  virtual void render() = 0;

  // Called from GLUT idle callback.
  virtual void idle() = 0;

  // Forwarded GLUT callbacks.
  virtual void reshape(int w, int h) = 0;
  virtual void onMouseButton(int button, int state, int x, int y) = 0;
  virtual void onMouseMotion(int x, int y) = 0;
  virtual void onKeyboard(unsigned char key, int x, int y) = 0;

  // Optional: print controls/help.
  virtual void usage() = 0;
};
