#pragma once
#include <memory>
#include <vector>

#include "IScene.h"

class SceneManager {
public:
  SceneManager() = default;
  ~SceneManager() = default;

  SceneManager(const SceneManager&) = delete;
  SceneManager& operator=(const SceneManager&) = delete;

  void add(std::unique_ptr<IScene> scene) {
    scenes_.push_back(std::move(scene));
    if (active_ < 0) active_ = 0;
  }

  int count() const { return (int)scenes_.size(); }

  IScene* active() {
    if (active_ < 0 || active_ >= (int)scenes_.size()) return nullptr;
    return scenes_[active_].get();
  }

  const IScene* active() const {
    if (active_ < 0 || active_ >= (int)scenes_.size()) return nullptr;
    return scenes_[active_].get();
  }

  int activeIndex() const { return active_; }

  bool setActive(int idx) {
    if (idx < 0 || idx >= (int)scenes_.size()) return false;
    active_ = idx;
    if (active()) {
      active()->reset();
      active()->usage();
    }
    return true;
  }

private:
  std::vector<std::unique_ptr<IScene>> scenes_;
  int active_ = -1;
};
