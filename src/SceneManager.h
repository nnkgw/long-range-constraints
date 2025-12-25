#pragma once

#include <vector>
#include <memory>
#include <cstddef>
#include <utility>
#include "IScene.h"

// Minimal scene manager for Step 0.
// Add more scenes later and switch by index.
class SceneManager {
public:
  SceneManager() = default;

  template <class TScene, class... Args>
  TScene* addScene(Args&&... args) {
    std::unique_ptr<TScene> p(new TScene(std::forward<Args>(args)...));
    TScene* raw = p.get();
    scenes_.push_back(std::move(p));
    if (active_ == static_cast<std::size_t>(-1)) active_ = 0;
    return raw;
  }

  IScene* active() {
    if (active_ == static_cast<std::size_t>(-1) || active_ >= scenes_.size()) return nullptr;
    return scenes_[active_].get();
  }

  const IScene* active() const {
    if (active_ == static_cast<std::size_t>(-1) || active_ >= scenes_.size()) return nullptr;
    return scenes_[active_].get();
  }

  std::size_t count() const { return scenes_.size(); }
  std::size_t activeIndex() const { return active_; }

  void setActiveIndex(std::size_t idx) {
    if (idx < scenes_.size()) active_ = idx;
  }

private:
  std::vector<std::unique_ptr<IScene>> scenes_;
  std::size_t active_ = static_cast<std::size_t>(-1);
};
