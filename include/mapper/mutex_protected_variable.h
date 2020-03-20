// Copyright (c) 2020 by Pensa Systems, Inc. -- All rights reserved
// Confidential and Proprietary

#pragma once

#include <mutex>

namespace mapper {

// This class allows to have a mutex-protected variable
// with protected creation, reading and writing
template<typename T>
class MutexProtectedVariable {
public:
  MutexProtectedVariable() {} ;
  MutexProtectedVariable(const T &newValue) {
  	std::unique_lock<std::mutex> lock(mutex_);
    value_ = newValue;
  }

  T GetValue() {
    std::unique_lock<std::mutex> lock(mutex_);
    return value_;
  }

  void SetValue(const T &newValue) {
    std::unique_lock<std::mutex> lock(mutex_);
    value_ = newValue;
  }
private:
  std::mutex mutex_;
  T value_;
};

}  // namespace mapper
