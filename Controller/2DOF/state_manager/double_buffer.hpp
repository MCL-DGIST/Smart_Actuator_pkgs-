#pragma once

// #include "intra_process_state_manager.hpp" // intra_process_state_manager.hpp 에 있는
// ScopedLock을 별도 파일로 분리하거나, 해당 파일을 include
#include <array>
#include <cstdio>
#include <evl/clock.h>
#include <evl/mutex.h>
#include <iostream> // for std::cout
#include <stdexcept>
#include <utility> // for std::swap

#include "ecat_data.hpp"

class ScopedLock {
private:
  struct evl_mutex *mutex_;
  bool b_mutex_locked_ = false;

public:
  ScopedLock(evl_mutex *mutex) : mutex_(mutex), b_mutex_locked_(false) {
    if (evl_lock_mutex(mutex_) != 0) throw std::runtime_error("Failed to acquire mutex lock");

    b_mutex_locked_ = true;
  }
  ~ScopedLock() {
    if (b_mutex_locked_) evl_unlock_mutex(mutex_);
  }
};

template <typename T> class DoubleBuffer {
private:
  T buffer_a_;
  T buffer_b_;

  T *read_ptr_;
  T *write_ptr_;

  mutable struct evl_mutex mtx_;
  bool is_inited_ = false; // Mutex가 초기화되었는지 여부

public:
  DoubleBuffer() : read_ptr_(&buffer_a_), write_ptr_(&buffer_b_) {}
  ~DoubleBuffer() {
    if (is_inited_) { evl_close_mutex(&mtx_); }
  }

  void init() {
    if (is_inited_) return;
    char namebuf[64];
    // [수정] snprintf 포맷 경고 해결을 위해 this를 void*로 캐스팅
    snprintf(namebuf, sizeof(namebuf), "d_buffer_mtx_%p", static_cast<void *>(this));
    if (evl_new_mutex(&mtx_, "+%s", namebuf) < 0) {
      throw std::runtime_error("Failed to create double buffer mutex");
    }
    // if (evl_new_mutex(&mtx_, EVL_CLOCK_MONOTONIC, EVL_CLONE_PUBLIC, "%s", namebuf) < 0) {
    //   throw std::runtime_error("Failed to create double buffer mutex");
    // }
    is_inited_ = true;
  }

  // 쓰기 스레드용: 쓰기 가능한 백 버퍼의 포인터를 얻는다 (잠금 없음)
  T *get_write_buffer() { return write_ptr_; }

  // 쓰기 스레드용: 쓰기를 마친 후 버퍼를 교체한다 (짧은 잠금)
  void publish() {
    ScopedLock lock(&mtx_);
    std::swap(read_ptr_, write_ptr_);
  }

  // 읽기 스레드용: 현재 읽기 가능한 프론트 버퍼의 데이터를 복사해서 얻는다 (짧은 잠금)
  T read() const {
    ScopedLock lock(const_cast<struct evl_mutex *>(&mtx_));
    return *read_ptr_;
  }
};
