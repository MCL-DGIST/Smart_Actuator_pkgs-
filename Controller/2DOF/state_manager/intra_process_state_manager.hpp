/** @file intra_process_state_manager.hpp
 *  @brief Data structure for intra-process communication between multiple threads
 *  @author Jeongwoo Hong (jwhong1209@gmail.com)
 */

#ifndef INTRA_PROCESS_STATE_MANAGER_HPP_
#define INTRA_PROCESS_STATE_MANAGER_HPP_

#include <cstring>
#include <stdexcept>

#include "ecat_data.hpp"
#include <atomic>
#include <evl/clock.h>
#include <evl/mutex.h>

#include "double_buffer.hpp"
/**
 * @brief RAII wrapper for EVL mutex locking/unlocking
 */
// class ScopedLock {
// private:
//   struct evl_mutex *mutex_;
//   bool b_mutex_locked_ = false;

// public:
//   ScopedLock(evl_mutex *mutex) : mutex_(mutex), b_mutex_locked_(false) {
//     if (evl_lock_mutex(mutex_) != 0) throw std::runtime_error("Failed to acquire mutex lock");

//     b_mutex_locked_ = true;
//   }
//   ~ScopedLock() {
//     if (b_mutex_locked_) evl_unlock_mutex(mutex_);
//   }
// };
/**
 * @brief State Manager for multi-threading within a (single) controller process
 * Optimized for high-frequency real-time operations
 */
class IntraProcessStateManager {
private:
  struct evl_mutex mtx_{}; // EVL mutex for thread synchronization
  ElmoPtwiDriver elmo_[SLAVES_NUM];
  AnybusCommunicator anybus_communicator_;
  EthercatStatus ecat_;

  timespec mtx_timeout_; // to prevent dead-lock

  bool inited_{false}; // to prevent re-initialization
  IntraProcessStateManager() = default;

public:
  ~IntraProcessStateManager() { evl_close_mutex(&mtx_); }

  //* ----- Singleton Pattern ----------------------------------------------------------------------
  IntraProcessStateManager(const IntraProcessStateManager &) = delete;
  IntraProcessStateManager &operator=(const IntraProcessStateManager &) = delete;
  IntraProcessStateManager(IntraProcessStateManager &&) = delete;
  IntraProcessStateManager &operator=(const IntraProcessStateManager &&) = delete;

  static IntraProcessStateManager &getInstance() {
    static IntraProcessStateManager instance;
    return instance;
  }

  void init() {
    static std::atomic<bool> inited{false};
    if (inited.load(std::memory_order_acquire)) return;
    // 1) attach 이후에만 호출되어야 함 (이미 main에서 보장)
    // 2) 이름 충돌 방지를 위해 PID를 붙임
    char namebuf[64];
    snprintf(namebuf, sizeof(namebuf), "IPSManager:%d", getpid());

    // 3) 이미 있으면 열기
    int fd = evl_open_mutex(&mtx_, "%s", namebuf);
    if (fd >= 0) {
      // 성공적으로 열었으면 재사용
      inited.store(true, std::memory_order_release);
      return;
    }

    // 4) 없으면 새로 만들기 (매크로가 내부 인자 정합성까지 챙겨줌)
    int rc = evl_new_mutex(&mtx_, "%s", namebuf);
    if (rc < 0) {
      // 혹시 경합 중이었으면 한 번 더 open 재시도
      int fd2 = evl_open_mutex(&mtx_, "%s", namebuf);
      if (fd2 >= 0) {
        inited.store(true, std::memory_order_release);
        return;
      }

      // 진짜 실패 — 원인 로깅
      std::fprintf(stderr, "[IPS] mutex open/create failed: %s (%d)\n", std::strerror(-rc), -rc);
      throw std::runtime_error("evl_created_mutex failed");
    }

    inited.store(true, std::memory_order_release);
  }

  //* ----- GETTER ---------------------------------------------------------------------------------
  ElmoPtwiDriver getElmoDriver(int slave_idx) {
    // if (pthread_mutex_timedlock(&mtx_, &mtx_timeout_) != 0)
    //   throw std::runtime_error("Failed to acquire mutex lock");
    // ElmoPtwiDriver driver = elmo_[slave_idx]; // ! mutex may not be unlock when exception happend
    // pthread_mutex_unlock(&mtx_);

    if (slave_idx < 0 || slave_idx >= SLAVES_NUM) throw std::out_of_range("Invalid slave index");

    ScopedLock lock(&mtx_);
    return elmo_[slave_idx];
  }

  AnybusCommunicator getAnybusCommunicator() {
    ScopedLock lock(&mtx_);
    return anybus_communicator_;
  }

  EthercatStatus getEcatStatus() {
    ScopedLock lock(&mtx_);
    return ecat_;
  }

  //* ----- SETTER ---------------------------------------------------------------------------------
  void setElmoDriver(int slave_idx, const ElmoPtwiDriver &driver) {
    if (slave_idx < 0 || slave_idx >= SLAVES_NUM) throw std::out_of_range("Invalid slave index");

    ScopedLock lock(&mtx_);
    elmo_[slave_idx] = driver;
  }

  void setAnybusCommunicator(const AnybusCommunicator &anybus) {
    ScopedLock lock(&mtx_);
    anybus_communicator_ = anybus;
  }

  void setEcatStatus(const EthercatStatus &ecat) {
    ScopedLock lock(&mtx_);
    ecat_ = ecat;
  }
};

#endif // INTRA_PROCESS_STATE_MANAGER_HPP_
