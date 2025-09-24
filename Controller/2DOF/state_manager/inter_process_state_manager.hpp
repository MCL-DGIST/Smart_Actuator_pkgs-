/**
 * @file inter_process_state_manager.hpp
 * @brief Data structure for Inter-process communication using POSIX shared memory
 * @author Jeongwoo Hong (jwhong1209@gmail.com)
 */

#ifndef INTER_PROCESS_STATE_MANAGER_HPP_
#define INTER_PROCESS_STATE_MANAGER_HPP_

#include <cstdio>      // system
#include <cstdlib>     // getenv
#include <cstring>     // memset
#include <fcntl.h>     // O_CREAT, O_RDWR for shm_open
#include <iostream>    // std::cout
#include <memory>      // std::unique_ptr
#include <semaphore.h> // sem_t, sem_open, sem_wait, sem_post
#include <stdexcept>   // std::runtime_error, std::out_of_range
#include <string>      // std::string
#include <sys/mman.h>  // shm_open, mmap, munmapop
#include <sys/stat.h>  // S_IRUSR, S_IWUSR
#include <time.h>      // clock_gettime, timespec
#include <unistd.h>    // ftruncate, close
#include <vector>      // std::vector

#include "ecat_data.hpp"
#include "process_shared_data.hpp"

// Shared Memory, semaphore의 이름. (문자열) 
#define SHM_NAME "/elmo_shared_memory" 
#define SEM_NAME "/elmo_semaphore"

enum class InterfaceMode // Interface Combobox in GUI
{
  SIM = 0, // simulation mode
  REAL,    // real robot mode
};

enum class TestMode // Test Tab in GUI
{
  IDLE = 0, // none
  AXIS,     // test each axis of servo driver (position, velocity, torque)
  CTRL,     // test implemented control algorithm
};

/**
 * @brief Shared data structure for inter-process communication
 * @note Data is defined as a struct for easy access and modification
 * * Matches data structure in controller & ROS node processes
 * TODO : Distinguish Read & Write data
 */

// Shared Memory 변수들
 typedef struct shared_data {
  ElmoPtwiDriver elmo[SLAVES_NUM]; // control/statusword, ...

  Actuator_DATA actuator;
  GUI_Command gui_cmd;
  Control_DATA control;

  /* Publish to GUI */
  EthercatStatus ecat; // working counter

  bool f_test;
  double test_sampling_period_ms;
  
  bool b_ecat_thread_started;
  bool b_elmo_thread_started;
  bool b_ctrl_thread_started;
  bool b_ecat_loop_running;
  bool b_elmo_loop_running;
  bool b_ctrl_loop_running;
  bool b_system_ready;

  /* real-time performance metrics */
  double ecat_sampling_period_ms;
  double ecat_jitter_ms;
  uint32_t ecat_overrun_cnt;
  double ecat_loop_time_sec;

  double elmo_sampling_period_ms;
  double elmo_jitter_ms;
  uint32_t elmo_overrun_cnt;
  double elmo_loop_time_sec;

  double ctrl_sampling_period_ms;
  double ctrl_jitter_ms;
  uint32_t ctrl_overrun_cnt;
  double ctrl_loop_time_sec;

  /* controller states, you want to log and monitor */
  double joint_pos_mes[SLAVES_NUM];
  double joint_vel_mes[SLAVES_NUM];
  double joint_tau_mes[SLAVES_NUM];

  double joint_pos_des[SLAVES_NUM];
  double joint_vel_des[SLAVES_NUM];
  double joint_tau_des[SLAVES_NUM];

  double motor_position_rad[SLAVES_NUM];
  double motor_velocity_rad[SLAVES_NUM];
  // double motor_tau[SLAVES_NUM];

  uint64_t last_update_timestamp;

  /* Subscribe from GUI */
  double joint_pos_target[SLAVES_NUM];
  double joint_vel_target[SLAVES_NUM];
  double joint_tau_target[SLAVES_NUM];

  InterfaceMode interface_mode;
  TestMode test_mode;

  bool b_all_joints_enabled;
  bool b_joint_enabled[SLAVES_NUM];
  bool b_joints_target_cmd_on[SLAVES_NUM];
  bool b_control_input_started; // checkbox
  bool b_reference_started;
  bool b_estop_button_pressed; // * this is ESTOP by push button, not by DO data (STO)
  // bool b_estop_switch_pressed;  // * this is ESTOP by STO switch

  int ref_sine_repeat;
  double ref_sine_amp;
  double ref_sine_freq;

  double joint_kp[SLAVES_NUM];
  double joint_kd[SLAVES_NUM];

} SharedData_process;

/**
 * @brief Inter-Process State Manager using shared memory and semaphore
 */
class InterProcessStateManager {
private:
  int shm_fd_;              // shared memory file descriptor
  sem_t *semaphore_;        // semaphore for synchronization
  bool is_creator_;         // true if this instance created the shared memory
  SharedData_process *shared_data_; // pointer to shared memory

public:
  class SafeAccessor;        // forward declaration for friend
  friend class SafeAccessor; // allow SafeAccessor to access private members

  /**
   * @brief Constructor
   * @param create_memory True = create shared memory, False = open existing one
   */
  explicit InterProcessStateManager(bool create_memory = false) : is_creator_(create_memory) {
    if (create_memory) {
      this->createSharedMemory(); // create shared memory (e.g. callback at controller process)
      std::cout << "[StateManager] Shared memory created successfully!" << std::endl;
    }
    else {
      this->openSharedMemory(); // access to shared memory (e.g.callback at ROS node process)
      std::cout << "[StateManager] Shared memory opened successfully!" << std::endl;
    }

    this->initSemaphore(); // initialize & open semaphore
    std::cout << "[StateManager] Semaphore initialized successfully!" << std::endl;
    std::cout << "[StateManager] SharedData_process is safely encapsulated with SafeAccessor!" << std::endl;
  }

  ~InterProcessStateManager() { this->cleanup(); }

  //* ----- SINGLETON PATTERN ---------------------------------------------------------------------
  InterProcessStateManager(const InterProcessStateManager &) = delete;
  InterProcessStateManager &operator=(const InterProcessStateManager &) = delete;

  static InterProcessStateManager &getInstance(bool creator = false) {
    static std::unique_ptr<InterProcessStateManager> instance;

    if (!instance)
      instance = std::unique_ptr<InterProcessStateManager>(new InterProcessStateManager(creator));

    return *instance;
  }

  //* ----- SIMPLE LOCKING METHODS (for direct access) ---------------------------------------------
  /** @brief Acquire semaphore lock for safe data access */
  void lock() { sem_wait(semaphore_); }

  /** @brief Release semaphore lock */
  void unlock() { sem_post(semaphore_); }

  /** @brief Update timestamp (call after modifying data) */
  void updateTimestamp() { shared_data_->last_update_timestamp = getCurrentTimestamp(); }

  //* ----- GETTER ---------------------------------------------------------------------------------
  //* Getter methods are called by ROS node process

  /** @brief Check system ready status (with automatic locking) */
  // bool isSystemReady()
  // {
  //   lock();
  //   bool ready = shared_data_->b_system_ready;
  //   unlock();
  //   return ready;
  // }

  /** @brief Get EtherCAT status */
  EthercatStatus getEcatStatus() {
    lock();
    EthercatStatus status = shared_data_->ecat;
    unlock();
    return status;
  }

  /** @brief Get complete ELMO driver data */
  ElmoPtwiDriver getElmoDriver(int slave_idx) {
    if (slave_idx < 0 || slave_idx >= SLAVES_NUM) throw std::out_of_range("Invalid slave index");

    lock();
    ElmoPtwiDriver driver = shared_data_->elmo[slave_idx];
    unlock();
    return driver;
  }

  //* ----- DIRECT ACCESS METHODS (for efficient data copying) -----------------------------------

  /** @brief Get direct access to shared data for bulk operations
   *  @warning Use with caution! Must handle semaphore manually
   */
  SharedData_process *getSharedData_processPtr() { return shared_data_; }
  sem_t *getSemaphore() { return semaphore_; }

  //* ----- SAFE DIRECT ACCESS PATTERN (Best of both worlds) ---------------------------------------

  /**
   * @brief RAII pattern-based Smart Accessor for safe direct access
   * @note Automatically handles lock/unlock while allowing direct access
   */
  class SafeAccessor {
  private:
    InterProcessStateManager &manager_;
    bool is_locked_;

  public:
    explicit SafeAccessor(InterProcessStateManager &mgr) : manager_(mgr), is_locked_(true) {
      manager_.lock(); // automatic lock at constructor
    }

    /** @brief Move constructor
     * @note This constructor is used to move SafeAccessor object
     * @param other SafeAccessor object to move from
     */
    SafeAccessor(SafeAccessor &&other) noexcept
        : manager_(other.manager_), is_locked_(other.is_locked_) {
      other.is_locked_ = false; // remove lock responsibility from original object
    }

    ~SafeAccessor() {
      if (is_locked_) {
        manager_.updateTimestamp(); // update timestamp
        manager_.unlock();          // automatic unlock at destructor
      }
    }

    /** @brief Safe access to private shared_data_ */
    SharedData_process *operator->() { return manager_.shared_data_; }
    SharedData_process &operator*() { return *manager_.shared_data_; }

    /* prevent copy and assignment, only move is allowed */
    SafeAccessor(const SafeAccessor &) = delete;
    SafeAccessor &operator=(const SafeAccessor &) = delete;
    SafeAccessor &operator=(SafeAccessor &&) = delete;
  };

  /**
   * @brief Safe direct access method
   * @example auto accessor = shm_manager_.getSafeAccess();
   *          accessor->motor_position_rad[0] = value;
   */
  SafeAccessor getSafeAccess() { return SafeAccessor(*this); }

  //* ----- CONVENIENT BULK OPERATIONS (optimization) ----------------------------------------------

  /**
   * @brief Copy array at once
   * @param motor_pos_rad motor position array
   * @param motor_vel_rad motor velocity array
   */
  // void setMotorArraysSafe(const double motor_pos_rad[SLAVES_NUM],
  //                         const double motor_vel_rad[SLAVES_NUM])
  // {
  //   lock();
  //   memcpy(shared_data_->motor_position_rad, motor_pos_rad, sizeof(double) * SLAVES_NUM);
  //   memcpy(shared_data_->motor_velocity_rad, motor_vel_rad, sizeof(double) * SLAVES_NUM);
  //   updateTimestamp();
  //   unlock();
  // }

  /**
   * @brief Copy controller data struct
   * template<typename ControllerState>
   * void setControllerState(const ControllerState& state);
   */

private:
  void createSharedMemory() {
    shm_unlink(SHM_NAME); // Remove existing shared memory

    /* Set umask to allow group/other access */
    mode_t old_umask = umask(0); // Temporarily set umask to 0

    /* Create shared memory */
    // shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, S_IRUSR | S_IWUSR);
    shm_fd_ = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666); // user authorization for read/write

    /* Restore original umask */
    umask(old_umask);

    if (shm_fd_ == -1) throw std::runtime_error("Failed to create shared memory");

    /* Set shared memory size */
    if (ftruncate(shm_fd_, sizeof(SharedData_process)) == -1) {
      close(shm_fd_);
      shm_unlink(SHM_NAME);
      throw std::runtime_error("Failed to set shared memory size");
    }

    /* Change file permissions explicitly (needed when created by root) */
    if (fchmod(shm_fd_, 0666) == -1)
      std::cout << "[StateManager] Warning: Failed to set file permissions 0666" << std::endl;

    /* Change ownership to real user (if running with sudo) */
    const char *sudo_user = getenv("SUDO_USER");
    if (sudo_user) {
      std::string chown_cmd =
          "chown " + std::string(sudo_user) + ":" + std::string(sudo_user) + " /dev/shm" + SHM_NAME;
      std::cout << "[StateManager] Changing ownership: " << chown_cmd << std::endl;
      int result = system(chown_cmd.c_str());

      if (result != 0)
        std::cout << "[StateManager] Warning: Failed to change ownership as sudo_user" << std::endl;
    }

    /* Memory mapping */ // shared_data_를 shared memory에 등록 하는 코드
    shared_data_ = static_cast<SharedData_process *>(
        mmap(nullptr, sizeof(SharedData_process), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));

    if (shared_data_ == MAP_FAILED) {
      close(shm_fd_);
      shm_unlink(SHM_NAME);
      throw std::runtime_error("Failed to map shared memory");
    }

    //* ----- Initialize shared data ---------------------------------------------------------------
    memset(shared_data_, 0, sizeof(SharedData_process)); // First zero out everything

    /* Set non-zero initial values */
    shared_data_->interface_mode = InterfaceMode::REAL;
    shared_data_->test_mode = TestMode::IDLE;
    shared_data_->last_update_timestamp = getCurrentTimestamp();
  }

  void openSharedMemory() {
    shm_fd_ = shm_open(SHM_NAME, O_RDWR, 0);
    if (shm_fd_ == -1) throw std::runtime_error("Failed to open shared memory");

    /* Memory mapping */
    shared_data_ = static_cast<SharedData_process *>(
        mmap(nullptr, sizeof(SharedData_process), PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0));

    if (shared_data_ == MAP_FAILED) {
      close(shm_fd_);
      throw std::runtime_error("Failed to map shared memory");
    }
  }

  void initSemaphore() {
    if (is_creator_) // shared memory creator (e.g. controller process)
    {
      sem_unlink(SEM_NAME); // Remove existing semaphore

      mode_t old_umask = umask(0); // set umask 0 temporarily to allow all permissions
      semaphore_ = sem_open(SEM_NAME, O_CREAT | O_EXCL, 0666, 1);
      umask(old_umask); // restore original umask

      if (semaphore_ == SEM_FAILED) { // open existing semaphore if O_EXCL failed
        semaphore_ = sem_open(SEM_NAME, O_CREAT, 0666, 1);

        if (semaphore_ == SEM_FAILED) {
          std::cout << "[StateManager] Semaphore creation failed: " << strerror(errno) << std::endl;
          throw std::runtime_error("Failed to create semaphore");
        }
      }

      /* check if semaphore exists at /dev/shm */
      std::string sem_file = "/dev/shm/sem." + std::string(SEM_NAME).substr(1);
      // std::cout << "[StateManager] Expected semaphore file: " << sem_file << std::endl;

      /* check if semaphore file exists and has correct permissions */
      struct stat st;
      if (stat(sem_file.c_str(), &st) == 0) {
        std::cout << "[StateManager] Semaphore file exists with permissions: " << std::oct
                  << (st.st_mode & 0777) << std::dec << std::endl;
        std::cout << "[StateManager] Owner UID: " << st.st_uid << ", GID: " << st.st_gid
                  << std::endl;
      }
      else
        std::cout << "[StateManager] Semaphore file not visible in filesystem" << std::endl;

      /* Change ownership to real user (if running with sudo) */
      const char *sudo_user = getenv("SUDO_USER");
      const char *sudo_uid = getenv("SUDO_UID");
      const char *sudo_gid = getenv("SUDO_GID");

      if (sudo_user && sudo_uid && sudo_gid) {
        std::cout << "[StateManager] Changing ownership to " << sudo_user << " (UID: " << sudo_uid
                  << ", GID: " << sudo_gid << ")" << std::endl;

        /* set permissions and ownership */
        std::string chmod_cmd = "chmod 666 " + sem_file + " 2>/dev/null";
        std::string chown_cmd = "chown " + std::string(sudo_uid) + ":" + std::string(sudo_gid) +
                                " " + sem_file + " 2>/dev/null";
        system(chmod_cmd.c_str());
        system(chown_cmd.c_str());
      }
    }
    else // shared memory user (e.g. ROS node process)
    {
      std::cout << "[StateManager] Opening existing semaphore '" << SEM_NAME << "'..." << std::endl;

      semaphore_ = sem_open(SEM_NAME, 0); // try to open semaphore

      if (semaphore_ == SEM_FAILED) {
        usleep(100'000); // wait for 100ms

        semaphore_ = sem_open(SEM_NAME, O_RDWR);
        if (semaphore_ == SEM_FAILED) {
          std::cout << "[StateManager] Failed to open semaphore: " << strerror(errno)
                    << " (errno: " << errno << ")" << std::endl;

          std::cout << "[StateManager] Debug info:" << std::endl;
          std::cout << "  Current UID: " << getuid() << ", GID: " << getgid() << std::endl;
          std::cout << "  Current EUID: " << geteuid() << ", EGID: " << getegid() << std::endl;

          /* check sem_file status */
          std::string sem_file = "/dev/shm/sem" + std::string(SEM_NAME).substr(1);
          struct stat st;
          if (stat(sem_file.c_str(), &st) == 0) {
            std::cout << "  Semaphore file exists, permissions: " << std::oct << (st.st_mode & 0777)
                      << std::dec << std::endl;
            std::cout << "  File owner UID: " << st.st_uid << ", GID: " << st.st_gid << std::endl;
          }
          else
            std::cout << "  Semaphore file not found: " << sem_file << std::endl;

          std::cout << "[StateManager] Failed to open semaphore: " << strerror(errno) << std::endl;
          throw std::runtime_error("Failed to open semaphore");
        }
      }
    }

    /* test semaphore functionality */
    // int sem_value;
    // if (sem_getvalue(semaphore_, &sem_value) == 0)
    //   std::cout << "[StateManager] Semaphore value: " << sem_value << std::endl;
  }

  uint64_t getCurrentTimestamp() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1'000'000'000ULL + ts.tv_nsec;
  }

  void cleanup() {
    if (shared_data_ != nullptr) munmap(shared_data_, sizeof(SharedData_process));

    if (shm_fd_ != -1) close(shm_fd_);

    if (semaphore_ != SEM_FAILED) sem_close(semaphore_);

    if (is_creator_) {
      shm_unlink(SHM_NAME);
      sem_unlink(SEM_NAME);
    }
  }
};

#endif // INTER_PROCESS_STATE_MANAGER_HPP_
