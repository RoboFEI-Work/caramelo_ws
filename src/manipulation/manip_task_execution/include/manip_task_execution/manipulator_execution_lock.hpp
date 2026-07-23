#pragma once

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstring>
#include <string>

namespace manip_task_execution
{

class ManipulatorExecutionLock
{
public:
  explicit ManipulatorExecutionLock(const std::string & lock_file)
  : lock_file_(lock_file),
    fd_(::open(lock_file_.c_str(), O_CREAT | O_RDWR, 0666))
  {
  }

  ~ManipulatorExecutionLock()
  {
    release();
    if (fd_ >= 0) {
      ::close(fd_);
    }
  }

  ManipulatorExecutionLock(const ManipulatorExecutionLock &) = delete;
  ManipulatorExecutionLock & operator=(const ManipulatorExecutionLock &) = delete;

  bool valid() const
  {
    return fd_ >= 0;
  }

  const std::string & lockFile() const
  {
    return lock_file_;
  }

  std::string error() const
  {
    return std::strerror(errno);
  }

  bool tryAcquire()
  {
    bool expected = false;
    if (!locally_acquired_.compare_exchange_strong(expected, true)) {
      return false;
    }

    if (fd_ < 0 || ::flock(fd_, LOCK_EX | LOCK_NB) != 0) {
      locally_acquired_.store(false);
      return false;
    }

    return true;
  }

  void release()
  {
    if (locally_acquired_.exchange(false) && fd_ >= 0) {
      ::flock(fd_, LOCK_UN);
    }
  }

private:
  std::string lock_file_;
  int fd_;
  std::atomic_bool locally_acquired_{false};
};

}  // namespace manip_task_execution
