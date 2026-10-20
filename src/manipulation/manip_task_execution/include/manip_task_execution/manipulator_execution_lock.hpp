// Lock de execucao do manipulador: flock(2) exclusivo num arquivo (parametro
// manipulator_lock_file, default /tmp/caramelo_manip_action.lock) para que um
// unico no (pick/place/amt1) mova o braco por vez. tryAcquire nao bloqueia;
// o errno do open fica guardado para error(). RAII: o destrutor solta o lock
// e fecha o fd.
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
    fd_(::open(lock_file_.c_str(), O_CREAT | O_RDWR, 0666)),
    open_errno_(fd_ < 0 ? errno : 0)
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

  // Erro do ::open do construtor (errno global pode ter mudado ate aqui).
  std::string error() const
  {
    return std::strerror(open_errno_);
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
  int open_errno_;
  std::atomic_bool locally_acquired_{false};
};

}  // namespace manip_task_execution
