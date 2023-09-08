/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/common/thread_pool.h"

#ifndef WIN32
#include <unistd.h>
#endif
#include <algorithm>
#include <chrono>
#include <numeric>

#include "absl/memory/memory.h"
#include "cartographer/common/task.h"
#include "glog/logging.h"

namespace cartographer {
namespace common {

/*执行任务*/ 
void ThreadPoolInterface::Execute(Task* task) { task->Execute(); }

void ThreadPoolInterface::SetThreadPool(Task* task) {
  task->SetThreadPool(this);
}

/*
  线程池初始化
    std::vector<std::thread> pool_ GUARDED_BY(mutex_);
*/ 
ThreadPool::ThreadPool(int num_threads) {
  CHECK_GT(num_threads, 0) << "ThreadPool requires a positive num_threads!";
  absl::MutexLock locker(&mutex_);
  for (int i = 0; i != num_threads; ++i) {
    pool_.emplace_back([this]() { ThreadPool::DoWork(); });
  }
}

ThreadPool::~ThreadPool() {
  {
    absl::MutexLock locker(&mutex_);
    CHECK(running_);
    running_ = false;
  }
  for (std::thread& thread : pool_) {
    thread.join();
  }
}

// 通知依赖完成,并将任务压入 task_queue_.调用该函数,将任务放入任务队列中,然后在线程中去执行
void ThreadPool::NotifyDependenciesCompleted(Task* task) {
  absl::MutexLock locker(&mutex_);
  auto it = tasks_not_ready_.find(task);
  CHECK(it != tasks_not_ready_.end());
  task_queue_.push_back(it->second);

  // 擦除任务
  tasks_not_ready_.erase(it);
}

// 将该任务,插入 tasks_not_ready_ 中
std::weak_ptr<Task> ThreadPool::Schedule(std::unique_ptr<Task> task) {
  std::shared_ptr<Task> shared_task;
  {
    absl::MutexLock locker(&mutex_);
    auto insert_result =
        tasks_not_ready_.insert(std::make_pair(task.get(), std::move(task)));
    CHECK(insert_result.second) << "Schedule called twice";
    shared_task = insert_result.first->second;
  }
  SetThreadPool(shared_task.get());
  return shared_task;
}

/*线程的执行函数*/ 
void ThreadPool::DoWork() {
#ifdef __linux__
  // This changes the per-thread nice level of the current thread on Linux. We
  // do this so that the background work done by the thread pool is not taking
  // away CPU resources from more important foreground threads.
  CHECK_NE(nice(10), -1);
#endif
  const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return !task_queue_.empty() || !running_;
  };
  for (;;) {
    std::shared_ptr<Task> task;
    {
      absl::MutexLock locker(&mutex_);
      /*
        等待 predicate 为 True
      */ 
      mutex_.Await(absl::Condition(&predicate));
      /*
        task_queue_ 有任务,取出任务执行
      */ 
      if (!task_queue_.empty()) {
        task = std::move(task_queue_.front());
        task_queue_.pop_front();
      } 
      else if (!running_) 
      {
        // running_ == false,直接结束线程
        return;
      }
    }
    CHECK(task);
    CHECK_EQ(task->GetState(), common::Task::DEPENDENCIES_COMPLETED);
    // 执行任务
    Execute(task.get());
  }
}

}  // namespace common
}  // namespace cartographer
