/*
 * Copyright 2018 The Cartographer Authors
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

#include "cartographer/common/task.h"

namespace cartographer {
namespace common {

Task::~Task() {
  // TODO(gaschler): Relax some checks after testing.
  if (state_ != NEW && state_ != COMPLETED) {
    LOG(WARNING) << "Delete Task between dispatch and completion.";
  }
}

// 获取任务状态
/*  
  初始化:                       NEW
  加入到thread_pool的准备容器中:  DISPATCHED
  加入到任务队列中:               DEPENDENCIES_COMPLETED
  开始执行:                     RUNNING
  执行完成:                     COMPLETED
*/ 
Task::State Task::GetState() {
  absl::MutexLock locker(&mutex_);
  return state_;
}

/*
  设置任务执行函数
    work_item = [this]() { DrainWorkQueue(); }
*/ 
void Task::SetWorkItem(const WorkItem& work_item) {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(state_, NEW);
  work_item_ = work_item;
}

/*
添加依赖,例如:
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
    constraint_task.state_ = NEW
    dependency = scan_matcher->creation_task_handle 匹配器初始化任务

    constraint_task.uncompleted_dependencies_++;
*/ 
void Task::AddDependency(std::weak_ptr<Task> dependency) {
  std::shared_ptr<Task> shared_dependency;
  {
    absl::MutexLock locker(&mutex_);
    CHECK_EQ(state_, NEW);
    /*
      dependency.lock() 操作:
        dependency 还存在,返回 std::shared_ptr<Task> 指针;
        dependency 不存在,返回 返回一个空指针
    */ 
    if ((shared_dependency = dependency.lock())) {
      ++uncompleted_dependencies_;
    }
  }
  if (shared_dependency) {
    shared_dependency->AddDependentTask(this);
  }
}

// 给任务设置 线程
void Task::SetThreadPool(ThreadPoolInterface* thread_pool) {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(state_, NEW);
  state_ = DISPATCHED;
  thread_pool_to_notify_ = thread_pool;
  if (uncompleted_dependencies_ == 0) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    thread_pool_to_notify_->NotifyDependenciesCompleted(this);
  }
}

/*
  shared_dependency->AddDependentTask(this);
    shared_dependency 为依赖任务
    this = dependent_task 主任务
*/ 
void Task::AddDependentTask(Task* dependent_task) {
  absl::MutexLock locker(&mutex_);
  if (state_ == COMPLETED) {
    // 依赖任务完成,调用
    dependent_task->OnDependenyCompleted();
    return;
  }
  /*
    std::set<>::insert()
      返回:二元组(pair)
        .first = 新元素的迭代器
        .second = bool(插入成功,返回true;已经存在,返回 false)
    依赖任务中的dependent_tasks_,插入主任务 dependent_task,以来任务执行完了,调用
  */
  bool inserted = dependent_tasks_.insert(dependent_task).second;
  CHECK(inserted) << "Given dependency is already a dependency.";
}

// 依赖任务执行完成后,主任务调用
void Task::OnDependenyCompleted() {
  absl::MutexLock locker(&mutex_);
  CHECK(state_ == NEW || state_ == DISPATCHED);
  //待执行的依赖任务数量
  --uncompleted_dependencies_;
  if (uncompleted_dependencies_ == 0 && state_ == DISPATCHED) {
    // 依赖完成
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);

    // 将任务,压入任务队列中
    thread_pool_to_notify_->NotifyDependenciesCompleted(this);
  }
}

// 执行任务
void Task::Execute() {
  {
    absl::MutexLock locker(&mutex_);
    CHECK_EQ(state_, DEPENDENCIES_COMPLETED);
    state_ = RUNNING;
  }

  // Execute the work item.
  if (work_item_) {
    work_item_();
  }

  absl::MutexLock locker(&mutex_);
  state_ = COMPLETED;
  // 依赖任务执行完成后,调用其主任务
  for (Task* dependent_task : dependent_tasks_) {
    dependent_task->OnDependenyCompleted();
  }
}

}  // namespace common
}  // namespace cartographer
