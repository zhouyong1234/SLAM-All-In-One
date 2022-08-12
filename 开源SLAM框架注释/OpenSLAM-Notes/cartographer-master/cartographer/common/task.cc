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
// 析构函数，误删记录
Task::~Task() {
  // TODO(gaschler): Relax some checks after testing.
  // 在任务没有完成时就删除，会添加到log文件中
  if (state_ != NEW && state_ != COMPLETED) {
    LOG(WARNING) << "Delete Task between dispatch and completion.";
  }
}

// 获取当前任务的状态
Task::State Task::GetState() {
  absl::MutexLock locker(&mutex_);
  return state_;
}

// 设置处理该任务的函数 work_item 
void Task::SetWorkItem(const WorkItem& work_item) {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(state_, NEW);
  work_item_ = work_item;// 赋值
}

// 添加约束项
void Task::AddDependency(std::weak_ptr<Task> dependency) {
  std::shared_ptr<Task> shared_dependency;
  {
    absl::MutexLock locker(&mutex_);
    CHECK_EQ(state_, NEW);
    if ((shared_dependency = dependency.lock())) {
      ++uncompleted_dependencies_;
    }
  }
  // 如果是相互依赖
  if (shared_dependency) {
    shared_dependency->AddDependentTask(this);
  }
}

// 设置线程池
void Task::SetThreadPool(ThreadPoolInterface* thread_pool) {
  // lock 
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(state_, NEW);
  state_ = DISPATCHED;
  thread_pool_to_notify_ = thread_pool;// 赋值
  if (uncompleted_dependencies_ == 0) {
  	// 没有未完成的依赖
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    // 调用  NotifyDependenciesCompleted 函数，标记依赖项全部完成
    thread_pool_to_notify_->NotifyDependenciesCompleted(this);
  }
}
// 添加依赖任务
void Task::AddDependentTask(Task* dependent_task) {
  absl::MutexLock locker(&mutex_);
  if (state_ == COMPLETED) {
  	// 当前任务完成的情况下，调用 OnDependenyCompleted 函数 
    dependent_task->OnDependenyCompleted();
    return;
  }
  // 当前任务没有完成，插入到依赖任务中
  bool inserted = dependent_tasks_.insert(dependent_task).second;
  CHECK(inserted) << "Given dependency is already a dependency.";
}

// 添加依赖任务时调用
void Task::OnDependenyCompleted() {
  // lock 
  absl::MutexLock locker(&mutex_);
  CHECK(state_ == NEW || state_ == DISPATCHED);
  --uncompleted_dependencies_;// 未完成的依赖项数目-1
  if (uncompleted_dependencies_ == 0 && state_ == DISPATCHED) {
    state_ = DEPENDENCIES_COMPLETED;
    CHECK(thread_pool_to_notify_);
    // 标志当前已经完成所有依赖项
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
    work_item_();// 执行函数
  }

  absl::MutexLock locker(&mutex_);
  state_ = COMPLETED;// 状态完成
  for (Task* dependent_task : dependent_tasks_) {
  	// 依赖项标志为完成
    dependent_task->OnDependenyCompleted();
  }
}

}  // namespace common
}  // namespace cartographer
