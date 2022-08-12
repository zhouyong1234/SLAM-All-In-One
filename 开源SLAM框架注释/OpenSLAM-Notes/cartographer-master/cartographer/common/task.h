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

#ifndef CARTOGRAPHER_COMMON_TASK_H_
#define CARTOGRAPHER_COMMON_TASK_H_

#include <set>
// 线程锁使用absl库
#include "absl/synchronization/mutex.h"
#include "glog/logging.h"
#include "thread_pool.h"

namespace cartographer {
namespace common {
// 线程池中的任务类

class ThreadPoolInterface;

class Task {
 public:
  friend class ThreadPoolInterface;// 友元
  // 一个函数就是一个工作项目
  // std::function c++11的新特性，用于实现函数回调
  using WorkItem = std::function<void()>;
  // 任务状态
  // 新建、解散、依赖完成？、运行、完成
  enum State { NEW, DISPATCHED, DEPENDENCIES_COMPLETED, RUNNING, COMPLETED };
  // 默认构造
  Task() = default;
  // 析构
  ~Task();
  // 获得当前状态，函数添加了线程锁	
  State GetState() LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW'.
  // 状态为新建时，设置任务项
  void SetWorkItem(const WorkItem& work_item) LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW'. 'dependency' may be nullptr, in which case it is
  // assumed completed.
  // 状态为新建时，添加约束
  void AddDependency(std::weak_ptr<Task> dependency) LOCKS_EXCLUDED(mutex_);

 private:
  // Allowed in all states.
  // 添加依赖任务
  void AddDependentTask(Task* dependent_task);

  // State must be 'DEPENDENCIES_COMPLETED' and becomes 'COMPLETED'.
  // 执行任务
  void Execute() LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW' and becomes 'DISPATCHED' or 'DEPENDENCIES_COMPLETED'.
  // 设置线程池
  void SetThreadPool(ThreadPoolInterface* thread_pool) LOCKS_EXCLUDED(mutex_);

  // State must be 'NEW' or 'DISPATCHED'. If 'DISPATCHED', may become
  // 'DEPENDENCIES_COMPLETED'.
  void OnDependenyCompleted();

  WorkItem work_item_ GUARDED_BY(mutex_);
  ThreadPoolInterface* thread_pool_to_notify_ GUARDED_BY(mutex_) = nullptr;
  State state_ GUARDED_BY(mutex_) = NEW;
  unsigned int uncompleted_dependencies_ GUARDED_BY(mutex_) = 0;
  std::set<Task*> dependent_tasks_ GUARDED_BY(mutex_);

  absl::Mutex mutex_;
};

}  // namespace common
}  // namespace cartographer

#endif  // CARTOGRAPHER_COMMON_TASK_H_
