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

// 执行任务
void ThreadPoolInterface::Execute(Task* task) { task->Execute(); }

// 往线程池中添加任务 
void ThreadPoolInterface::SetThreadPool(Task* task) {
  task->SetThreadPool(this);
}

// 线程池构造函数
// num_threads 线程数量
ThreadPool::ThreadPool(int num_threads) {
  absl::MutexLock locker(&mutex_);
  for (int i = 0; i != num_threads; ++i) {
  	// 创建线程并执行 DoWork任务
    pool_.emplace_back([this]() { ThreadPool::DoWork(); });
  }
}

// 析构函数
ThreadPool::~ThreadPool() {
  {
    absl::MutexLock locker(&mutex_);
    CHECK(running_);
    running_ = false;// 运行标志位 false 
  }
  for (std::thread& thread : pool_) {
    thread.join();// 关闭所有线程
  }
}

void ThreadPool::NotifyDependenciesCompleted(Task* task) {
  absl::MutexLock locker(&mutex_);
  auto it = tasks_not_ready_.find(task);
  CHECK(it != tasks_not_ready_.end());
  task_queue_.push_back(it->second);
  tasks_not_ready_.erase(it);
}

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

// 创建线程
void ThreadPool::DoWork() {
#ifdef __linux__
  // This changes the per-thread nice level of the current thread on Linux. We
  // do this so that the background work done by the thread pool is not taking
  // away CPU resources from more important foreground threads.
  CHECK_NE(nice(10), -1);
#endif
  // 线程锁
  const auto predicate = [this]() EXCLUSIVE_LOCKS_REQUIRED(mutex_) {
    return !task_queue_.empty() || !running_;
  };
  for (;;) {
  	// 不断循环执行，不断从 task_queue中取出任务并执行
    std::shared_ptr<Task> task;
    {
      // 线程锁
      absl::MutexLock locker(&mutex_);
      mutex_.Await(absl::Condition(&predicate));
      if (!task_queue_.empty()) {
        task = std::move(task_queue_.front());// 取出任务
        task_queue_.pop_front();// 删除任务
      } else if (!running_) {
        return;// 当前线程状态非运行
      }
    }
    CHECK(task);
    CHECK_EQ(task->GetState(), common::Task::DEPENDENCIES_COMPLETED);
    Execute(task.get());// 执行任务
  }
}

}  // namespace common
}  // namespace cartographer
