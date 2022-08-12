#ifndef DATATHREAD_H
#define DATATHREAD_H

#include <mutex>
#include <ctime>
#include <thread>
#include <condition_variable>

template <typename T>
struct DataThread{

  std::mutex mutex_;
  std::queue<T> data_queue_;
  std::condition_variable cv_;
  std::thread thread_;
  bool active_;

  DataThread() : active_(true){}

  void push(T data){
    mutex_.lock();
    data_queue_.push(data);
    mutex_.unlock();
  }

  T pop(){
    T result;
    mutex_.lock();
    result = data_queue_.front();
    data_queue_.pop();
    mutex_.unlock();
    return result;
  }

//  virtual void DataProcess(T data){

//  }

//  void DataProcessThread(){
//    while(1){
//      std::unique_lock<std::mutex> ul(mutex_);
//      cv_.wait(ul);
//      if(active_ == false) return;
//      ul.unlock();

//      while(!data_queue_.empty()){
//        auto data = pop();
//        //process
//        DataProcess(data);
//      }
//    }
//  }

};

#endif // DATATHREAD_H
