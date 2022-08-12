#include <iostream>
#include <chrono>

class Timer
{
private:
  std::chrono::steady_clock::time_point timeStartPoint_;
public:

  /** @brief Constructor 
   * 
   */ 
  Timer(/* args */){tic();};

  /** @brief Destructor 
   * 
   */ 
  ~Timer(){};

  /** @brief set timer begin timestamp
   * 
   */ 
  inline void tic() {
    timeStartPoint_ = std::chrono::steady_clock::now();
  }

  /** @brief get time from tic and reset timer begin
   *  @return microseconds from tic 
   */ 
  inline double toc_ms() {
    std::chrono::steady_clock::time_point timeEndPoint =  std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsedTime = timeEndPoint - timeStartPoint_;
    timeStartPoint_ = timeEndPoint;
    return elapsedTime.count() * 1000;
  } 
};


