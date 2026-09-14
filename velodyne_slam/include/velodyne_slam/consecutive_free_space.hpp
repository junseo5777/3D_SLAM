#pragma once

#include <cstddef>

namespace velodyne_slam
{
// 대응 광선이나 오도메트리가 없어도 스캔 번호는 계속 증가한다.
// 모든 지도 복셀을 순회하지 않고 다음 관측 시 번호 간격으로 연속 횟수를 초기화한다.
class ConsecutiveFreeSpace
{
public:
  bool observe(std::size_t scan, std::size_t threshold)
  {
    if (count_ != 0 && scan == last_scan_) {return count_ >= threshold;}
    if (count_ != 0 && scan > last_scan_ && scan - last_scan_ == 1) {
      if (count_ < threshold) {++count_;}
    } else {
      count_ = 1;
    }
    last_scan_ = scan;
    return count_ >= threshold;
  }

  void reset() {count_ = 0;}

private:
  std::size_t last_scan_ = 0;
  std::size_t count_ = 0;
};
}  // velodyne_slam 네임스페이스
