#include <gtest/gtest.h>
#include "velodyne_slam/consecutive_free_space.hpp"

using velodyne_slam::ConsecutiveFreeSpace;

TEST(ConsecutiveFreeSpace, DeletesOnlyOnFifthConsecutiveScan)
{
  ConsecutiveFreeSpace streak;
  for (std::size_t scan = 0; scan < 4; ++scan) {
    EXPECT_FALSE(streak.observe(scan, 5));
  }
  EXPECT_TRUE(streak.observe(4, 5));
}

TEST(ConsecutiveFreeSpace, GapDoesNotAccumulateOldEvidence)
{
  ConsecutiveFreeSpace streak;
  EXPECT_FALSE(streak.observe(0, 5));
  EXPECT_FALSE(streak.observe(1, 5));
  // 2번부터 9번 스캔까지 광선이 없거나 가려졌거나 대응 오도메트리가 없다고 가정한다.
  for (std::size_t scan = 10; scan < 14; ++scan) {
    EXPECT_FALSE(streak.observe(scan, 5));
  }
  EXPECT_TRUE(streak.observe(14, 5));
}

TEST(ConsecutiveFreeSpace, OccupiedObservationResetsStreak)
{
  ConsecutiveFreeSpace streak;
  for (std::size_t scan = 0; scan < 4; ++scan) {
    EXPECT_FALSE(streak.observe(scan, 5));
  }
  streak.reset();
  for (std::size_t scan = 4; scan < 8; ++scan) {
    EXPECT_FALSE(streak.observe(scan, 5));
  }
  EXPECT_TRUE(streak.observe(8, 5));
}

TEST(ConsecutiveFreeSpace, MultipleRaysInOneScanCountOnlyOnce)
{
  ConsecutiveFreeSpace streak;
  for (int ray = 0; ray < 100; ++ray) {EXPECT_FALSE(streak.observe(0, 5));}
  for (std::size_t scan = 1; scan < 4; ++scan) {
    EXPECT_FALSE(streak.observe(scan, 5));
  }
  EXPECT_TRUE(streak.observe(4, 5));
}
