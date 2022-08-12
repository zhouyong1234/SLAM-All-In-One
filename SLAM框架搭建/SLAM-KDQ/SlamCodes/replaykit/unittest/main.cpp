#include <gtest/gtest.h>
#include <replaykit/ReplayKit.hpp>

using namespace ::zz::replaykit;

namespace {

class ReplayKitTest : public ::testing::Test {
 public:
};

TEST_F(ReplayKitTest, BuildTest) {
  ReplayKit<Topics<float, float, float>, Commands<Command<float, bool>>>
      replaykit;

  replaykit.Subscribe<0>([](const double timestamp, const float &distance) {
    printf("Distance 0 %f\n", distance);
  });

  replaykit.Subscribe<1>([](const double timestamp, const float &distance) {
    printf("Distance 1 %f\n", distance);
  });

  replaykit.Subscribe<2>([](const double timestamp, const float &distance) {
    printf("Distance 2 %f\n", distance);
  });

  replaykit.Publish<2>(1, 2.0);
  replaykit.Publish<1>(1, 1.0);
  replaykit.Publish<0>(1, 0.0);
  EXPECT_EQ(replaykit.num_topics_, 3);
  EXPECT_EQ(replaykit.num_commanders_, 1);
}

}  // namespace

int main(int argc, char *argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
