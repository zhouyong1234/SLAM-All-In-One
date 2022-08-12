#include <stdio.h>
#include <unistd.h>
#include <thread>

#include <replaykit/ReplayKit.hpp>
#include <replaykit/ReplayReader.hpp>

#include <replaykit.pb.h>

using namespace ::zz::replaykit;
using namespace ::std;
using namespace ::replaykit;

int main(int argc, char *argv[]) {
  ReplayKit<Topics<float, float, float, ReplayItem, Buffer>,
            Commands<Command<float, bool>>>
      replaykit;

  FileReplayReader<decltype(replaykit)> reader("/tmp/replay.log", replaykit);
  auto recorder = make_shared<FileRecorder>("/tmp/replay.log.1");
  replaykit.SetRecorder(recorder);

  replaykit.EnableRecord();

  printf("Add subscriber 0\n");
  replaykit.Subscribe<0>([](const double timestamp, const float &distance) {
    printf("Distance 0 %f\n", distance);
  });

  printf("Add subscriber 1\n");
  replaykit.Subscribe<1>([](const double timestamp, const float &distance) {
    printf("Distance 1 %f\n", distance);
  });

  printf("Add subscriber 2\n");
  replaykit.Subscribe<2>([](const double timestamp, const float &distance) {
    printf("Distance 2 %f\n", distance);
  });

  printf("Add subscriber 3\n");
  replaykit.Subscribe<3>([](const double timestamp, const ReplayItem &item) {
    printf("Distance 4 %f\n", item.timestamp());
  });

  thread t1([&]() {
    sleep(1);
    reader.Start();
    printf("Publish topic 2.\n");
    replaykit.Publish<2>(1.34, 2.0);
    printf("Publish topic 1.\n");
    replaykit.Publish<1>(1, 1.0);
    printf("Publish topic 0.\n");
    replaykit.Publish<0>(1, 0.0);

    ReplayItem item;
    item.set_timestamp(15.0);
    item.set_seq(1);
    item.set_type(1);
    item.set_buffer(NULL, 0);
    replaykit.Publish<3>(123, item);

    // Test for unsubscriber topic.
    replaykit.PublishBuffer<4>(321, "Hello World!", 12);
  });

  printf("Start replaykit.\n");
  replaykit.Start();
  t1.join();
  t1.join();
  return 0;
}
