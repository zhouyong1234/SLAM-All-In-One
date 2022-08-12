#include <cstdio>
#include <unistd.h>
#include <thread>

#include <replaykit/ReplayKit.hpp>
#include <replaykit/ReplayReader.hpp>

#include <replaykit.pb.h>

#include <nanomsg/pubsub.h>
#include <nanomsg/nn.h>

using namespace ::zz::replaykit;
using namespace ::replaykit;

static constexpr int MAX_ADDR_SUPPORTED = 10;

typedef ReplayKit<MultipleSameTopics<Buffer, MAX_ADDR_SUPPORTED>, Commands<>> NanomsgReplayKit;

template <typename ReplayKitType>
class NanomsgRecorder {
 public:
  NanomsgRecorder(const std::vector<std::string> &addrs, const std::string &filename) {
    replayKit_.SetRecorder(std::make_shared<FileRecorder>(filename));
    replayKit_.EnableRecord();
    for (size_t i = 0; i < addrs.size(); i++) {
      threads_.emplace_back(std::thread(SubNanomsgToReplayKit, i, addrs.at(i), std::ref(replayKit_)));
    }
    replayKit_.Start();
    for (size_t i = 0; i < addrs.size(); i++) {
      threads_.at(i).join();
    }
  }

 private:
  std::vector<std::thread> threads_;
  ReplayKitType replayKit_;

  static void SubNanomsgToReplayKit(int i, const std::string &addr, ReplayKitType &replayKit) {
    int sock = nn_socket(AF_SP, NN_SUB);
    nn_connect(sock, addr.c_str());
    nn_setsockopt(sock, NN_SUB, NN_SUB_SUBSCRIBE, "", 0);
    Buffer pb_buffer;
    bool running = true;
    while (running) {
      char *pMsg = nullptr;
      int recv_count = nn_recv(sock, &pMsg, NN_MSG, 0);
      if (recv_count < 0) {
        std::cerr << "socket.recv(): got recv_count = " << recv_count << std::endl;
      } else {
        pb_buffer.clear_buffer();
        pb_buffer.set_buffer(pMsg, recv_count);
        PublishToTopic(i, replayKit.Now(), pb_buffer, replayKit);
      }
      nn_freemsg(pMsg);
    }
  }

  template<int i = 0, typename std::enable_if<i < ReplayKitType::TopicsType::num_topics_, int>::type = 0>
  static void PublishToTopic(int j, double timestamp, const Buffer &pb_buffer, ReplayKitType &replayKit) {
    if (i == j) {
      replayKit.template Publish<i>(timestamp, pb_buffer);
      return;
    }
    PublishToTopic<i + 1>(j, timestamp, pb_buffer, replayKit);
  }

  template<int i, typename std::enable_if<i >= ReplayKitType::TopicsType::num_topics_, int>::type = 0>
  static void PublishToTopic(int j, double timestamp, const Buffer &pb_buffer, ReplayKitType &replayKit) {}
};

template <typename ReplayKitType>
class NanomsgReplayer {
 public:
  NanomsgReplayer(const std::vector<std::string> &addrs, const std::string &filename) : reader_(filename, replayKit_) {
    for (size_t i = 0; i < addrs.size(); i++) {
      int sock = nn_socket(AF_SP, NN_PUB);
      nn_sockets_.push_back(sock);
      nn_bind(sock, addrs.at(i).c_str());
      SubscribeTopic(i, replayKit_);
    }
    printf("Replaying...\n");
    std::thread th([&]() {
      replayKit_.Start();
    });
    reader_.Start();
    sleep(1);
    printf("Replay finished.\n");
    th.join();
  }

  ReplayKitType replayKit_;
  FileReplayReader<ReplayKitType> reader_;
  std::vector<int> nn_sockets_;

  template<int i = 0, typename std::enable_if<i < ReplayKitType::TopicsType::num_topics_, int>::type = 0>
  void SubscribeTopic(int j, ReplayKitType &replayKit) {
    if (i == j) {
      replayKit.template Subscribe<i>([&](const double timestamp, const Buffer &pb_buffer) {
        nn_send(nn_sockets_.at(i), pb_buffer.buffer().c_str(), pb_buffer.buffer().size(), 0);
      });
      return;
    }
    SubscribeTopic<i + 1>(j, replayKit);
  }

  template<int i, typename std::enable_if<i >= ReplayKitType::TopicsType::num_topics_, int>::type = 0>
  void SubscribeTopic(int j, ReplayKitType &replayKit) {}
};

void show_help() {
  std::cout << "Usage: nanobag <record|replay> LOGFILE Address1 [Address2 [... [Address10]]]\n"
               "Record nanomsg from net addresses or replay it from the recorded file."
               "Currently support up to "<< MAX_ADDR_SUPPORTED <<" addresses.\n\n"
               "Example: \n"
               "1. To record bottom images and fc info from Falcon, and save it to file 'dataset.log':\n"
               "     nanobag record dataset.log tcp://127.0.0.1:18950 tcp://127.0.0.1:19051\n"
               "   Note: If the target file already exists, a new file with extra extension '.1' will be \n"
               "         created. In this case will be: 'dataset.log.1'\n\n"
               "2. To playback the dataset we've just recorded, simply substitute 'record' with 'replay':\n"
               "     nanobag replay dataset.log tcp://127.0.0.1:18950 tcp://127.0.0.1:19051\n"
               "   Warning: Although you can input any numbers of addresses to the arguments, we recommend "
               "            you to use the same address order for playing back. Otherwise you will get"
               "            wrong messages from the corresponding address.\n";

}

int main(int argc, char *argv[]) {
  if (argc < 4) {
    show_help();
    return -1;
  }
  std::vector<std::string> addresses;
  for (int i = 3; i < argc; i++) {
    addresses.emplace_back(argv[i]);
  }
  std::string filename(argv[2]);
  if (strcmp(argv[1], "record") == 0) {
    NanomsgRecorder<NanomsgReplayKit> nanomsgRecorder(addresses, filename);
  } else if (strcmp(argv[1], "replay") == 0) {
    NanomsgReplayer<NanomsgReplayKit> nanomsgReplayer(addresses, filename);
  } else {
    show_help();
  }
  return 0;
}
