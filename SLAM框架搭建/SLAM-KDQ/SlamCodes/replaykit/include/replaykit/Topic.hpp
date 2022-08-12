#pragma once

namespace zz {
namespace replaykit {

template <typename TopicBaseType>
class Topic {
 public:
  using type_ = TopicBaseType;
  typedef std::function<void(const double, const TopicBaseType&)>
      subscriber_type_;
  subscriber_type_ subscriber_;
  int topic_id_;
};

template <typename... TopicTypes>
class Topics {
 public:
  using topic_types_ = ::std::tuple<TopicTypes...>;

  using topics_type_ = ::std::tuple<Topic<TopicTypes>...>;

  static constexpr int num_topics_ =
      ::std::tuple_size<typename Topics::topics_type_>::value;
};

template <typename Element>
class TH_convert{
 public:
  typedef std::tuple<Element> t;
};

template <typename Arg, unsigned int N>
class TH_multiple_elements{
 private:
 public:
  typedef decltype(std::tuple_cat(typename TH_convert<Arg>::t(),typename TH_multiple_elements<Arg,N-1>::t())) t;
};
template <typename Arg>
class TH_multiple_elements<Arg,1>{
 public:
  typedef typename TH_convert<Arg>::t t;
};
template <typename Arg>
class TH_multiple_elements<Arg,0>{
 public:
  typedef std::tuple<> t;
};

template <typename TopicType, int N>
class MultipleSameTopics {
 public:
  using topic_types_ = typename TH_multiple_elements<TopicType, N>::t;

  using topics_type_ = typename TH_multiple_elements<Topic<TopicType>, N>::t;

  static constexpr int num_topics_ = N;
};

template <typename RequestType, typename ResponseType>
class Command {
 public:
  using request_type_ = RequestType;
  using response_type_ = ResponseType;
  using commander_type_ =
      std::function<ResponseType(const double timestamp, const RequestType&)>;
};

template <typename... CommandTypes>
class Commands {
 public:
  using commander_types_ =
      ::std::tuple<typename CommandTypes::commander_type_...>;
  static constexpr int num_commanders_ =
      ::std::tuple_size<commander_types_>::value;
};

}  // namespace replaykit
}  // namespace zz
