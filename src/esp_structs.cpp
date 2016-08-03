#include <esp_planner/esp_structs.h>

#include <boost/functional/hash.hpp>

namespace sbpl {
Edge::Edge(int first, int second,
           double probability, double evaluation_time) :
  first{first}, second{second},
  probability{probability}, evaluation_time{evaluation_time} {

  if (probability < 0 || probability > 1.0) {
    printf("ERROR: Illegal probability value %f for edge: (%d %d)\n", probability,
           first, second);
  }

  if (evaluation_time < 0) {
    printf("ERROR: Illegal evaluation time %f for edge: (%d %d)\n",
           evaluation_time, first, second);
  }
}
Edge::Edge(int first, int second, double probability) : Edge(first, second,
                                                               probability, 0.0) {
}
Edge::Edge(int first, int second) : Edge(first, second, 0.0, 0.0) {
}
bool Edge::operator==(const Edge &other) const {
  return (first == other.first) && (second == other.second);
}
bool Edge::operator!=(const Edge &other) const {
  return !(*this == other);
}
size_t Edge::GetHash() const {
  size_t hash_value = std::hash<int>()(first);
  // boost::hash_combine is not associative, which is what we need for directed
  // edges.
  boost::hash_combine(hash_value, std::hash<int>()(second));
  return hash_value;
}

std::ostream &operator<< (std::ostream &stream, const Edge &edge) {
  stream << "(" << edge.first << ", " << edge.second << ")";
  return stream;
}
} // namespace sbpl
