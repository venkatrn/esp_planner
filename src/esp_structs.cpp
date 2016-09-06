#include <esp_planner/esp_structs.h>

#include <boost/functional/hash.hpp>

namespace {
  const bool kBidirectionalEdges = false;
}

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
  bool edges_equal = (first == other.first) && (second == other.second);
  if (edges_equal) {
    return true;
  }
  if (kBidirectionalEdges) {
    edges_equal = edges_equal || ((first == other.second) && (second == other.first));
  }
  return edges_equal;
}
bool Edge::operator!=(const Edge &other) const {
  return !(*this == other);
}
size_t Edge::GetHash() const {
  size_t hash_value = 0;
  if (kBidirectionalEdges) {
    // Commutative hash.
    hash_value = std::hash<int>()(first) ^ std::hash<int>()(second);
  } else {
    hash_value = std::hash<int>()(first);
    // boost::hash_combine is not associative, which is what we need for directed
    // edges.
    boost::hash_combine(hash_value, std::hash<int>()(second));
  }
  return hash_value;
}

std::ostream &operator<< (std::ostream &stream, const Edge &edge) {
  stream << "(" << edge.first << ", " << edge.second << ")";
  return stream;
}
} // namespace sbpl
