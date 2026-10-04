#include "rocketlab/proto/snapshot.hpp"

#include <algorithm>
#include <cstring>

namespace rocketlab::proto {

void Name::assign(std::string_view text) noexcept {
  const std::size_t length = std::min(text.size(), kMaxNameLength - 1);
  std::memcpy(data, text.data(), length);
  // The tail is cleared rather than just terminated so that two snapshots
  // holding the same name compare equal, which matters when a transport
  // decides whether anything changed.
  std::memset(data + length, 0, kMaxNameLength - length);
}

std::string_view Name::view() const noexcept {
  return std::string_view(data, std::strlen(data));
}

}  // namespace rocketlab::proto
