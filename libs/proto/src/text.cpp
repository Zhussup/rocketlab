#include "rocketlab/proto/snapshot.hpp"

#include <algorithm>
#include <cstring>

namespace rocketlab::proto {

void Text::assign(std::string_view text) noexcept {
  const std::size_t length = std::min(text.size(), kMaxMessageLength - 1);
  std::memcpy(data, text.data(), length);
  // The tail is cleared rather than just terminated, so that two snapshots
  // holding the same message compare equal.
  std::memset(data + length, 0, kMaxMessageLength - length);
}

std::string_view Text::view() const noexcept {
  return std::string_view(data, std::strlen(data));
}

}  // namespace rocketlab::proto
