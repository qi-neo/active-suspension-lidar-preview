#include "unilidar_driver/protocol.hpp"

namespace unilidar_driver {

uint32_t crc32(const uint8_t* buf, std::size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= buf[i];
    for (int bit = 0; bit < 8; ++bit) {
      const uint32_t mask = static_cast<uint32_t>(-(static_cast<int32_t>(crc) & 1));
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

}  // namespace unilidar_driver
