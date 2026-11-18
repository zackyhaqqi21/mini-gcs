// gcs_bridge.cpp
// Ground-station side: reads the raw (possibly damaged) byte stream from stdin,
// runs it through uavproto::Parser, and prints one JSON object per line for
// every VALID packet, plus link statistics a few times per second.
//
//   ./drone_sim | ./gcs_bridge
//
// Needs POSIX read(); works on Linux, macOS and WSL.

#include <unistd.h>

#include <chrono>
#include <cstdio>

#include "protocol.hpp"

using namespace uavproto;

namespace {

void print_stats(const Stats& s) {
  std::printf(
      "{\"type\":\"stats\",\"ok\":%u,\"crc_errors\":%u,\"discarded\":%u,"
      "\"lost\":%u,\"reordered\":%u}\n",
      s.packets_ok, s.crc_errors, s.bytes_discarded, s.packets_lost,
      s.packets_reordered);
  std::fflush(stdout);
}

void handle_packet(const Packet& p) {
  const std::vector<uint8_t>& d = p.payload;
  switch (p.msg_id) {
    case MSG_ATTITUDE:
      if (d.size() == 6) {
        std::printf(
            "{\"type\":\"attitude\",\"seq\":%u,\"roll\":%.2f,\"pitch\":%.2f,"
            "\"yaw\":%.2f}\n",
            p.seq, get_i16(&d[0]) / 100.0, get_i16(&d[2]) / 100.0,
            get_i16(&d[4]) / 100.0);
      }
      break;
    case MSG_ALT_SPEED:
      if (d.size() == 4) {
        std::printf(
            "{\"type\":\"altspeed\",\"seq\":%u,\"alt_m\":%.2f,\"speed_ms\":%.2f}\n",
            p.seq, get_i16(&d[0]) / 100.0, get_i16(&d[2]) / 100.0);
      }
      break;
    case MSG_BATTERY:
      if (d.size() == 4) {
        std::printf(
            "{\"type\":\"battery\",\"seq\":%u,\"volt\":%.3f,\"pct\":%d}\n", p.seq,
            get_i16(&d[0]) / 1000.0, get_i16(&d[2]));
      }
      break;
    case MSG_HEARTBEAT:
      std::printf("{\"type\":\"heartbeat\",\"seq\":%u}\n", p.seq);
      break;
    default:
      return;  // unknown message id: ignore
  }
  std::fflush(stdout);
}

}  // namespace

int main() {
  Parser parser(handle_packet);

  uint8_t buf[512];
  auto last_stats = std::chrono::steady_clock::now();

  for (;;) {
    const ssize_t n = read(STDIN_FILENO, buf, sizeof(buf));
    if (n <= 0) break;
    parser.feed(buf, static_cast<size_t>(n));

    const auto now = std::chrono::steady_clock::now();
    if (now - last_stats >= std::chrono::milliseconds(250)) {
      print_stats(parser.stats());
      last_stats = now;
    }
  }
  print_stats(parser.stats());
  return 0;
}
