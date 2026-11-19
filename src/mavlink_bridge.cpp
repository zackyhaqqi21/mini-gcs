// mavlink_bridge.cpp
//
// MAVLink (v1 / v2) -> uav-telemetry-protocol converter.
//
// Reads MAVLink bytes from a UDP port (ArduPilot SITL / MAVProxy), a serial
// port (real flight controller + telemetry radio) or a file/stdin, and writes
// uavproto packets to stdout. It is a drop-in replacement for drone_sim:
//
//   ./mavlink_bridge --udp 14550 | ./gcs_bridge | python3 server/server.py ...
//
// Build (from the mini-gcs root, with lib/protocol.* from uav-telemetry-protocol):
//   g++ -std=c++17 -Wall -Wextra -O2 -Ilib src/mavlink_bridge.cpp lib/protocol.cpp -o build/mavlink_bridge
//
// MAVLink messages used:
//   HEARTBEAT (0)             -> MSG_HEARTBEAT   (GCS heartbeats are ignored)
//   ATTITUDE (30)             -> MSG_ATTITUDE
//   GLOBAL_POSITION_INT (33)  -> MSG_ALT_SPEED   (relative altitude, ground speed)
//   SYS_STATUS (1)            -> MSG_BATTERY
//
// Frames with a bad CRC are dropped, never converted. The CRC is checked with
// the per-message CRC_EXTRA, so a frame from a different dialect/version is
// rejected as well. Frames with an unknown message id cannot be verified; they
// are skipped as a whole (counted as "unknown").

#include "protocol.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace uavproto;

namespace {

constexpr double kPi = 3.14159265358979323846;

// ---------------------------------------------------------------------------
// Payload layout of the uavproto messages (see protocol.hpp): every field is an
// int16, little-endian, so each payload is 2 bytes per field.
//   ATTITUDE   roll, pitch, yaw      centi-degrees          (6 bytes)
//   ALT_SPEED  altitude, speed       cm, cm/s               (4 bytes)
//   BATTERY    voltage, percent      mV, %                  (4 bytes)
// Values that do not fit are clamped, so altitude saturates at +-327.67 m and
// battery voltage at 32.767 V (same limits as tools/mavlink_source.py).
// ---------------------------------------------------------------------------
int16_t to_i16(double v) {
  if (!std::isfinite(v)) return 0;
  if (v > 32767.0) v = 32767.0;
  if (v < -32768.0) v = -32768.0;
  return static_cast<int16_t>(std::lround(v));
}
std::vector<uint8_t> pack_attitude(int16_t roll, int16_t pitch, int16_t yaw) {
  std::vector<uint8_t> v;
  put_i16(v, roll);
  put_i16(v, pitch);
  put_i16(v, yaw);
  return v;
}
std::vector<uint8_t> pack_alt_speed(double alt_cm, double speed_cms) {
  std::vector<uint8_t> v;
  put_i16(v, to_i16(alt_cm));
  put_i16(v, to_i16(speed_cms));
  return v;
}
std::vector<uint8_t> pack_battery(double mv, int percent) {
  std::vector<uint8_t> v;
  put_i16(v, to_i16(mv));
  put_i16(v, to_i16(percent));
  return v;
}

// ---------------------------------------------------------------------------
// Little-endian readers (MAVLink wire format).
// ---------------------------------------------------------------------------
uint16_t rd_u16(const uint8_t* p) { return static_cast<uint16_t>(p[0] | (p[1] << 8)); }
int16_t rd_i16(const uint8_t* p) { return static_cast<int16_t>(rd_u16(p)); }
uint32_t rd_u32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}
int32_t rd_i32(const uint8_t* p) { return static_cast<int32_t>(rd_u32(p)); }
float rd_f32(const uint8_t* p) {
  uint32_t x = rd_u32(p);
  float f;
  std::memcpy(&f, &x, sizeof f);
  return f;
}

// ---------------------------------------------------------------------------
// MAVLink framing.
// ---------------------------------------------------------------------------
enum : uint32_t {
  MAV_HEARTBEAT = 0,
  MAV_SYS_STATUS = 1,
  MAV_ATTITUDE = 30,
  MAV_GLOBAL_POSITION_INT = 33,
};

struct MsgInfo {
  uint32_t id;
  uint8_t crc_extra;
  uint8_t base_len;  // payload length without extension fields
};
const MsgInfo kMsgs[] = {
    {MAV_HEARTBEAT, 50, 9},
    {MAV_SYS_STATUS, 124, 31},
    {MAV_ATTITUDE, 39, 28},
    {MAV_GLOBAL_POSITION_INT, 104, 28},
};
const MsgInfo* find_msg(uint32_t id) {
  for (const auto& m : kMsgs)
    if (m.id == id) return &m;
  return nullptr;
}

// MCRF4XX / X.25 CRC as used by MAVLink.
void crc_acc(uint16_t& crc, uint8_t b) {
  uint8_t tmp = static_cast<uint8_t>(b ^ (crc & 0xFF));
  tmp = static_cast<uint8_t>(tmp ^ (tmp << 4));
  crc = static_cast<uint16_t>((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));
}

struct MavStats {
  uint32_t frames_ok = 0;
  uint32_t crc_bad = 0;
  uint32_t unknown = 0;
  uint64_t bytes_skipped = 0;
};

class MavParser {
 public:
  // payload is zero-padded to the base length (MAVLink v2 trims trailing zeros).
  using Handler = void (*)(void* ctx, uint32_t msgid, const std::vector<uint8_t>& payload);

  MavParser(Handler h, void* ctx) : handler_(h), ctx_(ctx) {}

  void feed(const uint8_t* data, size_t n) {
    buf_.insert(buf_.end(), data, data + n);
    size_t i = 0;
    for (;;) {
      while (i < buf_.size() && buf_[i] != 0xFD && buf_[i] != 0xFE) {
        i++;
        stats.bytes_skipped++;
      }
      if (i >= buf_.size()) break;
      const size_t avail = buf_.size() - i;
      const bool v2 = buf_[i] == 0xFD;
      const size_t hdr = v2 ? 10 : 6;
      if (avail < hdr) break;
      const size_t len = buf_[i + 1];
      // MAVLink v2: only incompat flag bit 0 (signing) is defined. Any other bit
      // means this 0xFD is not a real frame start: drop it now instead of
      // waiting for up to ~280 bytes.
      if (v2 && (buf_[i + 2] & 0xFE)) {
        i += 1;
        stats.bytes_skipped++;
        continue;
      }
      const uint32_t msgid =
          v2 ? (static_cast<uint32_t>(buf_[i + 7]) | (static_cast<uint32_t>(buf_[i + 8]) << 8) |
                (static_cast<uint32_t>(buf_[i + 9]) << 16))
             : buf_[i + 5];
      const MsgInfo* info = find_msg(msgid);
      // A known message cannot be much longer than its base length (extensions only add a few bytes).
      if (info && len > info->base_len + 32u) {
        i += 1;
        stats.bytes_skipped++;
        continue;
      }
      const size_t sig = (v2 && (buf_[i + 2] & 0x01)) ? 13 : 0;  // signed frame
      const size_t total = hdr + len + 2 + sig;
      if (avail < total) break;
      if (!info) {
        // Unknown message: the CRC cannot be checked, so accept the frame only if
        // another frame start follows directly. Otherwise it was a false candidate
        // and must not swallow the real frames behind it.
        if (avail == total) break;  // wait for the next byte
        if (buf_[i + total] != 0xFD && buf_[i + total] != 0xFE) {
          i += 1;
          stats.bytes_skipped++;
          continue;
        }
        stats.unknown++;
        i += total;
        continue;
      }
      uint16_t crc = 0xFFFF;
      for (size_t k = 1; k < hdr + len; k++) crc_acc(crc, buf_[i + k]);
      crc_acc(crc, info->crc_extra);
      const uint16_t rx = rd_u16(&buf_[i + hdr + len]);
      if (crc != rx) {
        // Same idea as uavproto::Parser: drop only the false magic byte and
        // rescan, so a real frame inside the false candidate is still found.
        stats.crc_bad++;
        i += 1;
        stats.bytes_skipped++;
        continue;
      }
      std::vector<uint8_t> payload(buf_.begin() + i + hdr, buf_.begin() + i + hdr + len);
      if (payload.size() < info->base_len) payload.resize(info->base_len, 0);
      stats.frames_ok++;
      handler_(ctx_, msgid, payload);
      i += total;
    }
    buf_.erase(buf_.begin(), buf_.begin() + i);
  }

  MavStats stats;

 private:
  Handler handler_;
  void* ctx_;
  std::vector<uint8_t> buf_;
};

// ---------------------------------------------------------------------------
// Conversion to uavproto.
// ---------------------------------------------------------------------------
int16_t to_centideg(float rad, bool wrap) {
  double d = static_cast<double>(rad) * 18000.0 / kPi;
  if (!std::isfinite(d)) d = 0;
  if (wrap) {
    d = std::fmod(d, 36000.0);
    if (d > 18000.0) d -= 36000.0;
    if (d <= -18000.0) d += 36000.0;
  }
  if (d > 32767.0) d = 32767.0;
  if (d < -32768.0) d = -32768.0;
  return static_cast<int16_t>(std::lround(d));
}

struct Converter {
  uint8_t seq = 0;
  uint32_t sent = 0;

  void send(uint8_t msg_id, const std::vector<uint8_t>& payload) {
    std::vector<uint8_t> bytes = encode(msg_id, seq++, payload);
    if (std::fwrite(bytes.data(), 1, bytes.size(), stdout) != bytes.size()) std::exit(0);  // downstream closed
    std::fflush(stdout);
    sent++;
  }

  void on_mavlink(uint32_t id, const std::vector<uint8_t>& p) {
    switch (id) {
      case MAV_HEARTBEAT:
        if (p[4] == 6) return;  // MAV_TYPE_GCS: not the vehicle
        send(MSG_HEARTBEAT, {});
        break;
      case MAV_ATTITUDE:
        // yaw is sent in (-180, 180] deg; the dashboard should take it modulo 360 for a heading.
        send(MSG_ATTITUDE, pack_attitude(to_centideg(rd_f32(&p[4]), false), to_centideg(rd_f32(&p[8]), false),
                                         to_centideg(rd_f32(&p[12]), true)));
        break;
      case MAV_GLOBAL_POSITION_INT: {
        const int32_t rel_alt_mm = rd_i32(&p[16]);
        const double vx = rd_i16(&p[20]), vy = rd_i16(&p[22]);  // cm/s
        send(MSG_ALT_SPEED, pack_alt_speed(rel_alt_mm / 10.0, std::sqrt(vx * vx + vy * vy)));
        break;
      }
      case MAV_SYS_STATUS: {
        const uint16_t mv = rd_u16(&p[14]);
        if (mv == 0xFFFF) break;  // voltage unknown
        const int pct = static_cast<int8_t>(p[30]);  // -1 = unknown
        send(MSG_BATTERY, pack_battery(mv, pct < 0 ? 0 : (pct > 100 ? 100 : pct)));
        break;
      }
      default:
        break;
    }
  }
};

void handler_trampoline(void* ctx, uint32_t id, const std::vector<uint8_t>& p) {
  static_cast<Converter*>(ctx)->on_mavlink(id, p);
}

// ---------------------------------------------------------------------------
// Input sources.
// ---------------------------------------------------------------------------
int open_udp(const char* bind_addr, int port) {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return -1;
  int one = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port = htons(static_cast<uint16_t>(port));
  if (inet_pton(AF_INET, bind_addr, &a.sin_addr) != 1 || bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof a) < 0) {
    close(fd);
    return -1;
  }
  return fd;
}

bool baud_to_speed(int baud, speed_t& out) {
  switch (baud) {
    case 9600: out = B9600; return true;
    case 19200: out = B19200; return true;
    case 38400: out = B38400; return true;
    case 57600: out = B57600; return true;
    case 115200: out = B115200; return true;
    case 230400: out = B230400; return true;
#ifdef B921600
    case 921600: out = B921600; return true;
#endif
    default: return false;
  }
}

int open_serial(const char* dev, int baud) {
  speed_t sp;
  if (!baud_to_speed(baud, sp)) {
    errno = EINVAL;
    return -1;
  }
  int fd = open(dev, O_RDWR | O_NOCTTY);
  if (fd < 0) return -1;
  termios t{};
  if (tcgetattr(fd, &t) != 0) {
    close(fd);
    return -1;
  }
  cfmakeraw(&t);
  cfsetispeed(&t, sp);
  cfsetospeed(&t, sp);
  t.c_cflag |= (CLOCAL | CREAD);
  t.c_cc[VMIN] = 1;
  t.c_cc[VTIME] = 0;
  if (tcsetattr(fd, TCSANOW, &t) != 0) {
    close(fd);
    return -1;
  }
  return fd;
}

void usage(const char* argv0) {
  std::fprintf(stderr,
               "usage: %s (--udp PORT [--bind ADDR] | --serial DEV [--baud N] | --file PATH|-)\n"
               "  --udp PORT     listen for MAVLink on a UDP port (ArduPilot SITL: 14550)\n"
               "  --bind ADDR    UDP bind address (default 127.0.0.1)\n"
               "  --serial DEV   read from a serial port (default 115200 baud)\n"
               "  --file PATH    read a raw MAVLink capture; '-' = stdin\n"
               "Writes uavproto packets to stdout, statistics to stderr.\n",
               argv0);
}

void print_stats(const MavParser& mp, const Converter& cv) {
  std::fprintf(stderr, "mavlink: ok=%u crc_bad=%u unknown=%u skipped=%llu sent=%u\n", mp.stats.frames_ok,
               mp.stats.crc_bad, mp.stats.unknown, static_cast<unsigned long long>(mp.stats.bytes_skipped), cv.sent);
}

}  // namespace

int main(int argc, char** argv) {
  const char* bind_addr = "127.0.0.1";
  const char* serial_dev = nullptr;
  const char* file = nullptr;
  int udp_port = -1, baud = 115200;

  for (int i = 1; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&](const char* what) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", what);
        std::exit(2);
      }
      return argv[++i];
    };
    if (a == "--udp") udp_port = std::atoi(next("--udp"));
    else if (a == "--bind") bind_addr = next("--bind");
    else if (a == "--serial") serial_dev = next("--serial");
    else if (a == "--baud") baud = std::atoi(next("--baud"));
    else if (a == "--file") file = next("--file");
    else {
      usage(argv[0]);
      return 2;
    }
  }

  int fd = -1;
  if (udp_port > 0) fd = open_udp(bind_addr, udp_port);
  else if (serial_dev) fd = open_serial(serial_dev, baud);
  else if (file) fd = (std::string(file) == "-") ? 0 : open(file, O_RDONLY);
  else {
    usage(argv[0]);
    return 2;
  }
  if (fd < 0) {
    std::fprintf(stderr, "cannot open input: %s\n", std::strerror(errno));
    return 1;
  }

  Converter cv;
  MavParser mp(handler_trampoline, &cv);
  uint8_t buf[2048];
  auto last = std::chrono::steady_clock::now();
  for (;;) {
    ssize_t n = read(fd, buf, sizeof buf);
    if (n < 0) {
      if (errno == EINTR) continue;
      std::fprintf(stderr, "read error: %s\n", std::strerror(errno));
      break;
    }
    if (n == 0) break;  // EOF (file / stdin)
    mp.feed(buf, static_cast<size_t>(n));
    auto now = std::chrono::steady_clock::now();
    if (now - last > std::chrono::seconds(5)) {
      print_stats(mp, cv);
      last = now;
    }
  }
  print_stats(mp, cv);
  return 0;
}
