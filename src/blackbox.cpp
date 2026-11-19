// blackbox.cpp
//
// Flight-log recorder and replayer ("black box") for the mini-gcs byte stream.
//
// It records the RAW bytes that travel between the radio link and the parser,
// each chunk with a timestamp. Replaying the log feeds the exact same bytes
// (including noise, garbage and corrupted packets) back with the original
// timing, so a flight can be inspected again, or a parser bug reproduced.
//
//   record:  ./drone_sim | ./blackbox record flight.ubx | ./gcs_bridge ...
//            (stdin is logged and passed through to stdout; if the downstream
//             process dies, logging continues)
//   replay:  ./blackbox replay flight.ubx [--speed 2.0] [--loop] | ./gcs_bridge ...
//   info:    ./blackbox info flight.ubx
//
// Build: g++ -std=c++17 -Wall -Wextra -O2 blackbox.cpp -o blackbox
//
// File format ("UAVLOG01"), all integers little-endian:
//   8 bytes  magic "UAVLOG01"
//   repeated record:  u32 t_ms (since start of recording) | u16 length | length bytes
// Every record is flushed to disk as soon as it is written, so a crash or a
// power cut loses at most the record in progress. A truncated last record is
// ignored on replay.

#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;
const char kMagic[8] = {'U', 'A', 'V', 'L', 'O', 'G', '0', '1'};

int do_record(const char* path) {
  FILE* f = std::fopen(path, "wb");
  if (!f) {
    std::fprintf(stderr, "cannot create %s: %s\n", path, std::strerror(errno));
    return 1;
  }
  std::fwrite(kMagic, 1, sizeof kMagic, f);
  std::fflush(f);

  const auto start = Clock::now();
  bool out_ok = true;
  uint8_t buf[4096];
  uint64_t records = 0, bytes = 0;
  for (;;) {
    ssize_t n = read(0, buf, sizeof buf);
    if (n < 0) {
      if (errno == EINTR) continue;
      break;
    }
    if (n == 0) break;
    const auto t = static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count());
    uint8_t hdr[6] = {static_cast<uint8_t>(t), static_cast<uint8_t>(t >> 8), static_cast<uint8_t>(t >> 16),
                      static_cast<uint8_t>(t >> 24), static_cast<uint8_t>(n), static_cast<uint8_t>(n >> 8)};
    std::fwrite(hdr, 1, sizeof hdr, f);
    std::fwrite(buf, 1, static_cast<size_t>(n), f);
    std::fflush(f);
    records++;
    bytes += static_cast<uint64_t>(n);
    if (out_ok) {
      // Passthrough. If the consumer goes away, keep recording anyway.
      if (std::fwrite(buf, 1, static_cast<size_t>(n), stdout) != static_cast<size_t>(n) || std::fflush(stdout) != 0)
        out_ok = false;
    }
  }
  std::fclose(f);
  std::fprintf(stderr, "blackbox: recorded %llu records, %llu bytes\n", static_cast<unsigned long long>(records),
               static_cast<unsigned long long>(bytes));
  return 0;
}

struct Record {
  uint32_t t_ms;
  std::vector<uint8_t> data;
};

// Reads the next complete record. Returns false at EOF; sets truncated if a partial record was found.
bool read_record(FILE* f, Record& r, bool& truncated) {
  uint8_t hdr[6];
  size_t got = std::fread(hdr, 1, sizeof hdr, f);
  if (got == 0) return false;
  if (got < sizeof hdr) {
    truncated = true;
    return false;
  }
  r.t_ms = static_cast<uint32_t>(hdr[0]) | (static_cast<uint32_t>(hdr[1]) << 8) | (static_cast<uint32_t>(hdr[2]) << 16) |
           (static_cast<uint32_t>(hdr[3]) << 24);
  const size_t len = static_cast<size_t>(hdr[4]) | (static_cast<size_t>(hdr[5]) << 8);
  r.data.resize(len);
  if (len && std::fread(r.data.data(), 1, len, f) != len) {
    truncated = true;
    return false;
  }
  return true;
}

FILE* open_log(const char* path) {
  FILE* f = std::fopen(path, "rb");
  if (!f) {
    std::fprintf(stderr, "cannot open %s: %s\n", path, std::strerror(errno));
    return nullptr;
  }
  char magic[8];
  if (std::fread(magic, 1, sizeof magic, f) != sizeof magic || std::memcmp(magic, kMagic, sizeof magic) != 0) {
    std::fprintf(stderr, "%s is not a UAVLOG01 file\n", path);
    std::fclose(f);
    return nullptr;
  }
  return f;
}

int do_replay(const char* path, double speed, bool loop) {
  FILE* f = open_log(path);
  if (!f) return 1;
  bool truncated = false;
  do {
    const auto start = Clock::now();
    Record r;
    bool any = false;
    while (read_record(f, r, truncated)) {
      any = true;
      std::this_thread::sleep_until(start + std::chrono::microseconds(static_cast<int64_t>(r.t_ms * 1000.0 / speed)));
      if (std::fwrite(r.data.data(), 1, r.data.size(), stdout) != r.data.size() || std::fflush(stdout) != 0) {
        std::fclose(f);
        return 0;  // consumer closed the pipe
      }
    }
    if (truncated) {
      std::fprintf(stderr, "blackbox: log ends with a partial record (ignored)\n");
      break;
    }
    if (loop && !any) {  // empty log: looping would spin at 100% CPU forever
      std::fprintf(stderr, "blackbox: log has no records, nothing to loop\n");
      break;
    }
    if (loop) {
      std::fseek(f, static_cast<long>(sizeof kMagic), SEEK_SET);
    }
  } while (loop);
  std::fclose(f);
  return 0;
}

int do_info(const char* path) {
  FILE* f = open_log(path);
  if (!f) return 1;
  Record r;
  bool truncated = false;
  uint64_t records = 0, bytes = 0;
  uint32_t last_t = 0;
  while (read_record(f, r, truncated)) {
    records++;
    bytes += r.data.size();
    last_t = r.t_ms;
  }
  std::fclose(f);
  std::printf("records:  %llu\nbytes:    %llu\nduration: %.3f s\ntruncated: %s\n", static_cast<unsigned long long>(records),
              static_cast<unsigned long long>(bytes), last_t / 1000.0, truncated ? "yes" : "no");
  return 0;
}

void usage() {
  std::fprintf(stderr,
               "usage:\n"
               "  blackbox record FILE            log stdin to FILE, pass it through to stdout\n"
               "  blackbox replay FILE [--speed X] [--loop]\n"
               "  blackbox info FILE\n");
}

}  // namespace

int main(int argc, char** argv) {
  std::signal(SIGPIPE, SIG_IGN);
  if (argc < 3) {
    usage();
    return 2;
  }
  const std::string cmd = argv[1];
  if (cmd == "record") return do_record(argv[2]);
  if (cmd == "info") return do_info(argv[2]);
  if (cmd == "replay") {
    double speed = 1.0;
    bool loop = false;
    for (int i = 3; i < argc; i++) {
      std::string a = argv[i];
      if (a == "--speed" && i + 1 < argc) speed = std::atof(argv[++i]);
      else if (a == "--loop") loop = true;
      else {
        usage();
        return 2;
      }
    }
    if (speed <= 0) {
      std::fprintf(stderr, "--speed must be > 0\n");
      return 2;
    }
    return do_replay(argv[2], speed, loop);
  }
  usage();
  return 2;
}
