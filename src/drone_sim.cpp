// drone_sim.cpp
// Fake drone: flies a smooth pattern and writes telemetry packets (made with
// uavproto::encode) to stdout as a raw byte stream, optionally through the
// NoisyLink simulator from the uav-telemetry-protocol project.
//
//   ./drone_sim [--noise 0.05] [--rate 50] [--duration 0] [--seed 1]
//
//   --noise     0..1, fraction of packets hit by each fault type (default 0)
//   --rate      attitude packets per second (default 50)
//   --duration  seconds to run, 0 = forever (default 0)
//   --seed      random seed for the noisy link (default 1)

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>
#include <vector>

#include "noisy_link.hpp"
#include "protocol.hpp"

using namespace uavproto;
using namespace uavsim;

namespace {

constexpr double kPi = 3.14159265358979323846;

int16_t to_i16(double v) {
  if (v > 32767.0) v = 32767.0;
  if (v < -32768.0) v = -32768.0;
  return static_cast<int16_t>(std::lround(v));
}

struct Args {
  double noise = 0.0;
  double rate = 50.0;
  double duration = 0.0;
  uint32_t seed = 1;
};

bool parse_args(int argc, char** argv, Args* a) {
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    if (i + 1 >= argc) return false;
    const char* v = argv[++i];
    if (k == "--noise") a->noise = std::atof(v);
    else if (k == "--rate") a->rate = std::atof(v);
    else if (k == "--duration") a->duration = std::atof(v);
    else if (k == "--seed") a->seed = static_cast<uint32_t>(std::atol(v));
    else return false;
  }
  return a->rate > 0.0 && a->noise >= 0.0 && a->noise <= 1.0;
}

}  // namespace

int main(int argc, char** argv) {
  Args a;
  if (!parse_args(argc, argv, &a)) {
    std::fprintf(stderr,
                 "usage: %s [--noise 0..1] [--rate Hz] [--duration s] [--seed n]\n",
                 argv[0]);
    return 1;
  }

  NoisyLink link(uniform_noise(a.noise), a.seed);
  const double dt = 1.0 / a.rate;
  const auto t0 = std::chrono::steady_clock::now();

  uint8_t seq = 0;      // one counter for ALL packets, wraps at 255
  double yaw_deg = 0.0;  // heading, kept in (-180, 180]

  for (uint64_t n = 0;; ++n) {
    const double t = static_cast<double>(n) * dt;
    if (a.duration > 0.0 && t >= a.duration) break;

    // Smooth made-up flight: banking turns, gentle pitch, climbing/descending.
    const double roll = 28.0 * std::sin(2.0 * kPi * t / 9.0);
    const double pitch = 8.0 * std::sin(2.0 * kPi * t / 6.0 + 0.8) + 3.0;
    yaw_deg += 0.9 * roll * dt;  // roughly a coordinated turn
    if (yaw_deg > 180.0) yaw_deg -= 360.0;
    if (yaw_deg <= -180.0) yaw_deg += 360.0;
    const double alt_m = 80.0 + 25.0 * std::sin(2.0 * kPi * t / 30.0);
    const double speed_ms = 12.0 + 3.0 * std::sin(2.0 * kPi * t / 11.0);
    const double pct = std::fmax(0.0, 100.0 - 0.2 * t);
    const double volt_mv = 10500.0 + 21.0 * pct;

    std::vector<std::vector<uint8_t>> batch;
    auto add = [&](uint8_t id, const std::vector<uint8_t>& payload) {
      batch.push_back(encode(id, seq++, payload));
    };

    std::vector<uint8_t> p;
    put_i16(p, to_i16(roll * 100.0));  // centi-degrees
    put_i16(p, to_i16(pitch * 100.0));
    put_i16(p, to_i16(yaw_deg * 100.0));
    add(MSG_ATTITUDE, p);

    if (n % 2 == 0) {
      p.clear();
      put_i16(p, to_i16(alt_m * 100.0));     // cm
      put_i16(p, to_i16(speed_ms * 100.0));  // cm/s
      add(MSG_ALT_SPEED, p);
    }
    if (n % 10 == 0) {
      p.clear();
      put_i16(p, to_i16(volt_mv));  // mV
      put_i16(p, to_i16(pct));      // percent
      add(MSG_BATTERY, p);
    }
    if (n % 50 == 0) {
      add(MSG_HEARTBEAT, {});
    }

    const std::vector<uint8_t> bytes = link.transmit(std::move(batch));
    if (!bytes.empty() &&
        std::fwrite(bytes.data(), 1, bytes.size(), stdout) != bytes.size()) {
      return 0;  // reader went away
    }
    std::fflush(stdout);

    std::this_thread::sleep_until(
        t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                 std::chrono::duration<double>(static_cast<double>(n + 1) * dt)));
  }
  return 0;
}
