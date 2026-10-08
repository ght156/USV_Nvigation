//
// set_lidar_ip.cpp
//
// Livox Mid-360s lidar IP flashing tool (Linux, no ROS dependency).
//
// Usage:
//   set_lidar_ip <target_ip> [--netmask IP] [--gateway IP] [--host-ip IP] [--timeout SEC]
//
// Example:
//   # host NIC configured as 192.168.1.10/24 (factory lidar IP is 192.168.1.1xx),
//   # one lidar connected, flash it to 192.168.2.191:
//   sudo ./set_lidar_ip 192.168.2.191 --host-ip 192.168.1.10
//
// Defaults: netmask 255.255.255.0, gateway <target_subnet>.1,
//           host-ip 192.168.1.10, timeout 30 s.
//
// NOTE:
// - Connect ONLY ONE lidar during flashing; the tool refuses to work when
//   more than one lidar is discovered.
// - The lidar reboots after its IP is changed, connection loss is expected.
// - Requires Livox SDK2 (liblivox_lidar_sdk_shared) installed, e.g. /usr/local.
//
// Build:
//   g++ -std=c++14 -O2 -o set_lidar_ip set_lidar_ip.cpp \
//       -I/usr/local/include -L/usr/local/lib \
//       -llivox_lidar_sdk_shared -Wl,-rpath,/usr/local/lib -lpthread
//

#include <arpa/inet.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

namespace {

struct Options {
  std::string target_ip;
  std::string netmask = "255.255.255.0";
  std::string gateway;              // default: <target subnet>.1
  std::string host_ip = "192.168.1.10";
  int timeout_sec = 30;
};

struct State {
  std::mutex mtx;
  std::condition_variable cv;
  int discovered = 0;               // number of lidars seen
  uint32_t handle = 0;              // first discovered lidar
  LivoxLidarInfo info{};            // first discovered lidar info
  bool cmd_done = false;
  livox_status cmd_status = kLivoxLidarStatusFailure;
  uint8_t cmd_ret_code = 0xff;
};

State g_state;

bool IsValidIpv4(const std::string& ip) {
  struct in_addr addr;
  return inet_pton(AF_INET, ip.c_str(), &addr) == 1;
}

// Derive "<subnet>.1" gateway from an IPv4 address (assumes /24).
std::string DefaultGateway(const std::string& ip) {
  std::string gw = ip;
  const size_t pos = gw.find_last_of('.');
  if (pos == std::string::npos) return "";
  gw.replace(pos + 1, std::string::npos, "1");
  return gw;
}

void LidarInfoChangeCb(const uint32_t handle, const LivoxLidarInfo* info, void*) {
  std::lock_guard<std::mutex> lock(g_state.mtx);
  ++g_state.discovered;
  if (g_state.discovered == 1 && info != nullptr) {
    g_state.handle = handle;
    g_state.info = *info;
    printf("[discover] lidar found: sn=%s dev_type=%u current_ip=%s\n",
           info->sn, static_cast<unsigned>(info->dev_type), info->lidar_ip);
  } else if (info != nullptr) {
    printf("[discover] another lidar found: sn=%s current_ip=%s\n",
           info->sn, info->lidar_ip);
  }
  g_state.cv.notify_all();
}

void RebootCb(livox_status status, uint32_t handle,
              LivoxLidarRebootResponse* response, void*) {
  printf("reboot ack: status=%d ret_code=%u\n", static_cast<int>(status),
         response ? static_cast<unsigned>(response->ret_code) : 0u);
}

void SetIpCb(livox_status status, uint32_t handle,
             LivoxLidarAsyncControlResponse* response, void*) {
  std::lock_guard<std::mutex> lock(g_state.mtx);
  g_state.cmd_done = true;
  g_state.cmd_status = status;
  g_state.cmd_ret_code = (response != nullptr) ? response->ret_code : 0xff;
  (void)handle;
  g_state.cv.notify_all();
}

// Generate a minimal SDK config json (Mid360s default ports) for discovery.
std::string WriteTempConfig(const std::string& host_ip) {
  std::string path = "/tmp/set_lidar_ip_cfg_" + std::to_string(getpid()) + ".json";
  FILE* fp = std::fopen(path.c_str(), "w");
  if (fp == nullptr) return "";
  std::fprintf(fp,
      "{\n"
      "  \"lidar_summary_info\" : { \"lidar_type\": 8 },\n"
      "  \"Mid360s\": {\n"
      "    \"lidar_net_info\" : {\n"
      "      \"cmd_data_port\"  : 56100,\n"
      "      \"push_msg_port\"  : 56200,\n"
      "      \"point_data_port\": 56300,\n"
      "      \"imu_data_port\"  : 56400,\n"
      "      \"log_data_port\"  : 56500\n"
      "    },\n"
      "    \"host_net_info\" : [\n"
      "      {\n"
      "        \"host_ip\"        : \"%s\",\n"
      "        \"cmd_data_port\"  : 56101,\n"
      "        \"push_msg_port\"  : 56201,\n"
      "        \"point_data_port\": 56301,\n"
      "        \"imu_data_port\"  : 56401,\n"
      "        \"log_data_port\"  : 56501\n"
      "      }\n"
      "    ]\n"
      "  },\n"
      "  \"lidar_configs\" : []\n"
      "}\n",
      host_ip.c_str());
  std::fclose(fp);
  return path;
}

void PrintUsage(const char* prog) {
  std::fprintf(stderr,
      "usage: %s <target_ip> [--netmask IP] [--gateway IP] [--host-ip IP] [--timeout SEC]\n"
      "example: sudo %s 192.168.2.191 --host-ip 192.168.1.10\n",
      prog, prog);
}

}  // namespace

int main(int argc, char** argv) {
  Options opt;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto need_value = [&](const char* name) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "missing value for %s\n", name);
        std::exit(2);
      }
      return argv[++i];
    };
    if (arg == "--netmask")      opt.netmask = need_value("--netmask");
    else if (arg == "--gateway") opt.gateway = need_value("--gateway");
    else if (arg == "--host-ip") opt.host_ip = need_value("--host-ip");
    else if (arg == "--timeout") opt.timeout_sec = std::atoi(need_value("--timeout"));
    else if (arg == "-h" || arg == "--help") { PrintUsage(argv[0]); return 0; }
    else if (opt.target_ip.empty() && arg[0] != '-') opt.target_ip = arg;
    else { PrintUsage(argv[0]); return 2; }
  }

  if (opt.target_ip.empty() || !IsValidIpv4(opt.target_ip)) {
    std::fprintf(stderr, "invalid target ip: '%s'\n", opt.target_ip.c_str());
    PrintUsage(argv[0]);
    return 2;
  }
  if (!IsValidIpv4(opt.netmask) || !IsValidIpv4(opt.host_ip)) {
    std::fprintf(stderr, "invalid netmask or host-ip\n");
    return 2;
  }
  if (opt.gateway.empty()) opt.gateway = DefaultGateway(opt.target_ip);
  if (!IsValidIpv4(opt.gateway)) {
    std::fprintf(stderr, "invalid gateway: '%s'\n", opt.gateway.c_str());
    return 2;
  }

  printf("target: ip=%s netmask=%s gateway=%s | host_ip=%s timeout=%ds\n",
         opt.target_ip.c_str(), opt.netmask.c_str(), opt.gateway.c_str(),
         opt.host_ip.c_str(), opt.timeout_sec);

  const std::string cfg_path = WriteTempConfig(opt.host_ip);
  if (cfg_path.empty()) {
    std::fprintf(stderr, "failed to write temp config\n");
    return 1;
  }

  if (!LivoxLidarSdkInit(cfg_path.c_str(), opt.host_ip.c_str())) {
    std::fprintf(stderr, "livox lidar sdk init failed (check host-ip / NIC config)\n");
    std::remove(cfg_path.c_str());
    return 1;
  }
  std::remove(cfg_path.c_str());
  SetLivoxLidarInfoChangeCallback(LidarInfoChangeCb, nullptr);

  // Wait for discovery.
  {
    std::unique_lock<std::mutex> lock(g_state.mtx);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(opt.timeout_sec);
    while (g_state.discovered == 0) {
      if (g_state.cv.wait_until(lock, deadline) == std::cv_status::timeout) {
        std::fprintf(stderr,
            "no lidar discovered within %ds. check: host NIC ip, cable, lidar power.\n",
            opt.timeout_sec);
        LivoxLidarSdkUninit();
        return 1;
      }
    }
    if (g_state.discovered > 1) {
      std::fprintf(stderr,
          "more than one lidar discovered (%d). connect ONLY ONE lidar and retry.\n",
          g_state.discovered);
      LivoxLidarSdkUninit();
      return 1;
    }
  }

  const uint32_t handle = g_state.handle;
  if (opt.target_ip == g_state.info.lidar_ip) {
    printf("lidar ip is already %s, nothing to do.\n", opt.target_ip.c_str());
    LivoxLidarSdkUninit();
    return 0;
  }

  LivoxLidarIpInfo ip_info;
  std::memset(&ip_info, 0, sizeof(ip_info));
  std::snprintf(ip_info.ip_addr, sizeof(ip_info.ip_addr), "%s", opt.target_ip.c_str());
  std::snprintf(ip_info.net_mask, sizeof(ip_info.net_mask), "%s", opt.netmask.c_str());
  std::snprintf(ip_info.gw_addr, sizeof(ip_info.gw_addr), "%s", opt.gateway.c_str());

  printf("sending set-ip command to lidar (sn=%s) ...\n", g_state.info.sn);
  if (SetLivoxLidarIp(handle, &ip_info, SetIpCb, nullptr) != kLivoxLidarStatusSuccess) {
    std::fprintf(stderr, "failed to send set-ip command\n");
    LivoxLidarSdkUninit();
    return 1;
  }

  int ret = 1;
  {
    std::unique_lock<std::mutex> lock(g_state.mtx);
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::seconds(opt.timeout_sec);
    while (!g_state.cmd_done) {
      if (g_state.cv.wait_until(lock, deadline) == std::cv_status::timeout) {
        std::fprintf(stderr, "set-ip command timeout (%ds)\n", opt.timeout_sec);
        LivoxLidarSdkUninit();
        return 1;
      }
    }
  }

  if (g_state.cmd_status == kLivoxLidarStatusSuccess && g_state.cmd_ret_code == 0) {
    printf("SUCCESS: lidar ip set to %s; requesting reboot ...\n",
           opt.target_ip.c_str());
    if (LivoxLidarRequestReboot(handle, RebootCb, nullptr) != kLivoxLidarStatusSuccess) {
      fprintf(stderr, "warning: failed to send reboot command\n");
    } else {
      sleep(2);  /* 等重启 ack：雷达不会自己重启，必须显式发这条命令 */
    }
    printf("lidar is rebooting; verify with 'ping %s' from a host in the same subnet.\n",
           opt.target_ip.c_str());
    ret = 0;
  } else {
    std::fprintf(stderr, "FAILED: status=%d ret_code=%u\n",
                 static_cast<int>(g_state.cmd_status),
                 static_cast<unsigned>(g_state.cmd_ret_code));
  }

  LivoxLidarSdkUninit();
  return ret;
}
