#pragma once

#include <optional>
#include <string>

namespace m_utils {

struct Metadata {
  std::string boat_type;
  std::string sn;
  std::string version;
  std::string description;
};

// 从环境变量 USV_HOME 解析 metadata 路径：USV_HOME/app/metadata.json。
// 解析失败或文件不存在时返回 std::nullopt，并通过 error 输出原因。
std::optional<std::string> resolve_metadata_path_from_usv_home(std::string* error = nullptr);

// 直接通过 USV_HOME/app/metadata.json 读取元数据。
// 读取失败返回 std::nullopt，并通过 error 输出原因。
//
// 示例：
//   std::string err;
//   auto metadata = m_utils::load_metadata_from_usv_home(&err);
//   if (!metadata) {
//     // 例如：RCLCPP_ERROR(logger, "load metadata failed: %s", err.c_str());
//     return;
//   }
//   // 可直接使用 metadata->boat_type / metadata->version / metadata->sn
std::optional<Metadata> load_metadata_from_usv_home(std::string* error = nullptr);

// 从 metadata.json 加载常用字段；缺失字段会返回空字符串。
bool load_metadata(const std::string& metadata_path, Metadata& out, std::string* error = nullptr);

// 读取 metadata.json 中指定 key 的字符串值；key 不存在返回 std::nullopt。
std::optional<std::string> load_metadata_string(
  const std::string& metadata_path, const std::string& key, std::string* error = nullptr);

}  // namespace m_utils
