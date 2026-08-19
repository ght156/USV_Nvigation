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

// 解析 metadata 路径：<根目录>/app/metadata.json。
// - x86：根目录优先 USV_HOME，未设置则用进程当前工作目录；文件不存在时返回 nullopt（不报错）。
// - 非 x86（如 ARM）：根目录为 USV_HOME；未设置 USV_HOME 或文件不存在时返回 nullopt 并写 error。
std::optional<std::string> resolve_metadata_path_from_usv_home(std::string* error = nullptr);

// 读取 <根目录>/app/metadata.json 元数据（根目录规则同 resolve_metadata_path_from_usv_home）。
// - x86：文件不存在时返回各字段为空的 Metadata；解析失败仍返回 nullopt 并写 error。
// - 非 x86：USV_HOME/文件缺失或解析失败返回 nullopt 并写 error。
//
// 示例：
//   std::string err;
//   auto metadata = m_utils::load_metadata_from_usv_home(&err);
//   if (!metadata) {
//     // 非 x86 或 JSON 解析失败；x86 仅文件缺失时返回空字段 Metadata（has_value 为 true）
//     return;
//   }
//   if (metadata->sn.empty()) { /* x86 无 app/metadata.json 或 sn 未配置 */ }
std::optional<Metadata> load_metadata_from_usv_home(std::string* error = nullptr);

// 从 metadata.json 加载常用字段；缺失字段会返回空字符串。
bool load_metadata(const std::string& metadata_path, Metadata& out, std::string* error = nullptr);

// 读取 metadata.json 中指定 key 的字符串值；key 不存在返回 std::nullopt。
std::optional<std::string> load_metadata_string(
  const std::string& metadata_path, const std::string& key, std::string* error = nullptr);

}  // namespace m_utils
