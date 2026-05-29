#include "m_utils/metadata_util.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

namespace m_utils {

namespace {

bool read_json_file(
  const std::string& metadata_path, nlohmann::json& out, std::string* error)
{
  std::ifstream ifs(metadata_path);
  if (!ifs.is_open()) {
    if (error != nullptr) {
      *error = "无法打开 metadata 文件: " + metadata_path;
    }
    return false;
  }

  try {
    ifs >> out;
  } catch (const std::exception& e) {
    if (error != nullptr) {
      *error = "解析 metadata JSON 失败: " + std::string(e.what());
    }
    return false;
  }

  if (!out.is_object()) {
    if (error != nullptr) {
      *error = "metadata 根节点必须是 JSON 对象";
    }
    return false;
  }

  return true;
}

std::string get_string_or_empty(const nlohmann::json& obj, const char* key)
{
  const auto it = obj.find(key);
  if (it == obj.end() || !it->is_string()) {
    return "";
  }
  return it->get<std::string>();
}

}  // namespace

std::optional<std::string> resolve_metadata_path_from_usv_home(std::string* error)
{
  const char* usv_home_env = std::getenv("USV_HOME");
  if (usv_home_env == nullptr || usv_home_env[0] == '\0') {
    if (error != nullptr) {
      *error = "环境变量 USV_HOME 未设置";
    }
    return std::nullopt;
  }

  std::filesystem::path metadata_path =
    std::filesystem::path(usv_home_env) / "app" / "metadata.json";
  if (!std::filesystem::exists(metadata_path)) {
    if (error != nullptr) {
      *error = "metadata 文件不存在: " + metadata_path.string();
    }
    return std::nullopt;
  }

  return metadata_path.string();
}

std::optional<Metadata> load_metadata_from_usv_home(std::string* error)
{
  auto metadata_path = resolve_metadata_path_from_usv_home(error);
  if (!metadata_path.has_value()) {
    return std::nullopt;
  }

  Metadata out;
  if (!load_metadata(*metadata_path, out, error)) {
    return std::nullopt;
  }
  return out;
}

bool load_metadata(const std::string& metadata_path, Metadata& out, std::string* error)
{
  nlohmann::json root;
  if (!read_json_file(metadata_path, root, error)) {
    return false;
  }

  out.boat_type = get_string_or_empty(root, "type");
  out.sn = get_string_or_empty(root, "sn");
  out.version = get_string_or_empty(root, "version");
  out.description = get_string_or_empty(root, "description");
  return true;
}

std::optional<std::string> load_metadata_string(
  const std::string& metadata_path, const std::string& key, std::string* error)
{
  nlohmann::json root;
  if (!read_json_file(metadata_path, root, error)) {
    return std::nullopt;
  }

  const auto it = root.find(key);
  if (it == root.end()) {
    return std::nullopt;
  }
  if (!it->is_string()) {
    if (error != nullptr) {
      *error = "metadata key 不是字符串: " + key;
    }
    return std::nullopt;
  }
  return it->get<std::string>();
}

}  // namespace m_utils
