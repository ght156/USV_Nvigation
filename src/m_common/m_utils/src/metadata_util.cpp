#include "m_utils/metadata_util.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

namespace m_utils {

namespace {

bool is_x86_arch()
{
#if defined(__x86_64__) || defined(__amd64__) || defined(_M_X64) || defined(__i386__) || \
  defined(_M_IX86)
  return true;
#else
  return false;
#endif
}

// 元数据根目录：USV_HOME；x86 在未设置 USV_HOME 时回退为当前工作目录。
std::optional<std::filesystem::path> resolve_metadata_root_dir(std::string* error)
{
  const char* usv_home_env = std::getenv("USV_HOME");
  if (usv_home_env != nullptr && usv_home_env[0] != '\0')
  {
    return std::filesystem::path(usv_home_env);
  }

  if (is_x86_arch())
  {
    std::error_code ec;
    const auto cwd = std::filesystem::current_path(ec);
    if (!ec)
    {
      return cwd;
    }
    if (error != nullptr)
    {
      *error = "无法获取当前工作目录: " + ec.message();
    }
    return std::nullopt;
  }

  if (error != nullptr)
  {
    *error = "环境变量 USV_HOME 未设置";
  }
  return std::nullopt;
}

std::filesystem::path metadata_json_path(const std::filesystem::path& root)
{
  return root / "app" / "metadata.json";
}

bool read_json_file(
  const std::string& metadata_path, nlohmann::json& out, std::string* error)
{
  std::ifstream ifs(metadata_path);
  if (!ifs.is_open())
  {
    if (error != nullptr)
    {
      *error = "无法打开 metadata 文件: " + metadata_path;
    }
    return false;
  }

  try
  {
    ifs >> out;
  } catch (const std::exception& e)
  {
    if (error != nullptr)
    {
      *error = "解析 metadata JSON 失败: " + std::string(e.what());
    }
    return false;
  }

  if (!out.is_object())
  {
    if (error != nullptr)
    {
      *error = "metadata 根节点必须是 JSON 对象";
    }
    return false;
  }

  return true;
}

std::string get_string_or_empty(const nlohmann::json& obj, const char* key)
{
  const auto it = obj.find(key);
  if (it == obj.end() || !it->is_string())
  {
    return "";
  }
  return it->get<std::string>();
}

}  // namespace

std::optional<std::string> resolve_metadata_path_from_usv_home(std::string* error)
{
  const auto root = resolve_metadata_root_dir(error);
  if (!root.has_value())
  {
    return std::nullopt;
  }

  const std::filesystem::path path = metadata_json_path(*root);
  if (!std::filesystem::exists(path))
  {
    if (is_x86_arch())
    {
      if (error != nullptr)
      {
        error->clear();
      }
      return std::nullopt;
    }
    if (error != nullptr)
    {
      *error = "metadata 文件不存在: " + path.string();
    }
    return std::nullopt;
  }

  return path.string();
}

std::optional<Metadata> load_metadata_from_usv_home(std::string* error)
{
  const auto root = resolve_metadata_root_dir(error);
  if (!root.has_value())
  {
    return std::nullopt;
  }

  const std::filesystem::path path = metadata_json_path(*root);
  if (!std::filesystem::exists(path))
  {
    if (is_x86_arch())
    {
      if (error != nullptr)
      {
        error->clear();
      }
      return Metadata{};
    }
    if (error != nullptr)
    {
      *error = "metadata 文件不存在: " + path.string();
    }
    return std::nullopt;
  }

  Metadata out;
  if (!load_metadata(path.string(), out, error))
  {
    return std::nullopt;
  }
  return out;
}

bool load_metadata(const std::string& metadata_path, Metadata& out, std::string* error)
{
  nlohmann::json root;
  if (!read_json_file(metadata_path, root, error))
  {
    return false;
  }

  out.boat_type = get_string_or_empty(root, "type");
  out.sn        = get_string_or_empty(root, "sn");
  out.version   = get_string_or_empty(root, "version");
  out.description = get_string_or_empty(root, "description");
  return true;
}

std::optional<std::string> load_metadata_string(
  const std::string& metadata_path, const std::string& key, std::string* error)
{
  nlohmann::json root;
  if (!read_json_file(metadata_path, root, error))
  {
    return std::nullopt;
  }

  const auto it = root.find(key);
  if (it == root.end())
  {
    return std::nullopt;
  }
  if (!it->is_string())
  {
    if (error != nullptr)
    {
      *error = "metadata key 不是字符串: " + key;
    }
    return std::nullopt;
  }
  return it->get<std::string>();
}

}  // namespace m_utils
