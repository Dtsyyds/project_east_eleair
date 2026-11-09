#pragma once

#include <filesystem>
#include <stdexcept>
#include <string>

namespace fast_livo
{
// Both mapping and registration resolve paths against the FAST-LIVO2 source package.
inline std::filesystem::path resolveWorkspaceRoot(
    const std::string &configured_root, const std::filesystem::path &compiled_source_root)
{
  const bool automatic = configured_root == "auto";
  auto root = automatic ? compiled_source_root : std::filesystem::path(configured_root);
  if (root.empty() || !root.is_absolute())
    throw std::invalid_argument("workspace_root must be 'auto' or a non-empty absolute directory path.");

  root = root.lexically_normal();
  if (root != root.root_path() && root.filename().empty()) root = root.parent_path();
  if (automatic && (!std::filesystem::is_directory(root) ||
                    !std::filesystem::is_regular_file(root / "package.xml")))
    throw std::runtime_error(
        "Automatic workspace_root points to a missing FAST-LIVO2 source package: " + root.string() +
        ". Rebuild fast_livo and relocalization at the new source location, or set an absolute workspace_root.");
  return root;
}
}  // namespace fast_livo
