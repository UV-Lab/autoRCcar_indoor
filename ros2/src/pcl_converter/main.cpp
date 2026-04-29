#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include <cstring>
#include <iostream>
#include <string>

static bool isDirectory(const std::string& path) {
  struct stat info;
  return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

static bool createDirectoryRecursive(const std::string& path) {
  if (path.empty()) {
    return false;
  }

  std::string normalized = path;
  while (!normalized.empty() && normalized.back() == '/') {
    normalized.pop_back();
  }
  if (normalized.empty()) {
    return false;
  }
  if (isDirectory(normalized)) {
    return true;
  }

  std::string prefix;
  size_t pos = 0;
  if (normalized[0] == '/') {
    prefix = "/";
    pos = 1;
  }

  while (pos < normalized.size()) {
    size_t next = normalized.find('/', pos);
    std::string part = normalized.substr(pos, next == std::string::npos ? std::string::npos : next - pos);
    if (!part.empty()) {
      if (!prefix.empty() && prefix.back() != '/') {
        prefix += "/";
      }
      prefix += part;
      if (!isDirectory(prefix)) {
        if (mkdir(prefix.c_str(), 0755) != 0 && errno != EEXIST) {
          return false;
        }
      }
    }
    if (next == std::string::npos) {
      break;
    }
    pos = next + 1;
  }

  return isDirectory(normalized);
}

static std::string joinPath(const std::string& dir, const std::string& name) {
  if (dir.empty()) {
    return name;
  }
  if (dir.back() == '/') {
    return dir + name;
  }
  return dir + "/" + name;
}

static bool endsWithPCD(const std::string& filename) {
  if (filename.size() < 4) {
    return false;
  }
  std::string extension = filename.substr(filename.size() - 4);
  return extension == ".pcd" || extension == ".PCD";
}

static std::string replaceExtensionWithPLY(const std::string& filename) {
  std::string base = filename;
  if (endsWithPCD(base)) {
    base.resize(base.size() - 4);
  }
  return base + ".ply";
}

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "用法: " << argv[0] << " <输入目录> <输出目录>" << std::endl;
    return -1;
  }

  std::string input_dir = argv[1];
  std::string output_dir = argv[2];

  if (!isDirectory(input_dir)) {
    std::cerr << "输入目录不存在或不是目录: " << input_dir << std::endl;
    return -1;
  }

  if (!isDirectory(output_dir)) {
    std::cout << "输出目录不存在，尝试创建: " << output_dir << std::endl;
    if (!createDirectoryRecursive(output_dir)) {
      std::cerr << "无法创建输出目录: " << output_dir << std::endl;
      return -1;
    }
  }

  DIR* dir = opendir(input_dir.c_str());
  if (!dir) {
    std::cerr << "无法打开输入目录: " << input_dir << " (" << strerror(errno) << ")" << std::endl;
    return -1;
  }

  struct dirent* entry;
  bool any_converted = false;
  bool failed = false;

  while ((entry = readdir(dir)) != nullptr) {
    if (entry->d_type != DT_REG && entry->d_type != DT_UNKNOWN) {
      continue;
    }
    std::string filename(entry->d_name);
    if (!endsWithPCD(filename)) {
      continue;
    }

    any_converted = true;
    std::string input_file = joinPath(input_dir, filename);
    std::string output_file = joinPath(output_dir, replaceExtensionWithPLY(filename));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1) {
      std::cerr << "无法读取PCD文件: " << input_file << std::endl;
      failed = true;
      continue;
    }

    if (pcl::io::savePLYFileASCII(output_file, *cloud) != 0) {
      std::cerr << "保存PLY文件失败: " << output_file << std::endl;
      failed = true;
      continue;
    }

    std::cout << "转换成功: " << input_file << " -> " << output_file << " (" << cloud->points.size() << " 个点)" << std::endl;
  }

  closedir(dir);

  if (!any_converted) {
    std::cerr << "输入目录中未找到任何 .pcd 文件: " << input_dir << std::endl;
    return -1;
  }

  return failed ? -1 : 0;
}
