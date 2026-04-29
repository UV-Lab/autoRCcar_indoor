#ifndef B5BF0386_7C03_4F04_900B_5386C8471E19
#define B5BF0386_7C03_4F04_900B_5386C8471E19

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

namespace msf_util {

template<typename PointT>
inline int loadPointCloudFile(const std::string &file_name, pcl::PointCloud<PointT> &cloud) {
    // extract the subfix of file_name
    std::string subfix = file_name.substr(file_name.find_last_of(".") + 1);
    if ("pcd" == subfix || "PCD" == subfix) {
        if (0 != pcl::io::loadPCDFile<PointT>(file_name, cloud)) {
            return -1;
        }
    } else if ("ply" == subfix || "PLY" == subfix) {
        if (0 != pcl::io::loadPLYFile<PointT>(file_name, cloud)) {
            return -1;
        }
    }
    return 0;
}

}// namespace msf_util

#endif /* B5BF0386_7C03_4F04_900B_5386C8471E19 */
