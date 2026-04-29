#ifndef A8F08A03_0E95_4E3C_9330_54C54D5ED117
#define A8F08A03_0E95_4E3C_9330_54C54D5ED117

#include <memory>

#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Eigen>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointRGBT = pcl::PointXYZRGB;
using PointCloudRGBT = pcl::PointCloud<PointRGBT>;


class SubMap {
public:
    using SharedPtr = std::shared_ptr<SubMap>;
    std::int64_t map_idx;
    PointCloudT::Ptr map_cloud_ptr;
    pcl::PointXY min_pnt;
    pcl::PointXY max_pnt;
};


class MapSplitterParams {
public:
    using SharedPtr = std::shared_ptr<MapSplitterParams>;
    float grid_sz;
    int min_points_num_in_grid;
};

class MapSplitter {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapSplitter(MapSplitterParams::SharedPtr params_ptr);
    virtual ~MapSplitter();

    int init();

    int splitMap(const PointCloudT::Ptr &cloud_in_ptr);

    int saveMap(const std::string &path);

private:
    MapSplitterParams::SharedPtr m_params_ptr;
    std::map<std::int64_t, SubMap> m_idx_map_map;
    // Eigen::ArrayXi m_grid_idx_mat;
    pcl::PointXY m_map_min_pnt;
    pcl::PointXY m_map_max_pnt;
    int m_grid_col_cnt = 0;
    int m_grid_row_cnt = 0;
};


#endif /* A8F08A03_0E95_4E3C_9330_54C54D5ED117 */
