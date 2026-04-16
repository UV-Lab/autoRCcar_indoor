#ifndef A0AE325E_989C_4A69_8A15_B20EB1F317A1
#define A0AE325E_989C_4A69_8A15_B20EB1F317A1

#include <memory>
#include <pcl/common/io.h>
#include <pcl/common/impl/io.hpp>

#include "CloudToScan.h"
#include "GroundSegment.h"
#include "OccupancyMap.h"

namespace grid_map {


class GridMapMakerParams {
public:
    using SharedPtr = std::shared_ptr<GridMapMakerParams>;
    GroundSegmentParams::SharedPtr m_gs_params_ptr;
    CloudToScanParams::SharedPtr m_cts_params_ptr;
    OccupancyMapParams::SharedPtr m_occ_map_params_ptr;
};

class GridMapMaker {
public:
    GridMapMaker(GridMapMakerParams::SharedPtr params_ptr);
    virtual ~GridMapMaker();
    int init();

    int addFrameToGridMap(const Eigen::Matrix4f &tf_world_base,
                          CloudT::Ptr base_cloud_ptr);

    int addFrameToGridMap(const Eigen::Matrix4f &tf_world_base,
                          CloudT::Ptr base_cloud_ptr,
                          CloudT::Ptr sub_map_in_base);

    int saveGridMap(const std::string &file_path);

private:
    GridMapMakerParams::SharedPtr m_params_ptr = nullptr;
    std::shared_ptr<GroundSegment> m_gs_ptr = nullptr;
    std::shared_ptr<CloudToScan> m_cts_ptr = nullptr;
    std::shared_ptr<OccupancyMap> m_occ_map_ptr = nullptr;
    std::shared_ptr<OccupancyMap> m_obs_occ_map_ptr = nullptr;


    CloudT::Ptr m_ground_cloud_ptr = nullptr;
    CloudT::Ptr m_non_ground_cloud_ptr = nullptr;
};

}// namespace grid_map

#endif /* A0AE325E_989C_4A69_8A15_B20EB1F317A1 */
