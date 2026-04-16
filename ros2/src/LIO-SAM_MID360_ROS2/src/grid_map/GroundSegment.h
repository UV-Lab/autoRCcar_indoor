#ifndef DF36AF46_1E0E_4C9A_B356_D7A3D2872D42
#define DF36AF46_1E0E_4C9A_B356_D7A3D2872D42

#include <memory>

// #include <utility.h>

#include "Common.h"

namespace grid_map {

class GroundSegmentParams {
public:
    using SharedPtr = std::shared_ptr<GroundSegmentParams>;
    double min_height;
    double max_height;

    int num_iter;
    int num_lpr;
    double seed_th;
    double dist_th;
    double slope_angle_rad_th;
    // The height for simple segmentation.
    double seg_height_th;
};
class GroundSegment {
public:
    GroundSegment(GroundSegmentParams::SharedPtr params_ptr);
    virtual ~GroundSegment();

    int init();

    int groundSegment(CloudT::Ptr src_cloud_ptr);
    int groundSegment(CloudT::Ptr src_cloud_ptr,
                      CloudT::Ptr auxiliary_src_cloud_ptr);

    CloudT::Ptr getNonGroundCloud();
    CloudT::Ptr getGroundCloud();
    // Get the ground segmentation result point cloud.
    // CloudT::Ptr getGSCloud();

private:
    int preprocessCloud(CloudT::Ptr cloud_ptr);
    CloudT::Ptr extractInitSeeds(CloudT::Ptr sorted_cloud_ptr);
    int estimatePlane();

private:
    GroundSegmentParams::SharedPtr m_params_ptr = nullptr;
    CloudT::Ptr m_non_ground_cloud_ptr = nullptr;
    CloudT::Ptr m_ground_cloud_ptr = nullptr;
    std::array<float, 4> m_plane_abcd;
};

}// namespace grid_map

#endif /* DF36AF46_1E0E_4C9A_B356_D7A3D2872D42 */
