#include "GridMapMaker.h"

#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>

#include "RobLog.h"

using namespace grid_map;

GridMapMaker::GridMapMaker(GridMapMakerParams::SharedPtr params_ptr)
    : m_params_ptr(params_ptr) {}
GridMapMaker::~GridMapMaker() {}
int GridMapMaker::init() {
    ROB_LOG_INFO("Create the ground segmentation.");
    m_gs_ptr = std::make_shared<GroundSegment>(m_params_ptr->m_gs_params_ptr);
    m_gs_ptr->init();
    ROB_LOG_INFO("Create the cloud to scan object.");
    m_cts_ptr = std::make_shared<CloudToScan>(m_params_ptr->m_cts_params_ptr);
    ROB_LOG_INFO("Create the OccupancyMap.");
    m_occ_map_ptr =
            std::make_shared<OccupancyMap>(m_params_ptr->m_occ_map_params_ptr);
    m_occ_map_ptr->init();

    OccupancyMapParams::SharedPtr obs_occ_map_params_ptr =
            std::make_shared<OccupancyMapParams>();
    *obs_occ_map_params_ptr = *(m_params_ptr->m_occ_map_params_ptr);
    obs_occ_map_params_ptr->grid_map_name += "_obs";
    m_obs_occ_map_ptr = std::make_shared<OccupancyMap>(obs_occ_map_params_ptr);
    m_obs_occ_map_ptr->init();

    m_ground_cloud_ptr.reset(new CloudT());
    m_non_ground_cloud_ptr.reset(new CloudT());

    return 0;
}

int GridMapMaker::addFrameToGridMap(const Eigen::Matrix4f &tf_world_base,
                                    CloudT::Ptr base_cloud_ptr) {
    auto rc = m_gs_ptr->groundSegment(base_cloud_ptr);
    if (rc != 0) {
        ROB_LOG_WARN("Ground segment failed.");
        return -1;
    }

    auto ground_cloud_ptr = m_gs_ptr->getGroundCloud();
    auto non_ground_cloud_ptr = m_gs_ptr->getNonGroundCloud();

    CloudT::Ptr world_non_ground_cloud_ptr(new CloudT());
    pcl::transformPointCloud(*non_ground_cloud_ptr, *world_non_ground_cloud_ptr,
                             tf_world_base);
    *m_non_ground_cloud_ptr += *world_non_ground_cloud_ptr;
    CloudT::Ptr world_ground_cloud_ptr(new CloudT());
    pcl::transformPointCloud(*ground_cloud_ptr, *world_ground_cloud_ptr,
                             tf_world_base);
    *m_ground_cloud_ptr += *world_ground_cloud_ptr;

    std::vector<cv::Point2f> world_pnts;
    if (non_ground_cloud_ptr->size() > 10) {
        rc = m_cts_ptr->convertCloudToScan(tf_world_base, non_ground_cloud_ptr,
                                           world_pnts);
    } else {
        ROB_LOG_WARN("The non ground cloud size < 10. return.");
        return -2;
    }

    cv::Point2f pnt_w_l(tf_world_base(0, 3), tf_world_base(1, 3));
    m_occ_map_ptr->addLidar2dFrame(pnt_w_l, world_pnts);

    return 0;
}

int GridMapMaker::addFrameToGridMap(const Eigen::Matrix4f &tf_world_base,
                                    CloudT::Ptr base_cloud_ptr,
                                    CloudT::Ptr sub_map_in_base) {
    auto rc = m_gs_ptr->groundSegment(base_cloud_ptr, sub_map_in_base);
    if (rc != 0) {
        ROB_LOG_WARN("Ground segment failed.");
        return -1;
    }

    auto ground_cloud_ptr = m_gs_ptr->getGroundCloud();
    auto non_ground_cloud_ptr = m_gs_ptr->getNonGroundCloud();


    std::cout << "non_ground_cloud_ptr size = " << non_ground_cloud_ptr->points.size()
              << std::endl;
    std::cout << "ground_cloud_ptr size = " << ground_cloud_ptr->points.size()
              << std::endl;

    CloudT::Ptr world_non_ground_cloud_ptr(new CloudT());
    pcl::transformPointCloud(*non_ground_cloud_ptr, *world_non_ground_cloud_ptr,
                             tf_world_base);

    std::cout << "make be ok" << std::endl;


    *m_non_ground_cloud_ptr += *world_non_ground_cloud_ptr;
    CloudT::Ptr world_ground_cloud_ptr(new CloudT());
    pcl::transformPointCloud(*ground_cloud_ptr, *world_ground_cloud_ptr,
                             tf_world_base);
    *m_ground_cloud_ptr += *world_ground_cloud_ptr;

    std::vector<cv::Point2f> world_pnts;
    if (non_ground_cloud_ptr->size() > 10) {
        rc = m_cts_ptr->convertCloudToScan(tf_world_base, non_ground_cloud_ptr,
                                           world_pnts);
    } else {
        ROB_LOG_WARN("The non ground cloud size < 10. return.");
        return -2;
    }

    cv::Point2f pnt_w_l(tf_world_base(0, 3), tf_world_base(1, 3));
    m_occ_map_ptr->addLidar2dFrame(pnt_w_l, world_pnts);

    return 0;
}

int GridMapMaker::saveGridMap(const std::string &file_path) {
    auto rc = m_occ_map_ptr->saveGridMap(file_path);

    if (true) {

        ROB_LOG_INFO("Save the non ground cloud and ground cloud to %s.",
                     file_path.c_str());
        if (m_ground_cloud_ptr->size() > 0) {
            pcl::io::savePLYFile(file_path + "/non_ground.ply",
                                 *m_non_ground_cloud_ptr);
        }
        if (m_non_ground_cloud_ptr->size() > 0) {
            pcl::io::savePLYFile(file_path + "/ground.ply", *m_ground_cloud_ptr);
        }
    }

    bool is_save_obs_grid_map = true;
    if (true == is_save_obs_grid_map) {
        std::vector<cv::Point2f> world_pnts_2d;
        for (const auto &ele_pnt_3d : m_non_ground_cloud_ptr->points) {
            world_pnts_2d.emplace_back(cv::Point2f(ele_pnt_3d.x, ele_pnt_3d.y));
        }
        m_obs_occ_map_ptr->addWholeObstacleCloud(world_pnts_2d);
        m_obs_occ_map_ptr->saveGridMap(file_path);
    }

    return rc;

    // auto grim_map_img = m_occ_map_ptr->getOccupancyMapBlackWhite();
    // cv::imwrite("./grid_map.png", grim_map_img);
    return 0;
}