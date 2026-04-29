////////////////////////////////////////////////////////////////////////////////
//                                                                            //
//    Copyright© 2025 GAC Inc. and GAC Motors, All Rights Reserved.           //
//                                                                            //
//  All users are hereby notified that the materials in the form of digital   //
//  information available from this software (content, designs, color         //
//  schemes, graphic styles, images, logo, text, and videos) comes protected  //
//  under International Copyright Laws. Therefore it should not be reproduced //
//  in any form digital or offline without prior written permission of        //
//  GAC Inc. and GAC Motors.                                                  //
//                                                                            //
//  Any unauthorized reprint or material usage (GAC Inc.,                     //
//  and GAC Motors) either manually or digitally, is strictly                 //
//  prohibited.                                                               //
//                                                                            //
//  Any further unauthorized digital copying of this material via copying,    //
//  publication, reproduction or distribution of copyrighted works is an      //
//  infringement of the copyright owners' rights may be the subject of the    //
//  copyright of performers' protection under the Copyright Act. For such     //
//  illegal activities you will be strictly liable to GAC Inc.,               //
//  and GAC Motors for any and/or all damages (including                      //
//  recovery of attorneys' fees) which may be suffered and/ or incurred as a  //
//  result of your infringement.                                              //
//                                                                            //
////////////////////////////////////////////////////////////////////////////////


#pragma once

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>


namespace gac::lio_sam {
    
// LiDAR vertical height from the ground in meters 
// 这个值应该在lidar初始化的时候就确定了，后续不应该再修改了，否则会导致地图保存时的平面拟合结果不正确。
constexpr float kLidarVerticalHeight = 1.2f;  

class PlaneFittingProcess {
 public: 
    PlaneFittingProcess()=default;
    ~PlaneFittingProcess()=default;
    bool Process(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudKeyPoses3D);

    bool ExportPlaneFittingResult(const std::string& saveMapDirectory);

 private:
    Eigen::Matrix4f planeFittingRotation_  = Eigen::Matrix4f::Identity();
};

}
