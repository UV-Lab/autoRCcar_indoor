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


#include "planeFittingProcess.h"

#include <fstream>
#include <iomanip>
#include <iostream>

#include <Eigen/Eigenvalues>


namespace gac::lio_sam {

bool PlaneFittingProcess::Process(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloudKeyPoses3D){
   
   if (cloudKeyPoses3D->size() < 3){
      std::cerr << "Not enough points for plane fitting. Need at least 3, but got " << cloudKeyPoses3D->size() << std::endl;
      return false;
   }
   // Compute centroid
   Eigen::Vector3f centroid(0.0f, 0.0f, 0.0f);
   for (const auto &pt : cloudKeyPoses3D->points){
      centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
   }
   centroid /= static_cast<float>(cloudKeyPoses3D->size());

   // Compute covariance matrix
   Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
   for (const auto &pt : cloudKeyPoses3D->points)
   {
      Eigen::Vector3f d(pt.x, pt.y, pt.z);
      d -= centroid;
      cov += d * d.transpose();
   }
   cov /= static_cast<float>(cloudKeyPoses3D->size());

   // Eigen decomposition
   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eig_solver(cov);
   std::cout << "Estimated result: " << (eig_solver.info() == Eigen::Success) << std::endl;
   if (eig_solver.info() != Eigen::Success){
      std::cerr << "Eigen decomposition failed for plane fitting." << std::endl;
      return false;
   }

   // The normal of the plane is the eigenvector corresponding to the smallest eigenvalue
   Eigen::Vector3f normal = eig_solver.eigenvectors().col(0).normalized();
   if (normal.dot(Eigen::Vector3f::UnitZ()) < 0.0f){
      normal = -normal;
   }

   Eigen::Quaternionf q_align = Eigen::Quaternionf::FromTwoVectors(normal, Eigen::Vector3f::UnitZ());
   Eigen::Matrix3f rot_level = q_align.normalized().toRotationMatrix();
   //planeFittingRotation_ = Eigen::Matrix4f::Identity();
   planeFittingRotation_.block<3,3>(0,0) = rot_level;
   planeFittingRotation_(2,3) = kLidarVerticalHeight; // set translation in Z to the LiDAR height

   std::cout << "Estimated plane normal: " << normal.transpose() << std::endl;
   std::cout << "Rotation matrix (align normal to Z):\n" << rot_level << std::endl;
   return true;
} 

bool PlaneFittingProcess::ExportPlaneFittingResult(const std::string& saveMapDirectory){
       std::ofstream fs(saveMapDirectory + "/tf_new_old_mat.txt");
               if (!fs.is_open()) {
                  std::cerr << "Failed to open file for writing plane fitting result: " << saveMapDirectory + "/tf_new_old_mat.txt" << std::endl;
                  return false;
               }
               fs << std::fixed << std::setprecision(12);
               for (int row = 0; row < planeFittingRotation_.rows(); ++row) {
                  for (int col = 0; col < planeFittingRotation_.cols(); ++col) {
                     fs << planeFittingRotation_(row, col);
                     if (col + 1 < planeFittingRotation_.cols()) {
                        fs << " ";
                     }
                  }
                  fs << '\n';
               }

               if (!fs.good()) {
                  std::cerr << "Failed while writing plane fitting result: " << saveMapDirectory + "/tf_new_old_mat.txt" << std::endl;
                  return false;
               }

               std::cout << "Plane fitting matrix exported to: " << saveMapDirectory + "/tf_new_old_mat.txt" << std::endl;

               return true;
}

}
