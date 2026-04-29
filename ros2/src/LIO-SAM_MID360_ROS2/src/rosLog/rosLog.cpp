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


#include "rosLog.h"

#include <fstream>
#include <iostream>
#include <string>
#include <filesystem>



namespace gac::lio_sam {


RosLog::RosLog(const std::string& log_file)  {

   // log_file 是一个文件路径，包含文件名和后缀，例如 "/path/to/log_file.txt"
   std::cout << "===================================Initializing RosLog with file: " << log_file << std::endl;
   // 首先检查文件的目录是否存在，如果不存在则创建目录
   std::filesystem::path logFilePath(log_file);
   std::filesystem::path logDir = logFilePath.parent_path();
   if (!std::filesystem::exists(logDir)) {
         std::filesystem::create_directories(logDir);
   }
   initial_ = true;
   // 然后创建或覆盖日志文件，并写入初始内容
   log_file_ = log_file;
   Write(kErrorCode_Success);
}

bool RosLog::Write(const std::string& error_code) {

   if (!initial_) {
      std::cerr << "RosLog not initialized properly." << std::endl;
      return false;
   }
   
   // 首先先判断文件是否存在，如果存在读入里面的东西
   std::string error_code_str;
   std::ifstream inFile(log_file_, std::ios::in);
   if (inFile.is_open()) {
      error_code_str.assign((std::istreambuf_iterator<char>(inFile)),
                              std::istreambuf_iterator<char>());
      inFile.close();
      if (error_code_str != "{\"error_code\": " + std::string(kErrorCode_Success) + "}") {
         std::cerr << "File already exists and error_code: " << error_code << std::endl;
         return true;
      }
   }

   
   std::ofstream outFile(log_file_, std::ios::out);
   if (!outFile.is_open()) {
      std::cerr << "Failed to open file for writing: " << log_file_ << std::endl;
      return false;
   }
   outFile <<  "{\"error_code\": " << error_code << "}";
   outFile.close();
   std::cout << "Write to log file: " << log_file_ << " with error_code: " << error_code << std::endl;
   return true;
} 


}
