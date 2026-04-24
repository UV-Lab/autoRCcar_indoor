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

#include <string>

namespace gac::lio_sam {
    
constexpr const char* kErrorCode_Success = "0";

class RosLog {
 public: 
    RosLog(const std::string& log_file);
    ~RosLog()=default;
    bool Write(const std::string& log_output);
    bool Append(const std::string& log_output);

 private:
    bool initial_ = false;
    std::string log_file_;
};

}
