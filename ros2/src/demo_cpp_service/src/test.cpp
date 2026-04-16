#include "chapt4_interface/srv/facedetect.hpp"
using FaceDetect = chapt4_interfaces::srv::FaceDetect;
#include "rclcpp/rclcpp.hpp"


class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("turtle_controller")
    {
        
             // 3.创建服务
    face_server_ = this->create_service<FaceDetect>(
        "facedetect",
        [&](const std::shared_ptr<FaceDetect::Request> request,
            std::shared_ptr<FaceDetect::Response> response) -> void {
          // 判断巡逻点是否在模拟器边界内
        //   if ((0 < request->target_x && request->target_x < 12.0f)
        //    && (0 < request->target_y && request->target_y < 12.0f)) {
        //     target_x_ = request->target_x;
        //     target_y_ = request->target_y;
        //     response->result = FaceDetect::Response::SUCCESS;
        //   }else{
        //     response->result = FaceDetect::Response::FAIL;
        //   }
        response->number = 16;
        });
            // 声明和获取参数初始值
    // this->declare_parameter("k", 1.0);
    // this->declare_parameter("max_speed", 1.0);
    // this->get_parameter("k", k_);
    // this->get_parameter("max_speed", max_speed_);

    }


private:
  // 2.添加 FaceDetect 类型服务共享指针 face_server_ 为成员变量
  rclcpp::Service<FaceDetect>::SharedPtr face_server_;

   

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}