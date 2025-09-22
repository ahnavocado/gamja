#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>

class DummyCloudPub : public rclcpp::Node {
public:
  DummyCloudPub() : Node("dummy_cloud_pub") {
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() { publishCloud(); });
  }
private:
  void publishCloud() {
    // 바닥면 z=0 근처 + 두 개의 박스형 장애물 포인트
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // 바닥 샘플
    for (float x=-10; x<=10; x+=0.2f)
      for (float y=-10; y<=10; y+=0.2f)
        cloud->push_back({x, y, 0.02f*std::sin(0.3f*x)+0.01f}); // 살짝 요철

    // 장애물 A (x=3~4, y=1~2, z=0~1)
    for (float x=3; x<=4; x+=0.03f)
      for (float y=1; y<=2; y+=0.03f)
        for (float z=0; z<=1; z+=0.1f)
          cloud->push_back({x, y, z});

    // 장애물 B (x=-2~-1, y=-3~-2, z=0~1.2)
    for (float x=-2; x<=-1; x+=0.03f)
      for (float y=-3; y<=-2; y+=0.03f)
        for (float z=0; z<=1.2f; z+=0.1f)
          cloud->push_back({x, y, z});

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "lidar";             // 입력 프레임
    msg.header.stamp = now();
    pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyCloudPub>());
  rclcpp::shutdown();
  return 0;
}
