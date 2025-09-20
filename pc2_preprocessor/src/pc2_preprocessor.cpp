#include "pc2_preprocessor/pc2_preprocessor.hpp"

using std::placeholders::_1;

Pc2Preprocessor::Pc2Preprocessor()
: Node("pc2_preprocessor")
{
  // 파라미터 선언
  this->declare_parameter<std::string>("input_topic", "/pointcloud");
  this->declare_parameter<std::string>("filtered_cloud_topic", "/pointcloud/filtered");
  this->declare_parameter<std::string>("grid_topic", "/map");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<double>("voxel_leaf", 0.15);
  this->declare_parameter<double>("z_min", -1.0);
  this->declare_parameter<double>("z_max",  3.0);
  this->declare_parameter<double>("ground_dist_thresh", 0.15);
  this->declare_parameter<double>("grid_res", 0.10);
  this->declare_parameter<double>("x_lim", 20.0);
  this->declare_parameter<double>("y_lim", 20.0);
  this->declare_parameter<double>("inflate_radius", 0.3);

  // 파라미터 값 로드
  this->get_parameter("input_topic", input_topic_);
  this->get_parameter("filtered_cloud_topic", filtered_cloud_topic_);
  this->get_parameter("grid_topic", grid_topic_);
  this->get_parameter("map_frame", map_frame_);
  this->get_parameter("voxel_leaf", voxel_leaf_);
  this->get_parameter("z_min", z_min_);
  this->get_parameter("z_max", z_max_);
  this->get_parameter("ground_dist_thresh", ground_dist_thresh_);
  this->get_parameter("grid_res", grid_res_);
  this->get_parameter("x_lim", x_lim_);
  this->get_parameter("y_lim", y_lim_);
  this->get_parameter("inflate_radius", inflate_radius_);

  // QoS: 센서 데이터용
  auto qos = rclcpp::SensorDataQoS();

  sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    input_topic_, qos, std::bind(&Pc2Preprocessor::cloudCallback, this, _1));

  pub_filtered_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_cloud_topic_, 10);
  pub_grid_     = this->create_publisher<nav_msgs::msg::OccupancyGrid>(grid_topic_, 1);

  RCLCPP_INFO(this->get_logger(), "pc2_preprocessor started. Subscribing: %s", input_topic_.c_str());
}

void Pc2Preprocessor::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  // msg -> PCL 변환
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromROSMsg(*msg, *cloud);

  if (cloud->empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Input cloud is empty.");
    return;
  }

  // 1) Z-clip
  pcl::PointCloud<pcl::PointXYZ>::Ptr clipped(new pcl::PointCloud<pcl::PointXYZ>());
  clipped->reserve(cloud->size());
  for (const auto &p : cloud->points) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      if (p.z >= z_min_ && p.z <= z_max_) clipped->push_back(p);
    }
  }

  // 2) VoxelGrid 다운샘플
  pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(clipped);
  vg.setLeafSize(voxel_leaf_, voxel_leaf_, voxel_leaf_);
  vg.filter(*down);

  // 3) 바닥 제거 (RANSAC plane)
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(ground_dist_thresh_);
  seg.setMaxIterations(200);
  seg.setInputCloud(down);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  seg.segment(*inliers, *coeff);

  pcl::PointCloud<pcl::PointXYZ>::Ptr non_ground(new pcl::PointCloud<pcl::PointXYZ>());
  if (!inliers->indices.empty()) {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(down);
    extract.setIndices(inliers);
    extract.setNegative(true);  // 바닥 아닌 포인트만 추출
    extract.filter(*non_ground);
  } else {
    // 평면 못 찾으면 다운샘플 결과 사용
    non_ground = down;
  }

  // 4) 결과 점군 퍼블리시
  sensor_msgs::msg::PointCloud2 out_cloud;
  pcl::toROSMsg(*non_ground, out_cloud);
  out_cloud.header = msg->header;  // 원본과 동일한 frame 유지(권장: 나중에 tf로 map 변환)
  pub_filtered_->publish(out_cloud);

  // 5) 2D OccupancyGrid 투영 퍼블리시
  auto grid = makeGridMsg(msg->header, non_ground);
  pub_grid_->publish(grid);
}

nav_msgs::msg::OccupancyGrid
Pc2Preprocessor::makeGridMsg(const std_msgs::msg::Header &hdr,
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) const
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header = hdr;
  grid.header.frame_id = map_frame_; // 맵 프레임으로 발행(필요시 static tf로 정렬)
  grid.info.resolution = grid_res_;
  const int width  = static_cast<int>((2.0 * x_lim_) / grid_res_);
  const int height = static_cast<int>((2.0 * y_lim_) / grid_res_);
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = -x_lim_;
  grid.info.origin.position.y = -y_lim_;
  grid.info.origin.position.z = 0.0;
  grid.info.origin.orientation.w = 1.0;

  std::vector<int8_t> data(width * height, 0);

  // 점 → grid index
  auto to_index = [&](double x, double y, int &ix, int &iy) {
    ix = static_cast<int>((x + x_lim_) / grid_res_);
    iy = static_cast<int>((y + y_lim_) / grid_res_);
    return (ix >= 0 && ix < width && iy >= 0 && iy < height);
  };

  for (const auto &p : cloud->points) {
    int ix, iy;
    if (to_index(p.x, p.y, ix, iy)) {
      data[iy * width + ix] = 100; // occupied
    }
  }

  // 간단 인플레이션 (원형 반경 기반)
  if (inflate_radius_ > 1e-6) {
    const int r = static_cast<int>(inflate_radius_ / grid_res_);
    std::vector<int8_t> inflated = data;
    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        if (data[y * width + x] != 100) continue;
        for (int dy = -r; dy <= r; ++dy) {
          for (int dx = -r; dx <= r; ++dx) {
            if (dx*dx + dy*dy <= r*r) {
              int nx = x + dx, ny = y + dy;
              if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                inflated[ny * width + nx] = 100;
              }
            }
          }
        }
      }
    }
    data.swap(inflated);
  }

  grid.data = std::move(data);
  return grid;
}

// 노드 실행
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Pc2Preprocessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
