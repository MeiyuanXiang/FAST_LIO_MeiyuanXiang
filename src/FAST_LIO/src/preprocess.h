#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;

#define IS_VALID(a) ((abs(a) > 1e8) ? true : false) // 没用到

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE
{
  AVIA = 1,
  VELO16,
  OUST64
}; //{1, 2, 3} 支持的雷达类型

enum Feature
{
  Nor,
  Poss_Plane,
  Real_Plane,
  Edge_Jump,
  Edge_Plane,
  Wire,
  ZeroPoint
}; // 特征类型：正常、可能的平面点、确定的平面点、有跨越的边、边上的平面点、线段、无效点

enum Surround
{
  Prev,
  Next
}; // 位置标识：前一个、后一个

enum E_jump
{
  Nr_nor,
  Nr_zero,
  Nr_180,
  Nr_inf,
  Nr_blind
}; // 有跨越边的类型：正常、0、180、无穷大、最大值

struct orgtype
{
  double range;     // 和邻近点的点距，用以确定是否为重复点，默认为当前点的x坐标，但是最后一个点取xy平面向量的模
  double dista;     // 当前点与最后一个点距离的平方
  double angle[2];  // 当前点与前一个点的夹角、当前点与后一个点的夹角
  double intersect; // 前一个点和后一个点夹角
  E_jump edj[2];    // 前后两点的类型
  Feature ftype;    // 点类型
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

// velodyne数据结构
namespace velodyne_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型
    float intensity;                // 强度
    float time;                     // 时间
    uint16_t ring;                  // 点所属的圈数
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
} // namespace velodyne_ros

// 注册velodyne_ros的Point类型
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, time, time)(uint16_t, ring, ring))

// ouster数据结构
namespace ouster_ros
{
  struct EIGEN_ALIGN16 Point
  {
    PCL_ADD_POINT4D;                // 4D点坐标类型
    float intensity;                // 强度
    uint32_t t;                     // 时间
    uint16_t reflectivity;          // 反射率
    uint8_t ring;                   // 点所属的圈数
    uint16_t ambient;               // 没用到
    uint32_t range;                 // 距离
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 进行内存对齐
  };
} // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

class Preprocess
{
  public:

  Preprocess();
  ~Preprocess();
  
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf; // 全部点、边缘点、平面点
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE; // 雷达类型、采样间隔、扫描线数、扫描频率
  double blind; // 最小距离阈值
  bool feature_enabled, given_offset_time; // 是否提取特征、是否进行时间偏移
  ros::Publisher pub_full, pub_surf, pub_corn; // 发布全部点、发布平面点、发布边缘点

  private:
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct); // 没有用到
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
