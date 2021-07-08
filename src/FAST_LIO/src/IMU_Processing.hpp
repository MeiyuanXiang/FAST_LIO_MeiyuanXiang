#include <cmath>
#include <math.h>
#include <deque>
#include <mutex>
#include <thread>
#include <fstream>
#include <csignal>
#include <ros/ros.h>
#include <so3_math.h>
#include <Eigen/Eigen>
#include <common_lib.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <fast_lio/States.h>
#include <geometry_msgs/Vector3.h>

/*
  IMU_Processing主要进行IMU预处理。
*/

/// *************Preconfiguration

#define MAX_INI_COUNT (200)

const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); };

/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Process(const MeasureGroup &meas, StatesGroup &state, PointCloudXYZI::Ptr pcl_un_);
  void Reset();
  void IMU_Initial(const MeasureGroup &meas, StatesGroup &state, int &N);
  void IntegrateGyr(const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu);
  void UndistortPcl(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_in_out);

  ros::NodeHandle nh;

  void Integrate(const sensor_msgs::ImuConstPtr &imu);
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);

  double scale_gravity;

  Eigen::Vector3d angvel_last;
  Eigen::Vector3d acc_s_last;

  Eigen::Matrix<double, DIM_OF_PROC_N, 1> cov_proc_noise;

  Eigen::Vector3d cov_acc;
  Eigen::Vector3d cov_gyr;

  std::ofstream fout;

private:
  /*** Whether is the first frame, init for first frame ***/
  bool b_first_frame_ = true;
  bool imu_need_init_ = true;

  int init_iter_num = 1;
  Eigen::Vector3d mean_acc;
  Eigen::Vector3d mean_gyr;

  /*** Undistorted pointcloud ***/
  PointCloudXYZI::Ptr cur_pcl_un_;

  //// For timestamp usage
  sensor_msgs::ImuConstPtr last_imu_;

  /*** For gyroscope integration ***/
  double start_timestamp_;
  /// Making sure the equal size: v_imu_ and v_rot_
  std::deque<sensor_msgs::ImuConstPtr> v_imu_;
  std::vector<Eigen::Matrix3d> v_rot_pcl_;
  std::vector<Pose6D> IMUpose;
};

ImuProcess::ImuProcess()
    : b_first_frame_(true), imu_need_init_(true), last_imu_(nullptr), start_timestamp_(-1)
{
  Eigen::Quaterniond q(0, 1, 0, 0);
  Eigen::Vector3d t(0, 0, 0);
  init_iter_num = 1;
  scale_gravity = 1.0;
  cov_acc = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_gyr = Eigen::Vector3d(0.1, 0.1, 0.1);
  mean_acc = Eigen::Vector3d(0, 0, -1.0);
  mean_gyr = Eigen::Vector3d(0, 0, 0);
  angvel_last = Zero3d;
  cov_proc_noise = Eigen::Matrix<double, DIM_OF_PROC_N, 1>::Zero();
}

ImuProcess::~ImuProcess() { fout.close(); }

void ImuProcess::Reset()
{
  ROS_WARN("Reset ImuProcess");
  scale_gravity = 1.0;
  angvel_last = Zero3d;
  cov_proc_noise = Eigen::Matrix<double, DIM_OF_PROC_N, 1>::Zero();

  cov_acc = Eigen::Vector3d(0.1, 0.1, 0.1);
  cov_gyr = Eigen::Vector3d(0.1, 0.1, 0.1);
  mean_acc = Eigen::Vector3d(0, 0, -1.0);
  mean_gyr = Eigen::Vector3d(0, 0, 0);

  imu_need_init_ = true;
  b_first_frame_ = true;
  init_iter_num = 1;

  last_imu_ = nullptr;

  start_timestamp_ = -1;
  v_imu_.clear();
  IMUpose.clear();

  cur_pcl_un_.reset(new PointCloudXYZI());
  fout.close();
}

void ImuProcess::IMU_Initial(const MeasureGroup &meas, StatesGroup &state_inout, int &N)
{
  /** 1. initializing the gravity, gyro bias, acc and gyro covariance
   ** 2. normalize the acceleration measurenments to unit gravity **/
  ROS_INFO("IMU Initializing: %.1f %%", double(N) / MAX_INI_COUNT * 100);
  Eigen::Vector3d cur_acc, cur_gyr;

  if (b_first_frame_)
  {
    Reset();
    N = 1;
    b_first_frame_ = false;
  }

  for (const auto &imu : meas.imu)
  {
    const auto &imu_acc = imu->linear_acceleration;
    const auto &gyr_acc = imu->angular_velocity;
    cur_acc << imu_acc.x, imu_acc.y, imu_acc.z;
    cur_gyr << gyr_acc.x, gyr_acc.y, gyr_acc.z;

    scale_gravity += (cur_acc.norm() - scale_gravity) / N;
    mean_acc += (cur_acc - mean_acc) / N;
    mean_gyr += (cur_gyr - mean_gyr) / N;

    // cwiseProduct：输出相同位置的两个矩阵中各个系数的乘积所组成的矩阵
    cov_acc = cov_acc * (N - 1.0) / N + (cur_acc - mean_acc).cwiseProduct(cur_acc - mean_acc) * (N - 1.0) / (N * N);
    cov_gyr = cov_gyr * (N - 1.0) / N + (cur_gyr - mean_gyr).cwiseProduct(cur_gyr - mean_gyr) * (N - 1.0) / (N * N);

    N++;
  }

  state_inout.gravity = -mean_acc / scale_gravity * G_m_s2;
  state_inout.rot_end = Eye3d; // Exp(mean_acc.cross(Eigen::Vector3d(0, 0, -1 / scale_gravity)));
  state_inout.bias_g = mean_gyr;
}

void ImuProcess::UndistortPcl(const MeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out)
{
  /*
    IMU预积分
    把堆积的IMU数据进行预积分处理，推导出每个IMU数据时刻相对于全局坐标系的T，以及对应的方差。
    每次开始之前，都会把上一次预积分的最后一个IMU加入到本次IMU队列的头部，以及对应的姿态结果作为此次预积分的开始姿态。
    这种推理会导致IMU推理的轨迹飞的很快，但是不影响我们总的结果，因为我们取的是短时间内的相对运动姿态。
    这里IMU的预积分使用中值积分。
  */

  /* 数据准备 */

  /*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_); // 让IMU的时间能包住Lidar的时间
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = meas.lidar_beg_time;

  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  std::sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
  std::cout << "[ IMU Process ]: Process lidar from " << pcl_beg_time << " to " << pcl_end_time << ", "
            << meas.imu.size() << " imu msgs from " << imu_beg_time << " to " << imu_end_time << std::endl;

  /*** Initialize IMU pose ***/
  IMUpose.clear();
  // IMUpose.push_back(set_pose6d(0.0, Zero3d, Zero3d, state.vel_end, state.pos_end, state.rot_end));
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));

  /* 预积分过程 */

  /*** forward propagation at each imu point ***/
  Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  Eigen::Matrix3d R_imu(state_inout.rot_end);
  Eigen::MatrixXd F_x(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity());
  Eigen::MatrixXd cov_w(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Zero());
  double dt = 0;
  for (auto it_imu = v_imu.begin(); it_imu != (v_imu.end() - 1); it_imu++)
  {
    // 中值积分
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);

    angvel_avr << 0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
        0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
        0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr << 0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
        0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
        0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    angvel_avr -= state_inout.bias_g;
    acc_avr = acc_avr * G_m_s2 / scale_gravity - state_inout.bias_a;

#ifdef DEBUG_PRINT
    std::cout << head->header.stamp.toSec() << " " << angvel_avr.transpose() << " " << acc_avr.transpose() << std::endl;
#endif
    dt = tail->header.stamp.toSec() - head->header.stamp.toSec();

    /* covariance propagation */
    Eigen::Matrix3d acc_avr_skew;
    Eigen::Matrix3d Exp_f = Exp(angvel_avr, dt);
    acc_avr_skew << SKEW_SYM_MATRX(angvel_avr);

    F_x.block<3, 3>(0, 0) = Exp(angvel_avr, -dt);
    F_x.block<3, 3>(0, 9) = -Eye3d * dt;
    F_x.block<3, 3>(3, 6) = Eye3d * dt;
    F_x.block<3, 3>(6, 0) = -R_imu * acc_avr_skew * dt;
    F_x.block<3, 3>(6, 12) = -R_imu * dt;
    F_x.block<3, 3>(6, 15) = Eye3d * dt;

    Eigen::Matrix3d cov_acc_diag(Eye3d), cov_gyr_diag(Eye3d);
    cov_acc_diag.diagonal() = cov_acc;
    cov_gyr_diag.diagonal() = cov_gyr;
    cov_w.block<3, 3>(0, 0).diagonal() = cov_gyr * dt * dt * 10000;
    cov_w.block<3, 3>(3, 3) = R_imu * cov_gyr_diag * R_imu.transpose() * dt * dt * 10000;
    cov_w.block<3, 3>(6, 6) = R_imu * cov_acc_diag * R_imu.transpose() * dt * dt * 10000;
    cov_w.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d(0.0001, 0.0001, 0.0001) * dt * dt;   // bias gyro covariance
    cov_w.block<3, 3>(12, 12).diagonal() = Eigen::Vector3d(0.0001, 0.0001, 0.0001) * dt * dt; // bias acc covariance

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

    /* propogation of IMU attitude */
    R_imu = R_imu * Exp_f;

    /* Specific acceleration (global frame) of IMU */
    acc_imu = R_imu * acc_avr + state_inout.gravity;

    /* propogation of IMU */
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

    /* velocity of IMU */
    vel_imu = vel_imu + acc_imu * dt;

    /* save the poses at each IMU measurements */
    angvel_last = angvel_avr;
    acc_s_last = acc_imu;
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
  }

  // PCL的最后一个点的时间和IMU的最后一个时间往往不是精准对齐的，这里根据最后一个IMU的姿态，计算了最后一个点云的姿态，这个是我们其他所有其他时刻的点云要对准的坐标系

  /*** calculated the pos and attitude prediction at the frame-end ***/
  dt = pcl_end_time - imu_end_time;
  state_inout.vel_end = vel_imu + acc_imu * dt;
  state_inout.rot_end = R_imu * Exp(angvel_avr, dt);
  state_inout.pos_end = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

  /*
    点云补偿
    基本过程：把点云的POSE倒序处理，每两个为一对，在队列前面叫head，下一个叫tail，对于点云数据，也是倒序处理，找大于head时间戳的点来处理，点云处理的位置用一个变量来维护。
    详细过程：首先将PCL的指针it_pcl指向最后一个点云的位置，取出imu pose中最后面的两个pose，时间小的叫head，时间大的叫tail，
    以it_pcl为起点，倒序处理每个点，对于某个点，假设对应的时刻为i，判断i是否大于head的时间，如果大于，则以这个head对应的姿态为起点、pcl点与head的时间差为dt，计算head到i时刻的姿态。
    然后对比最后一个点云（结束时刻）的姿态，计算出i时刻的这个点，在结束时刻的lidar坐标系下的坐标。
    P_at_lidar_i=Pi; // i时刻lidar坐标系下的点

    P_at_imu_i=TblP_at_lidar_i; // i时刻lidar点在imu坐标系的坐标
    P_at_world_i=TwiP_at_imu_i; // 转换到world坐标系
    P_at_imu_e=Twe.inverse()*P_at_world_i; // 转换到结束帧时刻的imu坐标
    P_at_lidar_e=Tbl.inverse()*P_at_imu_e; // 有imu再转到lidar坐标
  */

  auto pos_liD_e = state_inout.pos_end + state_inout.rot_end * Lidar_offset_to_IMU;
  // pos_liD_e的计算假设是imu与lidar的坐标系之间没有旋转变换，只有平移量，这个是livox集成了bmi088，它们的坐标系都是前左上，只有平移
  // 如果是用自己的装配，需要根据条件调整，本质上就是Pw=Tib*Tbl*Pl
  // Pl是雷达坐标系的原点，在雷达坐标系就是(0,0,0)
  // Tib表示imu body坐标系到全局坐标系的转换，这里就是计算出来的rot_end,pos_end
  // Tbl表示lidar坐标系到imu body坐标系的转换，这里旋转为单位阵，平移就是Lidar_offset_to_IMU，即lidar原点在imu坐标系下的表示

#ifdef DEBUG_PRINT
  std::cout << "[ IMU Process ]: vel " << state_inout.vel_end.transpose() << " pos " << state_inout.pos_end.transpose() << " ba" << state_inout.bias_a.transpose() << " bg " << state_inout.bias_g.transpose() << std::endl;
  std::cout << "propagated cov: " << state_inout.cov.diagonal().transpose() << std::endl;
#endif

  /*** undistort each lidar point (backward propagation) ***/
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu << MAT_FROM_ARRAY(head->rot);
    acc_imu << VEC_FROM_ARRAY(head->acc);
    vel_imu << VEC_FROM_ARRAY(head->vel);
    pos_imu << VEC_FROM_ARRAY(head->pos);
    angvel_avr << VEC_FROM_ARRAY(head->gyr);

    for (; it_pcl->curvature / double(1000) > head->offset_time; it_pcl--)
    {
      dt = it_pcl->curvature / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      Eigen::Matrix3d R_i(R_imu * Exp(angvel_avr, dt));
      Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt + R_i * Lidar_offset_to_IMU - pos_liD_e);
      // T_ei表示i时刻的lidar坐标原点到end时刻的坐标平移量，用的全局坐标系的衡量
      // T_ei=Ti-Te
      // 为什么是Ti-Te，在global坐标系下原点为o，Ti是向量oi,Te是向量oe,
      // Ti-Te就是向量oi-oe，得到的是向量ei,
      // 即以e指向i的向量，以e为原点的向量
      // end时刻已经计算出来了就是pos_liD_e
      // i时刻的坐标计算方式与pos_liD_e一样
      // pos_liD_i=Pos_i+rot_i*Lidar_offset_to_IMU
      // 而Pos_i=Pos_head+velocity_head*dt+0.5*acc_head*dt*dt

      // 本质上是将P_i转换到全局坐标系，再转换到end的坐标系下
      Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      Eigen::Vector3d P_compensate = state_inout.rot_end.transpose() * (R_i * P_i + T_ei);
      
      // save Undistorted points and their rotation
      it_pcl->x = P_compensate(0);
      it_pcl->y = P_compensate(1);
      it_pcl->z = P_compensate(2);

      if (it_pcl == pcl_out.points.begin())
        break;
    }
  }
}

void ImuProcess::Process(const MeasureGroup &meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_)
{
  double t1, t2, t3;
  t1 = omp_get_wtime();

  if (meas.imu.empty())
  {
    std::cout << "no imu data" << std::endl;
    return;
  };

  ROS_ASSERT(meas.lidar != nullptr);

  if (imu_need_init_)
  {
    // The very first lidar frame
    IMU_Initial(meas, stat, init_iter_num);

    imu_need_init_ = true;

    last_imu_ = meas.imu.back();

    if (init_iter_num > MAX_INI_COUNT)
    {
      imu_need_init_ = false;
      ROS_INFO("IMU Initials: Gravity: %.4f %.4f %.4f; state.bias_g: %.4f %.4f %.4f; acc covarience: %.8f %.8f %.8f; gry covarience: %.8f %.8f %.8f",
               stat.gravity[0], stat.gravity[1], stat.gravity[2], stat.bias_g[0], stat.bias_g[1], stat.bias_g[2], cov_acc[0], cov_acc[1], cov_acc[2], cov_gyr[0], cov_gyr[1], cov_gyr[2]);
    }

    return;
  }

  // Undistort points：the first point is assummed as the base frame
  // Compensate lidar points with IMU rotation (with only rotation now)
  UndistortPcl(meas, stat, *cur_pcl_un_);

  t2 = omp_get_wtime();

  // Record last measurements
  last_imu_ = meas.imu.back();

  t3 = omp_get_wtime();

  std::cout << "[ IMU Process ]: Time: " << t3 - t1 << std::endl;
}