# FAST_LIO_MeiyuanXiang
FAST_LIO相关论文、代码中文注释以及代码改动  

# 参考
https://github.com/hku-mars/FAST_LIO  
https://github.com/gisbi-kim/FAST_LIO_SLAM  

# 环境
1. Ubuntu（测试了Ubuntu16.04.5、Ubuntu18.04）  
2. ROS (测试了kinetic、melodic)  
3. PCL（测试了pcl1.9）  
4. Eigen（测试了Eigen3.3.4）  
5. livox_ros_driver  

# 编译
1. 下载源码 git clone https://github.com/MeiyuanXiang/FAST_LIO_MeiyuanXiang.git  
2. 将FAST_LIO_MeiyuanXiang\src下的FAST_LIO或FAST_LIO_SLAM拷贝到ros工程空间src文件夹内，例如~/catkin_ws/src/  
3. cd ~/catkin_ws  
4. catkin_make  
5. source ~/catkin_ws/devel/setup.bash  

# Bag数据
链接：https://pan.baidu.com/s/1vaW3FSDyINF4m1MU6O08RA  
提取码：qx9x  
2020-09-16-quick-shack.bag、10hz_2020-07-25-17-36-51.bag和HKU_MB_2020-09-20-13-34-51.bag  

# MulRan数据
https://sites.google.com/view/mulran-pr/download  

# 运行
1. Livox Avia直连  
roslaunch fast_lio mapping_avia.launch  
roslaunch livox_ros_driver livox_lidar_msg.launch  
2. 室内场景数据  
roslaunch fast_lio mapping_avia.launch  
rosbag play 2020-09-16-quick-shack.bag  
或rosbag play 10hz_2020-07-25-17-36-51.bag  
3. 室外场景数据  
roslaunch fast_lio mapping_avia.launch  
rosbag play HKU_MB_2020-09-20-13-34-51.bag  
4. FAST-LIO2对MulRan数据集  
roslaunch fast_lio mapping_ouster64_mulran.launch # setting for MulRan dataset  
5. SC-PGO对MulRan数据集  
roslaunch aloam_velodyne fastlio_ouster64.launch # setting for MulRan dataset  

对于MulRan数据集的播放，打开新的终端，使用file_player_mulran播放MulRan数据(详情请参考：https://github.com/irapkaist/file_player_mulran)  
