# RoboMaster2021冬、夏令营文档

## 功能框架

RoboMaster S1利用eduSDK实现导航抓取等功能，由以下package构成：

**rms1_bringup** 

发布图片以及相机信息；发布静态tf变换（地图与base2cam）；地图加载；

接收速度位置指令，控制S1底盘以及机械爪运动，实现规则主要逻辑

**odom_calib** 

接收tag识别结果，更新map与odom的tf变换map2odom_transform；

发布货物相对底盘位姿base2item_pose

**simple_planner** 

订阅rviz相关话题，进行路径规划与速度发布

**apriltags2_ros** 

处理图片，发布tag信息；设置tag尺寸、ID

**apriltags2**  

tag编码，录入新tag

**joystick_drivers**  

joy手柄在noetic中的驱动(需xbox360手柄切换模式：扫描物块，抓取物块等)

## 测试

### 前期准备

* 相机标定 
  * 修改camera_info/head_camera.yaml
* 修改标定文件路径 
  * rms1_bringup/scripts/bringup.py第83行  
* 静态tf发布（相机与底盘&tag与地图）
  * 修改rms1_bringup/launch/rms1_bringup.launch第58行  
* 增加tag （以36h12码为例）
  * 修改apriltags2/src/tag36h12.c
* 更改tag种类 
  * 修改apriltags2_ros/config/setting.yaml
* 更改tag id&size 
  * 修改apriltags2_ros/config/tag.yaml  

### 启动测试

~~~ shell
roslaunch rms1_bringup rms1_bringup.launch
~~~

## 问题记录

* 物块自动对位抓取的成功率不高，测试发现速度控制的最小单位为 0.1m/s，位置控制的最小单位为0.02m，直接解算麦轮的最小单位为0.07m/s，提高成功率可以精调pid参数或者提高S1底盘控制精度
* RoboMaster S1视频解码，正常延时在220ms左右，偶尔会出现4000ms左右的超高延时，避免超高延时，发布图像的循环内不能有阻塞命令
* tag贴纸的四角的黑色格子会影响识别效果
* 图像分辨率为1280x720，发布频率为10hz，目前降采样为640x360，达到30hz以上（不生效，已移除相关代码）
* --------------------------------------------------------------------------------
* 摄像头标定算法讲解、配套功能包安装等
* TF静态变换所涉及的坐标系说明（摄像头base_cam、机器人base_link）
* 已适配Ubuntu 20.04 Noetic版本的ROS系统
* 翻车王老师 备 2021.07.21




