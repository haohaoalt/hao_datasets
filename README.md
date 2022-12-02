<!--
 * @Author: zhanghao
 * @Date: 2022-11-29 14:47:13
 * @LastEditTime: 2022-11-29 15:00:09
 * @FilePath: /hao_datasets/README.md
 * @Description: 
-->
# hao_datasets
自己用的SLAM数据集整理，KITTI TUM EUROC ROSBAG
## 01 KITTI

<img src="README.assets/image-20221129150136624.png" alt="image-20221129150136624" style="zoom:50%;" />

## 02 tum

https://vision.in.tum.de/data/datasets/rgbd-dataset/download

<img src="README.assets/image-20221129150154091.png" alt="image-20221129150154091" style="zoom:50%;" />

### TUM RGBD数据集工具及使用

#### 1. 工具

工具下载地址：https://vision.in.tum.de/data/datasets/rgbd-dataset/tools
• `add_pointclouds_to_bagfile.py` 给bag中加入点云topic
• `associate.py` 生成depth 和 rgb的匹配信息
• `evaluate_ate.py` 绝对误差评估，常用
• `evaluate_rpe.py` 相对误差评估
• `generate_pointcloud.py` 生成点云数据
• `generate_registered_pointcloud.py`
• `plot_camera_trajectories.m` matlab打印轨迹
• `plot_trajectory_into_image.py`
• `prepare3dedges.py`
• `project_point_cloud_to_image.py`

#### 2. 数据集

下载地址：https://vision.in.tum.de/data/datasets/rgbd-dataset/download
数据集包含`bag`及`tgz`两种格式
1）bag包：图像以15hz的频率发布，imu以500hz频率发布，较多卡顿现象，发布信息：

> /camera/depth/camera_info
> /camera/depth/image
> /camera/rgb/camera_info
> /camera/rgb/image_color
> /clock
> /cortex_marker_array
> /imu
> /rosout
> /rosout_agg
> /tf

使用方法：`rosbag play <bag_name>.bag`

2）tgz包：包含rgb及depth文件夹分别存放rgb图像及深度图，`accelerometer.txt`存放加速度信息，相对提供的bag包数据流畅。

使用方法：
1、解析文件夹，例程如：ORB_SLAM3/Examples/RGB-D/rgbd_tum.cc
2、使用如下脚本将tgz包解析转化为bag包，此时生成的bag包比官网提供的bag流畅，频率为30hz。

`generate_bags.py`：

```python
import cv2 
import time, sys, os
from ros import rosbag
import roslib
import rospy
roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import Image,Imu
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
from numpy import asarray

# import ImageFile
from PIL import ImageFile
from PIL import Image as ImagePIL

def CompSortFileNamesNr(f):
    g = os.path.splitext(os.path.split(f)[1])[0] #get the file of the
    numbertext = ''.join(c for c in g if c.isdigit())
    return int(numbertext)

def ReadIMU(IMUFile):
    '''return IMU data and timestamp of IMU'''
    IMUfp = open(IMUFile,'r')
    IMULines = IMUfp.readlines()
    #all = IMUDatas.readlines()
    IMUDatas = {}
    for l in IMULines:
        if l[0] == "#":
            continue;
        items = l.rstrip('\n').split(' ')
        IMUDatas[items[0]] = items[1:]
    
    IMUfp.close()
    return IMUDatas 

def ReadImages(assocoations):
   assofp = open(assocoations, 'r')
   asso = assofp.readlines()
   RGBImages = {}
   depthImages = {}
   for l in asso:
       if l[0] == "#":
           continue;
       items = l.rstrip('\n').split(' ')
       RGBImages[items[0]] = items[1]
       depthImages[items[2]] = items[3]

   assofp.close()
   return RGBImages, depthImages

def CreateBag(args):#assocoations, imu, output_bag_name
    '''read assocoations.txt'''
    RGBImages,depthImages = ReadImages(args[1])

    IMUDatas = ReadIMU(args[2]) #the url of IMU data

    '''Creates a bag file with camera images'''
    if not os.path.exists(args[3]):
       os.system(r'touch %s' % args[3])
    else:
       os.system(r'rm %s' % args[3])
       os.system(r'touch %s' % args[3])

    bagName = rosbag.Bag(args[3], 'w')

    try:
        for it, iData in IMUDatas.items():
            imu = Imu()
            imuStamp = rospy.rostime.Time.from_sec(float(it))
            #angular_v = Vector3()
            linear_a = Vector3()
            #angular_v.x = float(iData[0])
            #angular_v.y = float(iData[1])
            #angular_v.z = float(iData[2])
            linear_a.x = float(iData[0])
            linear_a.y = float(iData[1])
            linear_a.z = float(iData[2])
            imu.header.stamp = imuStamp
            #imu.angular_velocity = angular_v
            imu.linear_acceleration = linear_a

            bagName.write("/imu",imu,imuStamp)

        br = CvBridge()

        for imt, img in RGBImages.items():
            #img = args[2] + img; 
            print("Adding %s" % img)

            cv_image = cv2.imread(img)

            Stamp = rospy.rostime.Time.from_sec(float(imt))

            '''set image information '''
            Img = br.cv2_to_imgmsg(cv_image)
            Img.header.stamp = Stamp
            Img.header.frame_id = "camera"

            '''for mono8'''
            Img.encoding = "rgb8"
            bagName.write('/camera/rgb/image_color', Img, Stamp)

        for dt, dimg in depthImages.items():
            #dimg = args[2] + dimg; 
            print("Adding %s" % dimg)

            cv_image = cv2.imread(dimg, cv2.IMREAD_ANYDEPTH)

            '''set image information '''
            Stamp = rospy.rostime.Time.from_sec(float(dt))

            '''set image information '''
            dImg = br.cv2_to_imgmsg(cv_image)
            dImg.header.stamp = Stamp
            dImg.header.frame_id = "camera"

            #dImg.encoding = "32FC1"

            bagName.write('/camera/depth/image', dImg, Stamp)

    finally:
        bagName.close()

if __name__ == "__main__":
    print(sys.argv)

    if len(sys.argv) < 4:
        print("Usage:\n\t python generate_bags.py /path/assocoations.txt /path/accelerometer.txt output.bag")
        exit(1)

    CreateBag(sys.argv)

```

**使用方法**：
首先：使用官网提供的脚本`associate.py`生成`assocoations.txt`文件

```bash
python  rgbd_benchmark_tools/scripts/associate.py /path/rgb.txt /path/depth.txt > assocoations.txt
1
```

然后使用上述`generate_bags.py`生成bag包

```bash
python rgbd_benchmark_tools/scripts/generate_bags.py /path/assocoations.txt /path/accelerometer.txt output.bag
```







## 03 EuRoC

<img src="README.assets/image-20221129152408668.png" alt="image-20221129152408668" style="zoom: 50%;" />

Euroc提供ROS和zip两种数据格式

下载地址：https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets#downloads

<img src="README.assets/image-20221129151045027.png" alt="image-20221129151045027" style="zoom:67%;" />

ROSBAG格式：

<img src="README.assets/image-20221129151452891.png" alt="image-20221129151452891" style="zoom:50%;" />

ZIP格式：

```shell
├── body.yaml						// 没啥用
├── cam0							// 左目图像数据
│   ├── data	 					// *.png文件（文件命名即为时间戳）
│   ├── data.csv					// 两列，第一列为时间戳，第二列为对应图片名
│   └── sensor.yaml					// 相机内外参数文件
├── cam1							// 右目图像数据
│   ├── data
│   ├── data.csv
│   └── sensor.yaml
├── imu0
│   ├── data.csv					// 7列，[时间戳，gyr_x,gyr_y,gyr_z, acc_x,acc_y,acc_z]
│   └── sensor.yaml					// imu标定数据
├── leica0							// Leica MS50 用于测量ground_truth
│   ├── data.csv					// 4列 时间戳+位姿
│   └── sensor.yaml					// 外参
└── state_groundtruth_estimate0
    ├── data.csv					// 17列 [时间戳，pose,四元数，vectory，b_w,b_a]
    └── sensor.yaml
```

读取Euroc图片数据代码

```cpp
/**
 * @brief 读取Euroc图片数据
 *  */
size_t load_image_data(const string &image_folder,
                       std::vector<string> &limg_name,
                       std::vector<string> &rimg_name) {
    LOG(INFO) << "Loading " << image_folder;
    std::string l_path = image_folder + "/mav0/cam0/data.csv";
    std::string r_path = image_folder + "/mav0/cam1/data.csv";
    std::string r_img_prefix = image_folder + "/mav0/cam1/data/";
    std::ifstream limg_file(l_path);
    std::ifstream rimg_file(r_path);
    if (!limg_file.is_open() || !rimg_file.is_open()) {
        LOG(WARNING) << image_folder << " cannot be opened";
        return 0;
    }
    std::string line;
    std::string time;
    while (getline(limg_file, line)) {
        if (line[0] == '#')
            continue;
        std::istringstream is(line);
        int i = 0;
        while (getline(is, time, ',')) {
            bool is_exist = boost::filesystem::exists(r_img_prefix + time + ".png");
            if (i == 0 && is_exist) {
                limg_name.push_back(time + ".png");
                rimg_name.push_back(time + ".png");
            }
            i++;
        }
    }
    limg_file.close();
    rimg_file.close();
    LOG(INFO) << "loaded " << limg_name.size() << " images";
    return limg_name.size();
}
12345678910111213141516171819202122232425262728293031323334353637
```

读取图像时间戳的代码

```cpp
/**
 * @brief 计算 图像帧 与 offset_ns 之间的时间差
 * @param img_name Euroc 图像文件名
 * @param offset_ns 参考时间戳
 * */
float get_timestamp_from_img_name(const string &img_name,
                                  uint64_t offset_ns) {
    // 注：Euroc中，图片命名即为其时间戳
    string ts_ns_string = fs::path(img_name).stem().string();
    int64_t offset_t = boost::lexical_cast<uint64_t>(ts_ns_string) - offset_ns;
    int64_t t = offset_t / 1e5;     // 时间戳最后五位应当一致，或者说影响不大，认为是误差
    return static_cast<float>(t) / 1e4;
}
12345678910111213
```

读取Euroc imu数据代码

```cpp
/**
 * @brief 读取Euroc imu数据
 *  */
size_t load_imu_data(const string &imu_file_str,
                     std::list<XP::ImuData> *imu_samples_ptr,
                     uint64_t &offset_ts_ns) {
    CHECK(imu_samples_ptr != NULL);
    LOG(INFO) << "Loading " << imu_file_str;
    std::ifstream imu_file(imu_file_str.c_str());
    if (!imu_file.is_open()) {
        LOG(WARNING) << imu_file_str << " cannot be opened";
        return 0;
    }
    std::list<XP::ImuData> &imu_samples = *imu_samples_ptr;
    imu_samples.clear();
    // read imu data
    std::string line;
    std::string item;
    double c[6];
    uint64_t t;
    bool set_offset_time = false;
    while (getline(imu_file, line)) {
        if (line[0] == '#')
            continue;
        std::istringstream is(line);
        int i = 0;
        while (getline(is, item, ',')) {
            std::stringstream ss;
            ss << item;
            if (i == 0)
                ss >> t;
            else
                ss >> c[i - 1];
            i++;
        }
        if (!set_offset_time) {
            set_offset_time = true;
            offset_ts_ns = t;
        }
        XP::ImuData imu_sample;
        float _t_100us = (t - offset_ts_ns) / 1e5;
        imu_sample.time_stamp = _t_100us / 1e4;		// 以第一组IMU数据的时间戳为时间原点0，单位s
        imu_sample.ang_v(0) = c[0];
        imu_sample.ang_v(1) = c[1];
        imu_sample.ang_v(2) = c[2];
        imu_sample.accel(0) = c[3];
        imu_sample.accel(1) = c[4];
        imu_sample.accel(2) = c[5];

        VLOG(3) << "accel " << imu_sample.accel.transpose()
                << " gyro " << imu_sample.ang_v.transpose();
        imu_samples.push_back(imu_sample);
    }
    imu_file.close();
    LOG(INFO) << "loaded " << imu_samples.size() << " imu samples";
    return imu_samples.size();
}
```

## 04 lidar_SLAM

![image-20221129150109843](README.assets/image-20221129150109843.png)
