## dataset_converter
可以将ROS bag中的RGBD图像数据转换成TUM数据集的格式，并自动进行深度图和彩色图的匹配生成associate.txt文件
### 使用方法
#### 0 编译及环境
测试环境Ubuntu 20.04 + ROS noetic 
```bash
cp datasert_converter YOUR_WORKSPACE_PATH/src
cd YOUR_WORKSPACE_PATH
catkin_make
```

#### 1 修改rosbagToTUM.launch
指定输出目录，rosbag名称，彩色图话题名称，深度图话题名称
```
<arg name="outputpath" default="PATH_TO_OUTPUT_DIR" />
<param name="bag_name" type="string" value="BAG_NAME" />
<param name="topic_name_rgb" type="string" value="TOPIC_NAME_OF_RGB" />
<param name="topic_name_depth" type="string" value="TOPIC_NAME_OF_DEPTH" />
```
#### 2 终端运行
```bash
roslaunch dataset_converter rosbagToTUM.launch 
```

#### 最终可以获得如下的结构
```
PATH_TO_OUTPUT_DIR
└── dataset
    ├── associate.txt
    ├── depth
    ├── depth.txt
    ├── rgb
    └── rgb.txt
```
