https://www.guyuehome.com/35920


在ROS系统中，可以使用bag文件来保存和恢复系统的运行状态，比如录制雷达和相机话题的bag包，然后通过回放用来SLAM定位或者进行联合外参标定。但是在一些情况下bag包并不是那么方便，例如：

* 所运行的程序不能时时运行，需要每张图片按照顺序进行处理。此时虽然rosbag可以设置回放速度以及队列缓存，但是有时仍然不能很好的满足顺序处理的要求。
* 有些平台没有安装ROS，不能接收图像的topic数据，即不能很好地将程序进行移植。

  此时就需要我们将bag数据转存为TUM通用数据集的格式，然后通过读取TUM数据的方式读取我们的数据。具体的，本博文实现的功能为：提取并保存bag包中的RGB图像和depth图像，使其可以以单目或者RGB-D的数据集的形式供使用。

TUM是一个RGB-D的数据集，主页为[https://vision.in.tum.de/data/datasets/rgbd-dataset/download](https://vision.in.tum.de/data/datasets/rgbd-dataset/download) 。每一个数据集包含以下文件：depth ,rgb ,depth.txt ,rgb.txt ,accelerometer.txt ,groundtruth.txt。要运行此数据集，只需要恢复前四个文件即可。

其中rgb.txt和depth.txt记录的每张图片的采集时间和图片名称（图片名称也是以时间来命名的）。

上一层文件夹中的内容就是使用C++版本的bag2tum生成的

