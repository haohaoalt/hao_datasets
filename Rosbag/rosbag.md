# 01 对于bag包的处理

- **过滤单个topic**

```
rosbag filter input.bag only-tf.bag "topic == '/tf'"
```

- **过滤多个topic**

```
rosbag filter input.bag output.bag "topic == '/velodyne_point_cloud' or topic =='/visensor/imu' or topic == '/visensor/left/image_raw'"
```



- **从bag包中截取相应的数据**

  ```
  rosbag filter xxx.bag  xxxafter.bag "(topic =='/lslidar_packet' or topic =='/imu/data' or topic =='/nmea_sentence') "
  ```

- **从一个bag 包中截取某段时间内的数据**

  ```
  rosbag 2021-01-23-16-51-27_besidepicture.bag 2021-01-23-16-51-27_5min.bag" “ t.to_sec() <= 1611392188.19"
  ```

- **从bag包中截取相应的话题以及相应的时间段**

```
rosbag filter g00_kitti_2011_10_03_drive_0027_synced.bag 00g.bag "(topic == '/kitti/camera_color_left/image_raw' or topic == '/kitti/camera_color_left/image_raw') and (t.to_sec()>=1317617780 and t.to_sec()<=1317618000)"
```

