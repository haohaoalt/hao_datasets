<!--
 * @Author: https://github.com/haohaoalt
 * @Date: 2023-12-06 12:08:36
 * @LastEditors: hayden haohaoalt@163.com
 * @LastEditTime: 2023-12-06 13:02:45
 * @FilePath: /hao_datasets/kitti/kitti.md
 * @Description: 
 * Copyright (c) 2023 by haohaoalt@163.com, All Rights Reserved. 
-->

## KITTI2TUM

```
python kitti_poses_and_timestamps_to_trajectory.py 00.txt times.txt traj_output.txt
evo_traj tum traj_output.txt -p --plot_mode=xz
```
![1701835730072](image/kitti/1701835730072.png)

## pykitti

KITTI数据集比较麻烦，官网上并没有给出一个比较便捷的评测工具（尤其是对于ORBSLAM2输出格式），官网提供的一个评测工具是pykitti，详细的过程参考：https://zhuanlan.zhihu.com/p/76155544