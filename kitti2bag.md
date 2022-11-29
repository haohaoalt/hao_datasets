# kitty2bag

感觉在ubuntu20上比较简单 
pip install kitti2bag
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_30_drive_0027/2011_09_30_drive_0027_sync.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_30_calib.zip
$ unzip 2011_09_30_drive_0027_sync.zip
$ unzip 2011_09_30_calib.zip

Transform the RAW data to a ROS bag
kitti2bag -t 2011_09_30 -r 0027 raw_synced
