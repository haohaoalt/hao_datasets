# kitty2bag

感觉在ubuntu20上比较简单 
pip install kitti2bag
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_30_drive_0027/2011_09_30_drive_0027_sync.zip
$ wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_30_calib.zip
$ unzip 2011_09_30_drive_0027_sync.zip
$ unzip 2011_09_30_calib.zip

Transform the RAW data to a ROS bag
kitti2bag -t 2011_09_30 -r 0027 raw_synced
```
//创建conda python2.7环境
conda activate py27
python -V
pip install kitti2bag
pip install pyyaml
python -m pip install opencv-python==4.2.0.32
pip install pycryptodomex
pip install rospkg
pip install pycryptodomex
pip install gnupg

pip freeze > requirements.txt
pip install -r requirements.txt
```