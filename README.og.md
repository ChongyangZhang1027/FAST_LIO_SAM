# FAST_LIO_SAM

## Front_end : fastlio2      Back_end : lio_sam

<p align='center'>
    <img src="./FAST_LIO_SAM/pic/cover2.png " alt="drawing" width="200" height ="200"/>
    <img src="./FAST_LIO_SAM/pic/cover4.png" alt="drawing" width="200" height =200/>
    <img src="./FAST_LIO_SAM/pic/cover3.png" alt="drawing" width="200" height =200/>
    <img src="./FAST_LIO_SAM/pic/cover1.png" alt="drawing" width="200" height =200/>
</p>

## Videos : FAST-LIO-SAM [Bilibili_link](https://www.bilibili.com/video/BV12Y4y1g7xN/?vd_source=ed6bf57ee5a8e930b7a857e261dac86d)

## Related worked 

1. [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) tightly coupled LiDAR-IMU, lack of backend and global consistence.

2. [FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM) Kim added SC-PGO, finding loop closure by matching scancontext descriptor. SC-PGO and FAST-LIO2 are loosely coupled.

3. [FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC) Yanliang Wang used radius search for loop detection. This work also updated the frondend according to optimization result of backend, and rebuilt the ikdtree. It can be regarded as tightly coupled.

## Contributions  

[FAST_LIO_SAM](https://github.com/kahowang/FAST_LIO_SAM) main contribution:

1. put backend and frontend together
2. add mapping saving module
3. use GNSS XYZ position in backend

## Prerequisites

- Ubuntu 18.04 and ROS Melodic
- PCL >= 1.8 (default for Ubuntu 18.04)
- Eigen >= 3.3.4 (default for Ubuntu 18.04)
- GTSAM >= 4.0.0(tested on 4.0.0-alpha2)

## Build

```shell
cd YOUR_WORKSPACE/src
git clone https://github.com/kahowang/FAST_LIO_SAM.git
cd ..
catkin_make
```

## Quick test

### Loop clousre：

#### 1 .For indoor dataset 

Videos : [FAST-LIO-SAM' videos](https://www.bilibili.com/video/BV12Y4y1g7xN?spm_id_from=444.41.list.card_archive.click&vd_source=ed6bf57ee5a8e930b7a857e261dac86d)

dataset is from yanliang-wang 's [FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC)  ,[dataset](https://drive.google.com/file/d/1NGTN3aULoTMp3raF75LwMu-OUtzUx-zX/view?usp=sharing) which includes `/velodyne_points`(10Hz) and `/imu/data`(400Hz).

```shell
roslaunch fast_lio_sam mapping_velodyne16.launch
rosbag play  T3F2-2021-08-02-15-00-12.bag  
```

<p align ="center">
<img src = "./FAST_LIO_SAM/pic/indoor.gif "  alt ="car" width = 60%  height =60%; "/>
</p>


#### 2 .For outdoor dataset

dataset is from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) **Walking dataset:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]

Videos : [FAST-LIO-SAM' videos](https://www.bilibili.com/video/BV12Y4y1g7xN?spm_id_from=444.41.list.card_archive.click&vd_source=ed6bf57ee5a8e930b7a857e261dac86d)

```shell
roslaunch fast_lio_sam mapping_velodyne16_lio_sam_dataset.launch
rosbag  play  walking_dataset.bag
```

<div align="left">
<img src = "./FAST_LIO_SAM/pic/outdoor_1.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
<img src = "./FAST_LIO_SAM/pic/outdoor_2.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
</div>

#### 3.save_map

```shell
rosservice call /save_map "resolution: 0.0
destination: ''" 
success: True
```

#### 4.save_poes

```shell
rosservice call /save_pose "resolution: 0.0
destination: ''" 
success: False
```

evo 

```shell
evo_traj kitti optimized_pose.txt without_optimized_pose.txt -p
```

| ![evo1](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11evo1.png) | ![evo2](https://kaho-pic-1307106074.cos.ap-guangzhou.myqcloud.com/CSDN_Pictures/%E6%B7%B1%E8%93%9D%E5%A4%9A%E4%BC%A0%E6%84%9F%E5%99%A8%E8%9E%8D%E5%90%88%E5%AE%9A%E4%BD%8D/%E7%AC%AC%E4%BA%8C%E7%AB%A0%E6%BF%80%E5%85%89%E9%87%8C%E7%A8%8B%E8%AE%A11evo2.png) |
| ------------------------------------------------------------ | ------------------------------------------------------------ |

#### 5.some config 

```shell
# Loop closure
loopClosureEnableFlag: true		      # use loopclousre or not 
loopClosureFrequency: 4.0                     # Hz, regulate loop closure constraint add frequency
surroundingKeyframeSize: 50                   # submap size (when loop closure enabled)
historyKeyframeSearchRadius: 1.5             # meters, key frame that is within n meters from current pose will be considerd for loop closure
historyKeyframeSearchTimeDiff: 30.0           # seconds, key frame that is n seconds older will be considered for loop closure
historyKeyframeSearchNum: 20                  # number of hostory key frames will be fused into a submap for loop closure
historyKeyframeFitnessScore: 0.3              # icp threshold, the smaller the better alignment

# visual iktree_map  
visulize_IkdtreeMap: true

# visual iktree_map  
recontructKdTree: true

savePCDDirectory: "/fast_lio_sam_ws/src/FAST_LIO_SAM/PCD/"        # in your home folder, starts and ends with "/". Warning: the code deletes "LOAM" folder then recreates it. See "mapOptimization" for implementation
```



### Use GPS：

#### 1.dataset

dataset is from [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM) **Park dataset:** [[Google Drive](https://drive.google.com/drive/folders/1gJHwfdHCRdjP7vuT556pv8atqrCJPbUq?usp=sharing)]

Videos : [FAST-LIO-SAM' videos](https://www.bilibili.com/video/BV12Y4y1g7xN?spm_id_from=444.41.list.card_archive.click&vd_source=ed6bf57ee5a8e930b7a857e261dac86d)

```shell
roslaunch fast_lio_sam mapping_velodyne16_lio_sam_parking_dataset.launch
rosbag  play  parking_dataset.bag
```

Line Color define:  path_no_optimized(blue)、path_updated(red)、path_gnss(green)

<div align="left">
<img src = "./FAST_LIO_SAM/pic/gps_optimized_path.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
<img src = "./FAST_LIO_SAM/pic/gps_optimized_with_map.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
</div>

#### 2.some config 

```shell
# GPS Settings
useImuHeadingInitialization: false           # if using GPS data, set to "true"
useGpsElevation: false                      # if GPS elevation is bad, set to "false"
gpsCovThreshold: 2.0                        # m^2, threshold for using GPS data
poseCovThreshold: 0 #25.0                      # m^2, threshold for using GPS data  位姿协方差阈值 from isam2
```

#### 3.some fun

when you want to see the path in the Map [satellite map](http://dict.youdao.com/w/satellite map/#keyfrom=E2Ctranslation)，you can also use [Mapviz](http://wiki.ros.org/mapviz)p  plugin . You can refer to  my [blog](https://blog.csdn.net/weixin_41281151/article/details/120630786?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522165569598716782246421813%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=165569598716782246421813&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-2-120630786-null-null.142^v17^pc_search_result_control_group,157^v15^new_3&utm_term=MAPVIZ&spm=1018.2226.3001.4187) on CSDN.

<div align="left">
<img src = "./FAST_LIO_SAM/pic/mapviz_1.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
<img src = "./FAST_LIO_SAM/pic/mapviz_2.gif "  alt ="outdoor"  width=49.6%  height =60%; "/>
</div>



## Attention:

1. FAST-LIO2 use SO3 to represent attitude, but gtsam uses Euler RPY

2. ikdtree reconstruct: yanliang-wang [FAST-LIO-LC](https://github.com/yanliang-wang/FAST_LIO_LC)中的iktree  reconstruct 

3. In walking dataset, pure rotation leads to drifting. Thresholds for selecting keyframes should be modified.

4. Check GNSS Z direction, accuracy are usually very low. Set **"useGpsElevation"** to use GNSS height.


## some problems:

1. GNSS covariace are not transform to world frame.

2. **GeographicLib** transfers coordinate to NED, but EUN is preferred.



## UpdateLogs:

https://github.com/kahowang/FAST_LIO_SAM/blob/master/%E6%9B%B4%E6%96%B0%E6%97%A5%E5%BF%97.md



## Cite the Work 
If you use this repository in your academic research, a BibTeX citation is appreciated: 
```
@misc{wang2022fast_lio_sam,
  title={FAST-LIO-SAM: FAST-LIO with Smoothing and Mapping.},
  author={Wang, Jiahao},
  howpublished={\url{https://github.com/kahowang/FAST_LIO_SAM}},
  year={2022}
}
```
or, you can add a footnote link of this repository: 
`https://github.com/kahowang/FAST_LIO_SAM` 



## Acknowledgements 

​	In this project, the LIO module refers to [FAST-LIO](https://github.com/hku-mars/FAST_LIO) and the pose graph optimization refers to [FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM) and [LIO_SAM](https://github.com/TixiaoShan/LIO-SAM).The mainly idea is for [FAST_LIO_LC](https://github.com/yanliang-wang/FAST_LIO_LC).Thanks there great work .

​	Also thanks yanliang-wang、minzhao-zhu、peili-ma 's  great help .

​																																																																	edited by kaho 2022.6.20
