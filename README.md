# SSC
Code for IROS2021 paper [SSC: Semantic Scan Context for Large-Scale Place Recognition](https://github.com/lilin-hitcrt/SSC/blob/main/paper/ssc.pdf)

![pipeline](./pic/pipeline.png)

## Citation

```
@misc{li2021ssc,
      title={SSC: Semantic Scan Context for Large-Scale Place Recognition}, 
      author={Lin Li and Xin Kong and Xiangrui Zhao and Tianxin Huang, Wanlong Li, Feng Wen, Hongbo Zhang and Yong Liu},
      year={2021},
      booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
      pages={},
      organization={IEEE}
}
```

## Requirements
[OpenCV](https://opencv.org/)  
[PCL](https://pointclouds.org/)  
[yaml-cpp](https://github.com/jbeder/yaml-cpp) 

## Usage
Build the code:
```bash
    mkdir build && cd build && cmake .. && make -j5
```
Modify the [configuration file](https://github.com/lilin-hitcrt/SSC/blob/main/config/config_kitti.yaml).

A simple example:
```bash
    cd ../bin
    ./eval_pair
```
Precision-Recall:

```bash
    cd ../bin
    ./eval_seq
```
Top-k Recall:

```bash
    cd ../bin
    ./eval_top1
```
[plot curve](./script/README.md)

## Data
### Pair Lists
[Lists of evaluation samples](https://drive.google.com/file/d/1Y540LJFZHiaAooUX2KtxNIQhw-kzy7gQ/view?usp=sharing). We select all positive samples, and randomly select some negative samples according to a fixed ratio for evaluation. This folder contains lists of evaluation samples we used for the KITTI dataset (pairs_kitti) and KITTI-360 dataset (pairs_kitti360). The naming rule of the parent folder of the lists is "neg_ratio". For example, "neg_10" means that the number of negative samples is ten times that of positive samples.

### Semantic label for KITTI-360
We use the kd-tree to obtain the [semantic information](https://drive.google.com/file/d/1QvPw--pfikvWrWNP_tWfxxCawUf7IdEb/view?usp=sharing) of each scan in the KITTI-360 data set.

## Results
### KITTI
#### Top-k Recall
When using 5 m as the threshold, the top-k recall rate is shown in the figure:

![recall](./pic/recall.png)

#### Precision-Recall Curve
The Precision-Recall curve when α=100:

![pr](./pic/pr.png)

### KITTI-360
#### Precision-Recall Curve
The Precision-Recall curve when α=10:

![pr](./pic/pr_kitti360.png)

## Acknowledgement

Thanks to the source code of some great works such as [Scan Context](https://github.com/irapkaist/scancontext) and [Intensity Scan Context](https://github.com/wh200720041/iscloam)