# SSC
Code for IROS2021 paper [SSC: Semantic Scan Context for Large-Scale Place Recognition](https://ieeexplore.ieee.org/document/9635904)

![pipeline](./pic/pipeline.png)

## Citation

```
@INPROCEEDINGS{9635904,
  author={Li, Lin and Kong, Xin and Zhao, Xiangrui and Huang, Tianxin and Li, Wanlong and Wen, Feng and Zhang, Hongbo and Liu, Yong},
  booktitle={2021 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={{SSC}: Semantic Scan Context for Large-Scale Place Recognition}, 
  year={2021},
  volume={},
  number={},
  pages={2092-2099},
  doi={10.1109/IROS51168.2021.9635904}}
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

### Raw Data
We provide the [raw data](https://drive.google.com/file/d/1mq09Vkolfo99akq-EvvdA68ei5S0J0fY/view?usp=sharing) of the tables and curves in the paper. For the fairness of the experiment, make sure that your results are also based on the same [evaluation samples](https://drive.google.com/file/d/1Y540LJFZHiaAooUX2KtxNIQhw-kzy7gQ/view?usp=sharing) when citing the provided data. 

We provide the results of all the methods mentioned in the paper on the KITTI and KITTI-360 datasets. The first column of each data file is the similarity score, and the second column is the ground truth. We also provide a python script to draw curves.

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