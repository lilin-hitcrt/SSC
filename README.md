# SSC: Semantic Scan Context for Large-Scale Place Recognition
pass

![pipeline](./pic/pipeline.png)

## Citation
pass

## Requirements
[OpenCV](https://opencv.org/)  
[PCL](https://pointclouds.org/)  
[yaml-cpp](https://github.com/jbeder/yaml-cpp) 

## Usage
Build the code:
```bash
    mkdir build && cd build && cmake .. && make -j5
```
Modify the [configuration file](config/config.yaml).

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
[pairs](https://drive.google.com/file/d/1Y540LJFZHiaAooUX2KtxNIQhw-kzy7gQ/view?usp=sharing)

### Semantic label for KITTI-360
[label](https://drive.google.com/file/d/1QvPw--pfikvWrWNP_tWfxxCawUf7IdEb/view?usp=sharing)

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