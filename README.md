# SSC
Semantic Scan Context
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
```bash
    cd ../bin
    ./eval_pair or ./eval_seq
```
## pairs list
[pairs](https://drive.google.com/file/d/1Y540LJFZHiaAooUX2KtxNIQhw-kzy7gQ/view?usp=sharing)

## results

### Top-k Recall
<!-- <img src="./script/00_recall.png" alt="00" style="zoom:50%" /> <img src="./script/02_recall.png" alt="02" style="zoom:50%" /> <img src="./script/06_recall.png" alt="05" style="zoom:50%" />

<img src="./script/06_recall.png" alt="06" style="zoom:50%" /> <img src="./script/07_recall.png" alt="07" style="zoom:50%" /> <img src="./script/08_recall.png" alt="08" style="zoom:50%" /> -->
![00](./script/00_recall.png) ![02](./script/02_recall.png) ![05](./script/05_recall.png) ![06](./script/06_recall.png) ![07](./script/07_recall.png) ![08](./script/08_recall.png)

### Precision-Recall Curve
<!-- <img src="./script/00_pr.png" alt="00" style="zoom:50%" /> <img src="./script/02_pr.png" alt="02" style="zoom:50%" /> <img src="./script/06_pr.png" alt="05" style="zoom:50%" />

<img src="./script/06_pr.png" alt="06" style="zoom:50%" /> <img src="./script/07_pr.png" alt="07" style="zoom:50%" /> <img src="./script/08_pr.png" alt="08" style="zoom:50%" /> -->
![00](./script/00_pr.png) ![02](./script/02_pr.png) ![05](./script/05_pr.png) ![06](./script/06_pr.png) ![07](./script/07_pr.png) ![08](./script/08_pr.png)