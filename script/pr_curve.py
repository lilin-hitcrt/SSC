import  numpy as np
from sklearn import  metrics
from matplotlib import pyplot as plt
import os
def eval(file,out_path):
    db=np.genfromtxt(os.path.join(out_path,file+".txt"))
    db=np.nan_to_num(db)
    precision, recall, pr_thresholds = metrics.precision_recall_curve(db[:,1], db[:,0])
    plt.plot(recall, precision, color='darkorange',lw=2, label='P-R curve')
    plt.axis([0,1,0,1])
    plt.xlabel('Recall')
    plt.ylabel('Precision')
    plt.title('Precision-Recall Curve')
    plt.savefig(os.path.join(out_path,'pr_curve_'+file+'.png'))
    F1_score = 2 * precision * recall / (precision + recall)
    F1_score = np.nan_to_num(F1_score)
    # print(F1_score)
    F1_max_score = np.max(F1_score)
    print("F1:",F1_max_score)
    plt.show()
    plt.close()


if __name__=="__main__":
    #sequ=['0000','0002','0005','0006','0004','0009']
    sequ=['00','02','05','06','07','08']
    for v in sequ:
       eval(v,'/media/l/yp1/descs/pr_curve/pr_curve/kitti/neg_100/SSC_sk')
