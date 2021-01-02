import  numpy as np
from sklearn import  metrics
from matplotlib import pyplot as plt
import os
def eval(file,out_path):
    with open(os.path.join(os.path.abspath(out_path),file+".txt")) as f:
        score=[]
        truth=[]
        while True:
            line=f.readline().strip().split()
            if not line:
                break
            line=[float(v) for v in line]
            score.append(line[0])
            truth.append(line[1])
        score=np.array(score)
        truth=np.array(truth)
        np.save(os.path.join(out_path,'iris_score_'+file+'.npy'),score)
        np.save(os.path.join(out_path,'iris_truth_'+file+'.npy'),truth)
        precision, recall, pr_thresholds = metrics.precision_recall_curve(truth, score)
        lw = 2
        # plt.figure(figsize=(10, 10))
        plt.plot(recall, precision, color='darkorange',
                 lw=lw, label='P-R curve')
        plt.axis([0,1,0,1])
        plt.xlabel('Recall')
        plt.ylabel('Precision')
        plt.title('DL Precision-Recall Curve')
        plt.legend(loc="lower right")
        plt.savefig(os.path.join(out_path,'iris_pr_curve_'+file+'.png'))
        F1_score = 2 * precision * recall / (precision + recall)
        F1_score = np.nan_to_num(F1_score)
        # print(F1_score)
        F1_max_score = np.max(F1_score)
        print(F1_max_score)
        with open(os.path.join(out_path,'iris_F1_max_'+file+'.txt'), "w") as out:
            out.write(str(F1_max_score))
        plt.show()


if __name__=="__main__":
    # sequ=['00','02','05','06','07','08']
    # for v in sequ:
    #     eval(v,'/home/l/workspace/python/pr_curve/SSC_sk_10')
    import sys
    seq='00'
    if len(sys.argv)>1:
        seq=sys.argv[1]
    eval(seq,'./')
