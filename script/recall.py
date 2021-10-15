import numpy as np
from matplotlib import pyplot as plt
import sys
import os
def run(seq='00',folder="/media/l/yp2/KITTI/odometry/dataset/poses/"):
    pose_file=os.path.join(folder,seq+".txt")
    poses=np.genfromtxt(pose_file)
    poses=poses[:,[3,11]]
    inner=2*np.matmul(poses,poses.T)
    xx=np.sum(poses**2,1,keepdims=True)
    dis=xx-inner+xx.T
    dis=np.sqrt(np.abs(dis))
    id_pos=np.argwhere(dis<=5)
    id_pos=id_pos[id_pos[:,0]-id_pos[:,1]>50]
    pos_dict={}
    for v in id_pos:
        if v[0] in pos_dict:
            pos_dict[v[0]].append(v[1])
        else:
            pos_dict[v[0]]=[v[1]]
    rt_file=np.genfromtxt(seq+'.txt')
    recall=np.array([0.]*(rt_file.shape[1]-1))
    for v in rt_file:
        assert v[0] in pos_dict
        truth=pos_dict[v[0]]
        for i in range(1,len(v)):
            if v[i] in truth:
                recall[i-1:]+=1
                break
    recall/=len(rt_file)
    print(recall)
    plt.plot(list(range(1,len(recall)+1)),recall,marker='o')
    plt.axis([1,25,0,1])
    plt.title(seq)
    plt.xlabel('N top retrievals')
    plt.ylabel('Recall (%)')
    # plt.savefig(seq+'_recall.png')
    plt.show()
                

if __name__=='__main__':
    seq="00"
    if len(sys.argv)>1:
        seq=sys.argv[1]
    run(seq)
