import numpy as np
from matplotlib import pyplot as plt
import sys
import os
# def run(seq='00'):
#     pose_file="/media/l/yp2/KITTI/odometry/dataset/poses/"+seq+".txt"
#     poses=np.genfromtxt(pose_file)
#     poses=poses[:,[3,11]]
#     inner=2*np.matmul(poses,poses.T)
#     xx=np.sum(poses**2,1,keepdims=True)
#     dis=xx-inner+xx.T
#     dis=np.sqrt(np.abs(dis))
#     id_pos=np.argwhere(dis<3)
#     id_neg=np.argwhere(dis>20)
#     id_pos=id_pos[id_pos[:,0]-id_pos[:,1]>50]
#     id_neg=id_neg[id_neg[:,0]>id_neg[:,1]]
#     print(len(id_pos))
#     np.savez(seq+'.npz',pos=id_pos,neg=id_neg)

def run(seq='00'):
    pose_file="/media/l/yp2/KITTI/odometry/dataset/poses/"+seq+".txt"
    poses=np.genfromtxt(pose_file)
    poses=poses[:,[3,11]]
    inner=2*np.matmul(poses,poses.T)
    xx=np.sum(poses**2,1,keepdims=True)
    dis=xx-inner+xx.T
    dis=np.sqrt(np.abs(dis))
    id_pos=np.argwhere(dis<5)
    id_pos=id_pos[id_pos[:,0]-id_pos[:,1]>50]
    id_pos=list(set(id_pos[:,0]))
    id_pos.sort()
    print(len(id_pos))
    # np.savetxt(seq+'.txt',np.array(id_pos,dtype='int32'),fmt='%d')

if __name__=='__main__':
    seq="05"
    if len(sys.argv)>1:
        seq=sys.argv[1]
    run(seq)
