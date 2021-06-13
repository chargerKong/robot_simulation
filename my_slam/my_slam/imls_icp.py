import numpy as np
from sklearn.neighbors import KDTree
from 
class Imls_icp:
    
    def __init__(self):
        self.pre_pointcloud = None
        self.now_pointcloud = None
        self.kdt = None

    def set_source_pointcloud(self, pcloud):
        self.now_pointcloud = pcloud

    def set_target_pointcloud(self, pcloud):
        self.pre_pointcloud = pcloud
        self.remove_nan_point(self.pre_pointcloud)
        # 构建前一帧点云的KD树
        self.kdt = KDTree(self.pre_pointcloud)

    def remove_nan_point(pointcloud):
        np.delete(pointcloud, np.where(np.isnan(pointcloud)))

    def match(self):
        # 若没有计算法向量，则计算法向量
        if self.have_normals == False:
            # 寻找附近点
            for point in self.pre_pointcloud:
                dist ,ind = self.kdt.query(point, k=20)
            points = self.pre_pointcloud[ind]

            



            
    