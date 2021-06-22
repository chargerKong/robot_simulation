import numpy as np
from numpy.core.fromnumeric import shape
from numpy.linalg.linalg import norm
from sklearn.neighbors import KDTree

# from rosidl_parser.definition import Include

class Imls_icp:
    
    def __init__(self, _r=0.03, _h=0.1, _iter=10):
        self.m_r = _r
        self.m_h = _h
        self.m_iterations = _iter
        self.pre_pointcloud = None
        self.now_pointcloud = None
        # self.pre_kdt = None
        self.target_pointcloude_normal = None

    def set_source_pointcloud(self, pcloud):
        self.now_pointcloud = pcloud

    def set_target_pointcloud(self, pcloud):
        self.pre_pointcloud = pcloud
        self.remove_nan_point(self.pre_pointcloud)
        # 构建前一帧点云的KD树
        self.pre_kdt = KDTree(self.pre_pointcloud)
        self.have_normals = False

    def remove_nan_point(self, pointcloud):
        np.delete(pointcloud, np.where(np.isnan(pointcloud)))

    def match(self):
        # 若没有计算法向量，则计算法向量
        if self.have_normals == False:
            # 寻找附近点
            self.target_pointcloude_normal = np.zeros(shape=[len(self.pre_pointcloud), 2])
            for i in range(len(self.pre_pointcloud)):
                dist ,ind = self.pre_kdt.query([self.pre_pointcloud[i]], k=20)
                points = self.pre_pointcloud[ind[0]]
                # TODO: 判断一下points是不是都是非None
                
                # 计算每一个点的normal
                if len(points) > 3:
                    normal = self.compute_normal(points)
                else:
                    normal = [np.inf, np.inf]
                self.target_pointcloude_normal[i] = normal

        result = np.identity(3)
        covariance = np.ones(3)
        for i in range(self.m_iterations):
            # 位姿变换
            in_cloud = np.zeros(shape=(len(self.now_pointcloud), 2))
            for ix in range(len(self.now_pointcloud)):
                origin_pose = np.array([*self.now_pointcloud[ix], 1])
                now_pose = result @ origin_pose
                in_cloud[ix] = np.array([now_pose[0], now_pose[1]])
        # return True, result, None

        ref_cloud, ref_normal, in_cloud = self.projection_now_to_target(in_cloud)
        


    def project_now_to_target(self, in_cloud):
        """
        为每一个当前帧的点云中的激光点，计算匹配的点和对应的法向量
        return: out_cloud 当前帧的匹配点云
                out_normal 匹配点云对应的法向量
        """
        res_in_cloud = []
        res_out_cloud = []
        res_out_normal = []
        for idx in range(len(in_cloud)):
            # 找到该点在上一帧中的最近的点
            dist, ind = self.pre_kdt.query([in_cloud[idx]], k=1)
            # near_point = self.pre_pointcloud[ind[0]]
            near_normal = self.target_pointcloude_normal[ind[0]]
            # 如果法向量不存在，则不要这个点
            if np.any(near_normal == np.inf):
                continue
            # 如果距离太近了，也不要整个点
            if dist[0][0] > self.m_h * self.m_h:
                continue
            # 进行匹配
            height, is_success = self.implicit_mls_func(in_cloud[idx])
            if not is_success or not height:
                continue
            yi = in_cloud[idx] - height * near_normal
            res_in_cloud.append(in_cloud[idx])
            res_out_cloud.append(yi)
            res_out_normal.append(near_normal)
        return np.asarray(res_out_cloud), np.asarray(res_out_normal), np.asarray(res_in_cloud)
            

    def implicit_mls_func(self, xi):
        """
        主要用来曲面投影，xi为在曲面上的高度，用之前的pre_pointcloud构造一颗kd树
        """
        weight_sum = 0
        # 用于寻找xi最近的20个点
        search_num = 20
        ind, dist = self.pre_kdt.query_radius([xi], r=self.m_h, sort_results=True, return_distance=True)
        ind, dist = ind[:search_num], dist[:search_num]

        # 如果数量太少，则认为没有匹配点
        if len(ind) < 3:
            return False, 0

        # 计算距离
        denominator = 0
        numerator = 0
        for i in range(len(ind)):
            exph = -np.power(np.linalg.norm(xi - self.pre_pointcloud[i]), 2) / (self.m_h * self.m_h)
            w_x = np.math.exp(exph)
            
            numerator += w_x * (xi - self.pre_pointcloud[i].T @ self.target_pointcloude_normal[i])
            denominator += w_x

        height = numerator / denominator
        
        return True, height


    
    def compute_normal(self, points):
        """
        计算这些点的法向量
        """
        points = points - np.mean(points, axis=0)
        eigvalue, eigvector = np.linalg.eig(points.T@points)
        min_eig_index = np.argmin(eigvalue)
        return eigvector[:, min_eig_index]