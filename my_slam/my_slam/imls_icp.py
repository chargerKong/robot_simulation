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

        covariance = np.ones((3, 3))

        for i in range(self.m_iterations):
            # 位姿变换
            in_cloud = np.zeros(shape=(len(self.now_pointcloud), 2))
            for ix in range(len(self.now_pointcloud)):
                origin_pose = np.array([*self.now_pointcloud[ix], 1])
                now_pose = result @ origin_pose
                in_cloud[ix] = np.array([now_pose[0], now_pose[1]])
     
        ref_cloud, ref_normal, in_cloud = self.projection_now_to_target(in_cloud)

        if len(in_cloud) < 5 or len(ref_cloud) < 5:
            print("Not Enough Correspondence: {}, {}".format(len(in_cloud), len(ref_cloud)))
            return False, result, None
        # 计算帧间位移，从当前的now到之前的target
        is_success, delta_trans = self.solve_motion(in_cloud,
                                                    ref_cloud,
                                                    ref_normal)

        return True, result, None
        
    def solve_motion(self, in_cloud, ref_cloud, ref_normal):
        """
        求解now在target下的位姿
        """
        M = np.zeros((4, 4))
        gt = np.zeros((1, 4))


        for i in range(len(in_cloud)):
            p = in_cloud[i]
            refp = ref_cloud[i]
            ni = np.expand_dims(ref_normal[i], axis=1)
            # define of point to line metric
            Ci = ni @ ni.T
            # construct Mi
            Mi = np.array([[1, 0, p[0], -p[1]], [0, 1, p[1], p[0]]])
            M += Mi.T @ Ci @ Mi
            gt += -2 * np.expand_dims(refp, axis = 0) @ Ci @ Mi

        g = gt.T

        M = 2 * M
        A = M[0:2, 0:2]
        B = M[0:2, 2:4]
        D = M[2:4, 2:4]

        S = D - B.T@np.linalg.inv(A)@B
        SA = np.linalg.det(S) * np.linalg.inv(S)

        # 左边式子a lambda^2 + b lambda + c
        t = np.zeros((4, 4))
        t[:2, :2] = np.linalg.inv(A) @ B @ SA.T @ SA @ B.T @ np.linalg.inv(A.T)
        t[:2, 2:4] = -np.linalg.inv(A) @ B @ SA.T @ SA
        t[2:4, :2] = t[:2, 2:4].T
        t[2:4, 2:4] = SA.T @ SA
        c = g.T @ t @ g

        t[:2, :2] = np.linalg.inv(A) @ B @ SA @ B.T @ np.linalg.inv(A.T)
        t[:2, 2:4] = -np.linalg.inv(A.T) @ B @ SA
        t[2:4, :2] = t[:2, 2:4].T
        t[2:4, 2:4] = SA
        b = 4 * g.T @ t @ g

        t[:2, :2] = np.linalg.inv(A) @ B @ B.T @ np.linalg.inv(A.T)
        t[:2, 2:4] = -np.linalg.inv(A) @ B
        t[2:4, :2] = t[:2, 2:4].T
        t[2:4, 2:4] = np.identity(2)
        a = 4 * g.T @ t @ g

        # 计算(31)右边lambda的系数，lambda_coeff分别对应0次-4次
        p = [-np.linalg.det(S), 2 * (S[0,0] + S[1,1])]
        lambda_coeff = [16, 8 * p[1],p[1] * p[1] + 4 * p[0], 2 * p[1] * p[0], p[0]]
        # lambda_coeff = [p[0], 2 * p[1] * p[0], p[1] * p[1] + 4 * p[0],8 * p[1], 16]
        # 左右合并
        poly_coeff = np.array([
            - lambda_coeff[0],
            - lambda_coeff[1],
            np.squeeze(a) - lambda_coeff[2],
            np.squeeze(b) - lambda_coeff[3],
            np.squeeze(c) - lambda_coeff[4]
        ]) 
        # 求解四次多项式
        lambda_, is_success = self.solver_fourth_order(poly_coeff)
        
        if not is_success:
            self._logger.info("solve polynomial failed")
            return False, None

        W = np.zeros((4, 4))
        W[2:4, 2:4] = np.identity(2)
        res = -np.linalg.inv((2 * M + 2 * lambda_ * W)) @ g
        # res1 = -np.linalg.inv((2 * M + 2 * lambda_ * W)) @ g
        # theta = np.arctan2(res[3], res[2])
        delta_Trans = np.array([
            [res[2], -res[3], res[0]],
            [res[3], res[2], res[1]],
            [0, 0, 1]
        ])
        return True, delta_Trans


    def projection_now_to_target(self, in_cloud):
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

    def solver_fourth_order(self, coeff):
        """
        求解多项式的根，返回第一个实根
        """
        for ans in np.roots(coeff):
            if isinstance(ans, np.complex128) or isinstance(ans, np.complex64) or np.abs(ans) < 1e-4:
                continue
                
            return ans, True
        return 0, False
