#!/usr/bin/env python3
import torch
import open3d.ml.torch as ml3d
import open3d as o3d
import numpy as np
from datetime import datetime
from skimage.measure import LineModelND, ransac
import code,os,sys,struct    

dic = {"x":[-0.5,1], #red for x
        "y":[-0.5,0.5],#green for y
        "z":[0,1]} #blue for z


class Edge:

    def __init__(self) -> None:
        pass



    def edge_extraction(self,pcd):
        plane_z = 0
        startTime = datetime.now()

        bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-0.4, -0.1, 0.45), max_bound=(0.5, 0.15, 0.7)) 
        pcd = pcd.crop(bbox)
        pcd = pcd.uniform_down_sample(every_k_points=15)


        while abs(plane_z) < 0.5 and len(np.asarray(pcd.points)) > 500:
            plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                                    ransac_n=3,
                                                    num_iterations=1000)
            [_, _, plane_z, _] = plane_model
        
            pcd = pcd.select_by_index(inliers) if abs(plane_z) > 0.5 else pcd.select_by_index(inliers, invert=True)
        
        #o3d.visualization.draw_geometries([pcd])

        wall_centroid = o3d.geometry.OrientedBoundingBox.get_oriented_bounding_box(pcd)
        tensor_points = torch.from_numpy(np.asarray(pcd.points))
        res = 2.5
        NearestN = 150 # thicker edge

        i = torch.arange(len(tensor_points))

        nsearch = ml3d.layers.KNNSearch(return_distances=True)
        ans = nsearch(tensor_points, tensor_points, NearestN)

        idx = ans.neighbors_index.reshape(len(tensor_points),NearestN).t()
        D = ans.neighbors_distance.reshape(len(tensor_points),NearestN).sqrt()
        NN = D[:,1]

        X,Y,Z = tensor_points[:,0],tensor_points[:,1],tensor_points[:,2]
        x,y,z = X[idx.long()],Y[idx.long()],Z[idx.long()]

        centroid = torch.stack([torch.mean(x, dim=0),
                                torch.mean(y, dim=0),
                                torch.mean(z, dim=0)]).t()

        dist = torch.sqrt(torch.sum((tensor_points - centroid) ** 2, dim=1))

        edge_tensor = torch.nonzero((dist > NN*res)*i).squeeze()

        edge_cloud = pcd.select_by_index(edge_tensor.tolist())
        print (f"EDGE EXTRACTIOIN time : {datetime.now() - startTime}" )

        
        edge_points, valid_edge_cloud = self.line_ransac(edge_cloud)

        return edge_points, valid_edge_cloud, wall_centroid
    
    
    def line_ransac(self, edge_cloud):
        startTime = datetime.now()

        A = []

        edge_cloud, _ = edge_cloud.remove_statistical_outlier(nb_neighbors=10,
                                                    std_ratio=0.01)
        edge_cloud = edge_cloud.voxel_down_sample(0.003)
        while len(np.asarray(edge_cloud.points)) > 100:
    
            # robustly fit line only using inlier data with RANSAC algorithm
            model_robust, inlier_TF = ransac(np.asarray(edge_cloud.points), LineModelND, min_samples=2,
                                        residual_threshold=0.005, max_trials=500)
            [point, direction] = model_robust.params
            indx = np.where(inlier_TF)[0]

            inline_cloud = edge_cloud.select_by_index(indx)
            edge_cloud = edge_cloud.select_by_index(indx,invert=True)

            inline_cloud.paint_uniform_color([1.0, 0, 0])
            edge_cloud.paint_uniform_color([0.5, 0.5, 0.5])
            
            
            if len(indx) < 100 or abs(direction[0]) < 0.5:
                continue

            A.append(np.asarray(inline_cloud.points))
        
        valid_point = np.concatenate(A, axis=0)
        valid_cloud = o3d.geometry.PointCloud()
        valid_cloud.points = o3d.utility.Vector3dVector(valid_point)
        valid_cloud.paint_uniform_color([1.0, 0, 0])

        print (f"line_ransac time : {datetime.now() - startTime}")
        return A, valid_cloud 


