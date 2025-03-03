import open3d as o3d
import numpy as np
import copy
def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])
#Read the ply point cloud file in the computer
source = o3d.read_point_cloud("C:/Users/mta/Downloads/proba_Cloud.xyz")  #source is the point cloud that needs to be registered
target = o3d.read_point_cloud("C:/Users/mta/Downloads/proba.xyz")  #target is the target point cloud

threshold = 0.02
trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         [-0.139, 0.967, -0.215, 0.7],
                         [0.487, 0.255, 0.835, -1.4], [0.0, 0.0, 0.0, 1.0]])
print("Initial alignment")
evaluation = o3d.registration.evaluate_registration(source, target, threshold, trans_init)
print(evaluation)
print("Apply point-to-point ICP")
reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint(),
        o3d.registration.ICPConvergenceCriteria(max_iteration = 2000))
print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)
draw_registration_result(source, target, reg_p2p.transformation)

