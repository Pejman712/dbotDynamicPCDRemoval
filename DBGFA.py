import numpy as np
from sklearn.neighbors import KDTree
import matplotlib.pyplot as plt
import os
import open3d as o3d
from scipy.spatial import ConvexHull
from sklearn.cluster import KMeans, DBSCAN, OPTICS
from sklearn.preprocessing import StandardScaler
import copy
from PIL import Image
import time
import sys


#############ALGORITHM INPUTS#########################################################################################################

averagedensity,dence,odistances,k = [],[],[],100
algorithm = True
voxel_size = 0.6

method= "Average"
#method = "Sphere_Based_Volume"
#method = "ConvexHull"
address = "saving address"
pointsize = 2
ND = True



#############Functions###########################################################################################################################
def Stationary_Dynamic_visualization(source, target):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([0, 1, 0])
    target_temp.paint_uniform_color([1, 0, 0])
    o3d.visualization.draw_geometries([source_temp, target_temp])
def prefitler(file,gtreshold,ctreshold,render):
# Load the point cloud data
    pcd = o3d.io.read_point_cloud(file)
# Convert the point cloud data to a NumPy array
    points = np.asarray(pcd.points)

# Apply DBSCAN to cluster the points
    dbscan = DBSCAN(eps=0.8, min_samples=2)
    labels = dbscan.fit_predict(points)
    x=[]
    for i in labels:
        if i > 0:
            x.append(i)
# Identify the ground cluster based on label assignments
    ground_cluster_label = np.argmax(np.bincount(x))
    ground_indices = np.where(labels == ground_cluster_label)[0]
    ground_points = points[ground_indices]

# Create a new point cloud object with the ground points
    din = []
    b = 0
    for i in ground_points:
        if i[2] < gtreshold or i[2] > ctreshold:
             din.append(b)
        b = int(b+1)
    ndin= np.array(din)
    ground_points  = np.delete(ground_points,ndin, axis=0)
    ground_pcd = o3d.geometry.PointCloud()
    ground_pcd.points = o3d.utility.Vector3dVector(ground_points)
    o3d.io.write_point_cloud("filtered_{}".format(file), ground_pcd )

# Visualize the ground points (optional)
    

# Remove the ground points from the original point cloud
    non_ground_indices = np.where(labels != ground_cluster_label)[0]
    non_ground_points = points[non_ground_indices]

    # Create a new point cloud object with the non-ground points
    non_ground_pcd = o3d.geometry.PointCloud()
    non_ground_pcd.points = o3d.utility.Vector3dVector(non_ground_points)

    # Visualize the non-ground points (optional)
    if render:
        o3d.visualization.draw_geometries([pcd])
        o3d.visualization.draw_geometries([ground_pcd])
        o3d.visualization.draw_geometries([non_ground_pcd])

def saveimage(source,name, address, pointsize):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    opt = vis.get_render_option()
    opt.point_size = pointsize
    vis.add_geometry(source)
    image = vis.capture_screen_image("{}/{}.png".format(address,name),do_render=True)
    vis.destroy_window()

def densityfiltering(pcd_data,dense,npodist,treshold,iteration_number,step,n_neighbors,method):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pcd_data)
    dindex,j = [],0
    dindex.clear()
    #dense.clear()
    for i in dense:
        equalizer = npodist[j]
        if float(i)*equalizer < treshold:
        #if float(i) < treshold:
            dindex.append(j)
        j=j+1
    points  = np.delete(pcd_data,dindex, axis=0)
    if iteration_number == 1:
        #print("iteration completed")
        return points,dindex,dense
    else:
        #print("iteration compeleted")
        densityfiltering(points,treshold+step,iteration_number-1,step,n_neighbors,method)
        return points,dindex,dense

def progressbar(current_value,total_value,bar_lengh,progress_char): 
    percentage = int((current_value/total_value)*100)                                                # Percent Completed Calculation 
    progress = int((bar_lengh * current_value ) / total_value)                                       # Progress Done Calculation 
    loadbar = "Progress: [{:{len}}]{}%".format(progress*progress_char,percentage,len = bar_lengh)    # Progress Bar String
    print(loadbar, end='\r')   
    
def preprocess_point_cloud(pcd, voxel_size):
    print(":: Downsample with a voxel size %.3f." % voxel_size)
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: Estimate normal with search radius %.3f." % radius_normal)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh

def prepare_dataset(source,target,voxel_size):
    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh

def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    #distance_threshold = 0.1
    #print(":: RANSAC registration on downsampled point clouds.")
    #print("   Since the downsampling voxel size is %.3f," % voxel_size)
    #print("   we use a liberal distance threshold %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result

def saveimage(source,name, address, pointsize):
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(source)
    vis.get_render_option().point_size = pointsize
    image = vis.capture_screen_image("{}/{}.png".format(address,name),do_render=True)
    vis.destroy_window()

def DBGFA(nmap,voxel_size):
    transfromdata = []
    print("Welcome to the Density Based Ghost Filtering Algorithm!\n")
    if nmap == 1:
        file = input("Enter the pcd file name (e.g. k3.pcd): ")
        prefitler(file,0.01,2,False)
        source1 = o3d.io.read_point_cloud("filtered_{}".format(file))
        saveimage(source1,"{} map".format(file), address, pointsize)
        return source1.points
    else:
        targetfile = input("Enter the Target pcd(e.g. K3.pcd): ")
        prefitler(targetfile,0.01,2,False)
        target = o3d.io.read_point_cloud("filtered_{}".format(targetfile))
        saveimage(target,"{} map".format(targetfile), address, pointsize)
        for i in range(nmap-1):
            sourcefile = input("Enter the source{} pcd(e.g. k3.pcd): ".format(str(i+1)))
            prefitler(sourcefile,0.01,2,False)
            source = o3d.io.read_point_cloud("filtered_{}".format(sourcefile))
            saveimage(source,"{} map".format(sourcefile), address, pointsize)
            source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, voxel_size)
            result_ransac = execute_global_registration(source_down, target_down,source_fpfh, target_fpfh, voxel_size)
            threshold = 1
            result_icp = o3d.pipelines.registration.registration_icp(
            source, target, threshold, result_ransac.transformation,
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
            source.transform(result_icp.transformation)
            transfromdata.append(source.points)
        pcdpoints = transfromdata[0]
        for i in range(len(transfromdata)-1):
            pcdpoints = np.concatenate((pcdpoints, transfromdata[i+1]), axis=0)
        pcdpoints = np.concatenate((pcdpoints, target.points), axis=0)
        #pcdpoints = np.concatenate(transfromdata, axis=0)
        return pcdpoints

##########################################################################################################################################


number = int(input("number of map:"))  
pcd_data = np.array(DBGFA(number ,voxel_size))
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pcd_data)
o3d.visualization.draw_geometries([pcd])


saveimage(pcd,"allmaps",address,pointsize)


tree = KDTree(pcd_data)
distances, kindex = tree.query(pcd_data, k)
center = pcd.get_center()
for i in range(len(pcd_data)):
    dist = np.linalg.norm(pcd_data[i] - center)
    odistances.append(dist)
npodist= np.array(odistances)
if method == "Sphere_Based_Volume": 
    for i in distances:
        volume = (4/3) * np.pi * np.power(np.max(i), 3)
        dence.append(k/volume)
elif method == "ConvexHull":
    for i in kindex:
        volume = ConvexHull(pcd_data[i]).volume
        dence.append(k/volume)    
elif method == "Average":
    for i in distances:
        volume= (4/3) * np.pi * np.power(np.average(i), 3)
        dence.append(k/volume)    
npdence= np.array(dence)

npdence = np.power(npdence,2)

# Perform element-wise multiplication on nested arrays

result_nested_array = np.multiply(npodist, npdence)

# Print the resulting nested array
#print(result_nested_array)


if ND:
    #Normal Disturbution 
    plt.hist(result_nested_array, bins=100, density=True, alpha=0.7, color='blue')

    # Add a line plot of the normal distribution
    x = np.linspace(np.min(result_nested_array), np.max(result_nested_array), 1000)
    mean = np.mean(result_nested_array)
    std = np.std(result_nested_array)
    y = (1 / (np.sqrt(2 * np.pi) * std)) * np.exp(-(x - mean)**2 / (2 * std**2))
    plt.plot(x, y, color='red', linewidth=2)
    # Set the plot title and labels
    plt.title('Normal Distribution')
    plt.xlabel('Points Density')
    plt.ylabel('Density')
    plt.savefig('{}my_plot.png'.format(address),dpi=1500)


# Display the plot
layor = 10
nextstep=int(np.std(dence))
staring_point = 100 
ending_point = int(np.average(dence)) + nextstep * layor

print(staring_point,ending_point,nextstep)

frame = 50

iteration_number,step,n_neighbors =1,int(nextstep),k
p = 0
s = 1
per =False
if algorithm == True:
    for treshold in range(staring_point,ending_point,int(nextstep/frame)):
        points1,dp1,dence1 = densityfiltering(pcd_data,dence,npodist,treshold,iteration_number,step,n_neighbors,method)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points1)
        if 100*len(dp1)/len(pcd_data)%5 < 1:
        #pcd1 = o3d.geometry.PointCloud()
        #pcd1.points = o3d.utility.Vector3dVector(pcd_data[dp1]) 
            o3d.io.write_point_cloud("{}/{} Method, {} nearest points, {:.2f} percent deleted points with treshold of {}.pcd".format(address,method,k,100*len(dp1)/len(pcd_data),treshold),pcd)
                    #o3d.1io.write_point_cloud("{} Method, {} nearest points, {:.2f} percent deleted points with treshold of {}.ply".format(method,k,100*len(dp1)/len(pcd_data),treshold),pcd)
                    #np.savetxt("{} Method, {} nearest points, {:.2f} percent deleted points with treshold of {}.ply".format(method,k,100*len(dp1)/len(pcd_data),treshold), points1, delimiter=" "
            vis = o3d.visualization.Visualizer()     
            vis.create_window()
            opts = vis.get_render_option()
            opts.point_size = pointsize
            vis.add_geometry(pcd)
            del opts
            image = vis.capture_screen_image("{}/{} Method, {} nearest points, {:.2f} percent deleted points with treshold of {} and {}.png".format(address,method,k,100*len(dp1)/len(pcd_data),treshold,iteration_number),do_render=True)
            vis.destroy_window()
        progressbar(100*len(dp1)/len(pcd_data),40,30,'■') 
        if 100*len(dp1)/len(pcd_data) > 31:
            print("Stopping the code.")
            elapsed_time = time.time() - start_time
            print(f"Processing time: {elapsed_time} seconds")
            sys.exit()
 
            

        
        
        

