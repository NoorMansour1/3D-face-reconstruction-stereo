# 3D-face-reconstruction-stereo
This project aims to recreate a 3D model of a face given two stereo image pairs. The disparity between corresponding points in the stereo image pairs encode the depth information about the object in the 3D world scene. After stereo rectification, the disparities between the two images are calculated with the SemiGlobal matching Method (SGM). Given this disparity map, we create a pointcloud and apply some processing to filter outliers and fill gaps. Because two image pairs are provided, we create a pointcloud for each one and combine them using the Iterative Closest Point (ICP) algoirthm. The final step is to create a mesh out of the combined point cloud and estimate the quality of it. 

Left Side Mesh      |  Combined Mesh
:-------------------------:|:-------------------------:
![Second](/results/Mesh_Left_Side.png) | ![First](/results/meshed.png)



