import numpy as np
import pandas as pd

import vtk
import open3d as o3d

from pyntcloud.io import read_ply, write_ply
from pyntcloud import PyntCloud

name = "qns"
n = 1000
collection = []

class VtkPointCloud:
    def __init__(self, zMin=-10.0, zMax=10.0, maxNumPoints=1e6):
        self.maxNumPoints = maxNumPoints
        self.vtkPolyData = vtk.vtkPolyData()
        self.clearPoints()
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(self.vtkPolyData)
        mapper.SetColorModeToDefault()
        mapper.SetScalarRange(zMin, zMax)
        mapper.SetScalarVisibility(1)
        self.vtkActor = vtk.vtkActor()
        self.vtkActor.SetMapper(mapper)

    def addPoint(self, point):
        if self.vtkPoints.GetNumberOfPoints() < self.maxNumPoints:
            pointId = self.vtkPoints.InsertNextPoint(point[:])
            self.vtkDepth.InsertNextValue(point[2])
            self.vtkCells.InsertNextCell(1)
            self.vtkCells.InsertCellPoint(pointId)
        else:
            r = random.randint(0, self.maxNumPoints)
            self.vtkPoints.SetPoint(r, point[:])
        self.vtkCells.Modified()
        self.vtkPoints.Modified()
        self.vtkDepth.Modified()

    def clearPoints(self):
        self.vtkPoints = vtk.vtkPoints()
        self.vtkCells = vtk.vtkCellArray()
        self.vtkDepth = vtk.vtkDoubleArray()
        self.vtkDepth.SetName('DepthArray')
        self.vtkPolyData.SetPoints(self.vtkPoints)
        self.vtkPolyData.SetVerts(self.vtkCells)
        self.vtkPolyData.GetPointData().SetScalars(self.vtkDepth)
        self.vtkPolyData.GetPointData().SetActiveScalars('DepthArray')

def vtk_Renderer():
    pointCloud = VtkPointCloud()

    for i in range(len(result)):
        #print(result["x"][i])
        point = list()
        point.append(result["x"][i])
        point.append(result["y"][i])
        point.append(result["z"][i])
        pointCloud.addPoint(point)

    # Renderer
    renderer = vtk.vtkRenderer()
    renderer.AddActor(pointCloud.vtkActor)
    renderer.SetBackground(.2, .3, .4)
    renderer.ResetCamera()

    # Render Window
    renderWindow = vtk.vtkRenderWindow()
    renderWindow.AddRenderer(renderer)

    # Interactor
    renderWindowInteractor = vtk.vtkRenderWindowInteractor()
    renderWindowInteractor.SetRenderWindow(renderWindow)

    # Begin Interaction
    renderWindow.Render()
    renderWindowInteractor.Start()

def post_Processing():
    for i in range(len(result)):
    #print(result["x"][i])
        if(result["nx"][i] < 0.0):
            result = result.drop([i])

def create_LiDAR_sensor():
    mesh_sphere = o3d.geometry.TriangleMesh.create_sphere(radius=.1)
    mesh_sphere.compute_vertex_normals()
    mesh_sphere.paint_uniform_color([0.1, 0.1, 0.7])
    # scale, rotation, translation
    your_transform = np.asarray(
                [[0.862, 0.011, -0.507,  0.5],
                [-0.139, 0.967, -0.215,  0.7],
                [0.487, 0.255,  0.835, -1.4],
                [0.0, 0.0, 0.0, 1.0]])
    #mesh_sphere.transform()
    mesh_sphere.transform(your_transform)
    return mesh_sphere

def triangle_area_multi(v1,v2,v3):
    # compute area of multiple triangles in face-vertex-format
    # parms: v1,v2,v3 (N,3) ndarrays
    return 0.5 * np.linalg.norm(np.cross(v2-v1,v3-v1), axis=1)

def find_neighbour(pcd):
    ###KDTREE and NN
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    print("Find its neighbors with distance less than 0.2, paint green.")
    pointnr = 10
    distance = 0.2
    [k, idx, _] = pcd_tree.search_radius_vector_3d(pcd.points[pointnr], distance)
    np.asarray(pcd.colors)[idx[1:], :] = [0, 1, 0]

    print("Visualize the point cloud.")
    o3d.visualization.draw_geometries([pcd])

def interactive_mode():
    #mesh3d = o3d.io.read_triangle_mesh("suz.ply")
    '''
    print("Demo for manual geometry cropping")
    print(
        "1) Press 'Y' twice to align geometry with negative direction of y-axis"
    )
    print("2) Press 'K' to lock screen and to switch to selection mode")
    print("3) Drag for rectangle selection,")
    print("   or use ctrl + left click for polygon selection")
    print("4) Press 'C' to get a selected geometry and to save it")
    print("5) Press 'F' to switch to freeview mode")
    '''
    #pcd = o3d.io.read_point_cloud("../../TestData/ICP/cloud_bin_0.pcd")
    #o3d.visualization.draw_geometries_with_editing([pcd])

# coordinate system, xyz
coord = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.6, origin=[0, 0, 0])
#collection.append(coord)

# create lidar source
#mesh_sphere = create_LiDAR_sensor()
#collection.append(mesh_sphere)



# create point cloud
print("reading ply objects...")
mesh_raw = PyntCloud.from_file("./input.obj")

#mesh_raw = read_ply("input_raw.obj")
#print(mesh_raw["points"].head()) #vertex list
print(mesh_raw["mesh"]) #face list

#print(coord.is_intersecting(mesh_sphere))

# get x,y,z
mesh_points_xyz = mesh_raw["points"][["x","y","z"]].values
# get normals x,y,z
mesh_points_normals = mesh_raw["points"][["nx","ny","nz"]].values
# colors?
v1_xyz = mesh_points_xyz[mesh_raw["mesh"]["v1"]]
v2_xyz = mesh_points_xyz[mesh_raw["mesh"]["v2"]]
v3_xyz = mesh_points_xyz[mesh_raw["mesh"]["v3"]]

v1_normals = mesh_points_normals[mesh_raw["mesh"]["v1"]]
v2_normals = mesh_points_normals[mesh_raw["mesh"]["v2"]]
v3_normals = mesh_points_normals[mesh_raw["mesh"]["v3"]]

# The first thing we need to define is how many points will our output point cloud have. This is really situation-dependent and because of that our code should let the user define this parameter, which we’ll name n.

# Once we know how many points we need to generate, we have to randomly select n triangles of the mesh and generate one point, in a random position, inside it’s corresponding triangle.

areas = triangle_area_multi(v1_xyz, v2_xyz, v3_xyz)
probabilities = areas / areas.sum()

weighted_random_indices = np.random.choice(range(len(areas)), size=n, p=probabilities)
print(weighted_random_indices)

### now find points within the triangles - barycentric coordinates
v1_xyz = v1_xyz[weighted_random_indices]
v2_xyz = v2_xyz[weighted_random_indices]
v3_xyz = v3_xyz[weighted_random_indices]
v1_normals = v1_normals[weighted_random_indices]
v2_normals = v2_normals[weighted_random_indices]
v3_normals = v3_normals[weighted_random_indices]

u = np.random.rand(n,1)
v = np.random.rand(n,1)
is_a_problem = u + v > 1
u[is_a_problem] = 1 - u[is_a_problem]
v[is_a_problem] = 1 - v[is_a_problem]
w = 1 - (u + v)

result = pd.DataFrame()

result_xyz = (v1_xyz * u) + (v2_xyz * v) + (v3_xyz * w)
result_xyz = result_xyz.astype(np.float32)
result["x"] = result_xyz[:,0]
result["y"] = result_xyz[:,1]
result["z"] = result_xyz[:,2]
sum_normals = v1_normals + v2_normals + v3_normals
result_normals = sum_normals / np.linalg.norm(sum_normals, axis = 1)[..., None]
result_normals = result_normals.astype(np.float32)
result["nx"] = result_normals[:,0]
result["ny"] = result_normals[:,1]
result["nz"] = result_normals[:,2]

print(result.head())

# post processing of pcl in result df
#post_Processing()

write_ply("pointcloud.ply", points=result)
print(result)
print("PCL ready.")

### OUTPUT
# --> PCL_output.py


