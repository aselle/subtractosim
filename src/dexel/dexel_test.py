# Licensed to the Apache Software Foundation (ASF) under one
# or more contributor license agreements.  See the NOTICE file
# distributed with this work for additional information
# regarding copyright ownership.  The ASF licenses this file
# to you under the Apache License, Version 2.0 (the
# "License"); you may not use this file except in compliance
# with the License.  You may obtain a copy of the License at
#
#   http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
# KIND, either express or implied.  See the License for the
# specific language governing permissions and limitations
# under the License.    

import time
import math
from tridexel import Cylinder, Sphere, TriDexelImage, V3f, Box
import numpy as np
import open3d as o3d

# base work
DX = .15
# sphere = Sphere(V3f(0,0,0), 10)
box = Box(V3f(-9,-3,-9), V3f(9,8,9), 0)


sphere2 = Sphere(V3f(0,8,0), 5, 0)
tridex = TriDexelImage(DX, box.box())
tridex.trace(box)
tridex2 = TriDexelImage(DX, sphere2.box())
tridex2.trace(sphere2)
tridex.inplace_union(tridex2)

class Timer:
    def __init__(self):
        self.timers = {}
        self.cumulative = {}
    
    def start(self,name):
        self.timers[name] = time.time()

    def stop(self,name):
        stop_time = time.time()
        if name not in self.cumulative:
            self.cumulative[name] = 0.
        self.cumulative[name] += stop_time - self.timers[name]

    def report(self):
        for name, time in self.cumulative.items():
            print("%10.2f %s" % (time, name))



start =time.time()

vis = o3d.visualization.Visualizer()
vis.create_window()

# geometry is the point cloud used in your animaiton
work_geometry = o3d.geometry.PointCloud()
cutter_geometry = o3d.geometry.PointCloud()

def update_geometry(geometry, tridex, color=[1,1,1]):
    pc  =tridex.get_pointcloud()
    
    cloud = np.array(pc) # .astype(np.float64)
    points = cloud[...,:3].astype(np.float64)
    normals = cloud[...,3:6].astype(np.float64)
    colors = np.array([[0,1,1], [0,1,0], [1,1,1]], np.float64)
    color_code = cloud[:,6].astype(np.uint32)
    rs = color_code 
    gs = np.ones(shape=(cloud.shape[0]))
    colors = colors[color_code]

    geometry.points = o3d.utility.Vector3dVector(points)
    geometry.normals = o3d.utility.Vector3dVector(normals)
    geometry.colors = o3d.utility.Vector3dVector(colors) # .astype(np.float64))
    # geometry.estimate_normals()
    vis.update_geometry(geometry)

update_geometry(work_geometry, tridex)
vis.add_geometry(work_geometry)
vis.add_geometry(cutter_geometry)

frame = 0
def cap_vid(vis):
    global frame
    # vis.capture_screen_image("test.%04d.jpg" % frame)
    frame+=1

timer = Timer()

timer.start("Testo")
timer.stop("Testo")
N=320+1
display_offset=128
timer.start("pass1")
for r in [13., 12.,11.,10.,9.]:
    startpass = time.time()
    for i in range(N):
        t = i/float(N) * 2*math.pi
        z = math.cos(t) * r
        x = math.sin(t) * r
        cylinder = Cylinder(V3f(x,5,z), 5, 1, 1)
        timer.start("trace")
        tridex_cutter2 = TriDexelImage(DX, cylinder.box())
        tridex_cutter2.trace(cylinder)
        timer.stop("trace")
        timer.start("cut")
        tridex.inplace_cut(tridex_cutter2)
        timer.stop("cut")
        start = time.time()
        if i%display_offset == 0:
            timer.start("update_geometry")
            update_geometry(cutter_geometry, tridex_cutter2, [1,0,1])
            update_geometry(work_geometry, tridex, [1,1,1])
            timer.stop("update_geometry")

            timer.start("update_renderer")
            vis.update_renderer()
            timer.stop("update_renderer")
            cap_vid(vis)

            timer.start("poll")
            vis.poll_events()
            timer.stop("poll")
            timer.start("update_renderer")
            vis.update_renderer()
            timer.stop("update_renderer")

    
    stoppass = time.time()
    print(f"Pass took {stoppass-startpass}")
    timer.report()

    # tridex_last = None
    # for i in range(N):
    #     t = i/float(N) * 2*math.pi
    #     z = math.cos(t) * r
    #     x = math.sin(t) * r
    #     cylinder = Cylinder(V3f(x,5,z), 10, 1)
    #     tridex_cutter2 = TriDexelImage(DX, cylinder.box())
    #     tridex_cutter2.trace(cylinder)
    #     #tridex.inplace_cut(tridex_cutter2)
    #     if tridex_last:
    #         tridex_last.inplace_union(tridex_cutter2)
    #     else:
    #         tridex_last = tridex_cutter2

    #     start = time.time()
    #     if i%display_offset == 0: 
    #         tridex.inplace_cut(tridex_last)

    #         update_geometry(cutter_geometry, tridex_last, [1,0,1])
    #         tridex_last = None
    #         update_geometry(work_geometry, tridex, [1,1,1])
    #         vis.update_renderer()
    #         cap_vid(vis)

    #     vis.poll_events()
    #     vis.update_renderer()

    # if tridex_last: 
    #     tridex.inplace_cut(tridex_last)
    #     tridex_last = None


    stoppass = time.time()
    print(f"Pass took {stoppass-startpass}")
timer.stop("pass1")
timer.report()

for r in [4.,3.5,3.,2.5,2.,1.5,1.,.75]:
    for i in range(N):
        
        t = i/float(N) * 2*math.pi
        z = math.cos(t) * r
        x = math.sin(t) * r
        cylinder = Cylinder(V3f(x,7,z), 10, .5, 1)
        tridex_cutter2 = TriDexelImage(DX, cylinder.box())
        tridex_cutter2.trace(cylinder)
        tridex.inplace_cut(tridex_cutter2)

        if i%display_offset == 0:
            update_geometry(cutter_geometry, tridex_cutter2, [1,0,1])
            update_geometry(work_geometry, tridex, [1,1,1])
            vis.update_renderer()
            cap_vid(vis)

            vis.poll_events()
            vis.update_renderer()

while 1:
    vis.poll_events()
    vis.update_renderer()