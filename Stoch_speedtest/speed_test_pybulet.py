import pybullet as p
import time
from os import system
loopN = 100
p.connect(p.DIRECT)
#p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"), 0, 0, 0)
robot = p.loadURDF("/mesh_obj/stoch_two_obj.urdf")

start = time.time()
single_timestep = time.time()
system('clear')
for i in range(loopN):
    if(i == 1):
    	single_timestep = time.time()
    p.stepSimulation()
    if(i == 1):
    	print("Single_step_time",time.time()- single_timestep)
    
time_elapsed=time.time() - start
print("time_elapsed:",time_elapsed)
print("frequency:",loopN/time_elapsed)