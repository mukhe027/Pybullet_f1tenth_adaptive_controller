import numpy as np
import time
import matplotlib
matplotlib.use('pdf')
matplotlib.rcParams['pdf.fonttype'] = 42
matplotlib.rcParams['ps.fonttype'] = 42
from matplotlib import pyplot as plt
# import scipy.linalg as sp
import sys, os, fire, math
import pybullet
import seaborn as sns
from tqdm import tqdm
from datetime import datetime
from scipy.spatial.transform import Rotation as R
from gap_follower import GapFollower
from scipy.signal import place_poles
start_time = datetime.now().strftime("%Y-%m-%d%H-%M-%S")
script_name = os.path.split(sys.argv[0])[-1].split(".")[0]
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append('..')
eps = 1e-12

class BulletSim():
	def __init__(self):
		self.i = 0
		self.Vx = 2.0
		self.sim_ts = 0.002
		self.controller_freq = 25
		self.sim_num_ts = 100000
		self.text_id = None
		self.text_id_2 = None
		self.cam_fov = 60
		self.cam_aspect = 640 / 480
		self.cam_near = 0.01
		self.cam_far = 1000
		self.cam_view_matrix = pybullet.computeViewMatrix([0, 0, 0.5], [0, 0, 0], [1, 0, 0])
		self.cam_projection_matrix = pybullet.computeProjectionMatrixFOV(self.cam_fov, self.cam_aspect, self.cam_near, self.cam_far)
		self.distance = 100000
		self.gf = GapFollower()
		return 
		
	def run_sim(self):
		pybullet.connect(pybullet.GUI)
		pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_WIREFRAME,0)
		pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI,1)
		pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_Y_AXIS_UP,1)

		pybullet.resetSimulation()

		ground_tuple = pybullet.loadSDF(os.path.join(
					os.path.dirname(__file__), "./tracks/gbr/gbr.sdf"))

		new_orientation = pybullet.getQuaternionFromEuler((-np.pi/2, 0, 0))
		for gt_elem in ground_tuple:
			pybullet.resetBasePositionAndOrientation(gt_elem, (0, 0, 0), new_orientation)

		init_heading_euler = R.from_euler("YZX",[0.0,0.0,-90.0] , degrees=True)
		init_heading_quat  = init_heading_euler.as_quat()

		agent = pybullet.loadURDF(os.path.join(
					os.path.dirname(__file__), "./urdf/uva-f1tenth/uva-f1tenth-model.urdf"),[0.0, 0.204, 0.0],init_heading_euler.as_quat())

		# base_p_orient = pybullet.getBasePositionAndOrientation(agent) 

		pybullet.setGravity(0, -9.81, 0)
		pybullet.setTimeStep(self.sim_ts)
		pybullet.setRealTimeSimulation(0)
		long_velocity = self.Vx/0.05

		for gt_elem in ground_tuple:			
			pybullet.changeDynamics(gt_elem,0,lateralFriction=1.0,spinningFriction=0.0,rollingFriction=0.0,restitution=0)
		pybullet.changeDynamics(agent,2,lateralFriction=1.0,spinningFriction=0.0,rollingFriction=0.0,restitution=0)
		pybullet.changeDynamics(agent,3,lateralFriction=1.0,spinningFriction=0.0,rollingFriction=0.0,restitution=0)
		pybullet.changeDynamics(agent,5,lateralFriction=1.0,spinningFriction=0.0,rollingFriction=0.0,restitution=0)
		pybullet.changeDynamics(agent,7,lateralFriction=1.0,spinningFriction=0.0,rollingFriction=0.0,restitution=0)

		for i in np.arange(self.sim_num_ts):			
			pybullet.stepSimulation()
			# time.sleep(0.000001)
			
			if not (i%int((1/self.controller_freq) * (1/self.sim_ts))): 
				pybullet.setJointMotorControl2(bodyUniqueId=agent,
					jointIndex=2,
					controlMode=pybullet.VELOCITY_CONTROL,
					targetVelocity = long_velocity)
				pybullet.setJointMotorControl2(bodyUniqueId=agent,
					jointIndex=3,
					controlMode=pybullet.VELOCITY_CONTROL,
					targetVelocity = long_velocity)
				
				pybullet.setJointMotorControl2(bodyUniqueId=agent,
					jointIndex=5,
					controlMode=pybullet.VELOCITY_CONTROL,
					targetVelocity = long_velocity)
				pybullet.setJointMotorControl2(bodyUniqueId=agent,
					jointIndex=7,
					controlMode=pybullet.VELOCITY_CONTROL,
					targetVelocity = long_velocity)	

			# cg_pos_3d_xyz = np.array(pybullet.getLinkState(agent,8)[0])
			agent_pos, agent_orn = pybullet.getBasePositionAndOrientation(agent)
			cg_pos_3d_xyz = np.array(agent_pos)
			self.veh_2d_world_pos = cg_pos_3d_xyz[[0,2]]
			# cg_heading_3d_quat = R.from_quat(np.array(pybullet.getLinkState(agent,8)[1]))
			cg_heading_3d_quat = R.from_quat(np.array(agent_orn))
			self.veh_2d_world_heading  = -cg_heading_3d_quat.as_euler('YZX', degrees=False)[0]

			# Setup camera positioning.
			xA, yA, zA = agent_pos
			yA = yA + 0.3
			xB = xA + np.cos(self.veh_2d_world_heading) * self.distance
			zB = zA + np.sin(self.veh_2d_world_heading) * self.distance
			yB = yA

			view_matrix = pybullet.computeViewMatrix(
			                    cameraEyePosition=[xA, yA, zA],
			                    cameraTargetPosition=[xB, yB, zB],
			                    cameraUpVector=[0.0, 1.0, 0.0]
			                )

			projection_matrix = pybullet.computeProjectionMatrixFOV(
						fov=90, aspect=1.5, nearVal=0.02, farVal=3.5)

			if not (i%self.controller_freq):
				imgs = pybullet.getCameraImage(640, 480,
							view_matrix,
							projection_matrix, shadow=True,
							renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
				
				true_scan_depths = self.get_true_depth_values(imgs[3][240,:])
				_, steering_angle = self.gf.process_lidar(true_scan_depths)	
				# self.target_steering_angle = np.deg2rad(self.linear_Cntrl(self.K,e_state))
				self.target_steering_angle = -steering_angle

				pybullet.setJointMotorControl2(bodyUniqueId=agent,
					jointIndex=4,
					controlMode=pybullet.POSITION_CONTROL,
					targetPosition = self.target_steering_angle)
				pybullet.setJointMotorControl2(bodyUniqueId=agent,
					jointIndex=6,
					controlMode=pybullet.POSITION_CONTROL,
					targetPosition = self.target_steering_angle)
		return 
	
	def get_true_depth_values(self, input):
		return self.cam_far * self.cam_near / (self.cam_far - (self.cam_far - self.cam_near)*input)


if __name__ == "__main__":
	bs = BulletSim()
	bs.run_sim()
	# bs.plot_error_states()
	quit()