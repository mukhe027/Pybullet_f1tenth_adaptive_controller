import numpy as np
import matplotlib
import random
#matplotlib.use('Agg')

'''matplotlib.use("pdf")
matplotlib.rcParams["pdf.fonttype"] = 42
matplotlib.rcParams["ps.fonttype"] = 42'''
from matplotlib import pyplot as plt

# import scipy.linalg as sp
import sys, os
import pybullet
import seaborn as sns
from tqdm import tqdm
from datetime import datetime
from scipy.spatial.transform import Rotation as R
from gap_follower import GapFollower
from torchDeepAdap.torch_controller_ackerman import MRAC
from torchDeepAdap.ref_ackerman import refModel
from torchDeepAdap.ackerman import ackerman
from torchDeepAdap.refLibrary import refSignal

 ###########Applying Linear Adaptive Controller##############
#Control and Simulation Parameters
learningON = 1
#Controller ON states
pratik_cntrl_ON = 1
adaptive_cntrl_ON = 1


total_iter = 20000#35000 berlin #25000 gbr #20000 montreal100000
start_state = np.reshape([0,0,0,0],(4,1))
env = ackerman(start_state)
ref_env = refModel(start_state)
ref_cmd = refSignal(int(total_iter))

initialize_state = np.reshape([0,0],(2,1))

disturbance_on = 1

 ###########Applying Linear Adaptive Controller##############

start_time = datetime.now().strftime("%Y-%m-%d%H-%M-%S")
script_name = os.path.split(sys.argv[0])[-1].split(".")[0]
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append("..")
eps = 1e-12


class BulletSim:
    def __init__(self):
        self.i = 0
        self.Vx = 2.0
        self.sim_ts = 0.002
        self.controller_freq = 25
        self.sim_num_ts = total_iter
        self.text_id = None
        self.text_id_2 = None
        self.cam_fov = 60
        self.cam_aspect = 640 / 480
        self.cam_near = 0.01
        self.cam_far = 1000
        self.cam_view_matrix = pybullet.computeViewMatrix(
            [0, 0, 0.5], [0, 0, 0], [1, 0, 0]
        )
        self.cam_projection_matrix = pybullet.computeProjectionMatrixFOV(
            self.cam_fov, self.cam_aspect, self.cam_near, self.cam_far
        )
        self.distance = 100000
        self.gf = GapFollower()
        return

    def run_sim(self):
        pybullet.connect(pybullet.GUI, options="--width=640 --height=480")
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_WIREFRAME, 0)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, 0)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_Y_AXIS_UP, 1)
        pybullet.configureDebugVisualizer(
            pybullet.COV_ENABLE_SEGMENTATION_MARK_PREVIEW, 0
        )
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_DEPTH_BUFFER_PREVIEW, 0)
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RGB_BUFFER_PREVIEW, 0)

        pybullet.resetSimulation()

        ground_tuple = pybullet.loadSDF(
            #os.path.join(os.path.dirname(__file__), "./tracks/circle_cw/circle_cw.sdf")#good track
            os.path.join(os.path.dirname(__file__), "./tracks/montreal/montreal.sdf")#good track
            #os.path.join(os.path.dirname(__file__), "./tracks/gbr/gbr.sdf")#good track
            #os.path.join(os.path.dirname(__file__), "./tracks/berlin/berlin.sdf")#good track
           
        )

        new_orientation = pybullet.getQuaternionFromEuler((-np.pi / 2, 0, 0))
        for gt_elem in ground_tuple:
            pybullet.resetBasePositionAndOrientation(
                gt_elem, (0, 0, 0), new_orientation
            )

        init_heading_euler = R.from_euler("YZX", [0.0, 0.0, -90.0], degrees=True)
        init_heading_quat = init_heading_euler.as_quat()

        agent = pybullet.loadURDF(
            os.path.join(
                os.path.dirname(__file__), "./urdf/uva-f1tenth/uva-f1tenth-model.urdf"
            ),
            [0.0, 0.204, 0.0],
            init_heading_euler.as_quat(),
        )

        # base_p_orient = pybullet.getBasePositionAndOrientation(agent)

        pybullet.setGravity(0, -9.81, 0)
        pybullet.setTimeStep(self.sim_ts)
        pybullet.setRealTimeSimulation(0)
        long_velocity = self.Vx / 0.2 #self.Vx / 0.05

        for gt_elem in ground_tuple:
            pybullet.changeDynamics(
                gt_elem,
                0,
                lateralFriction=0.05,
                spinningFriction=0.0,
                rollingFriction=0.0,
                restitution=0.0,
            )
        pybullet.changeDynamics(
            agent,
            2,
            lateralFriction=0.05,
            spinningFriction=0.0,
            rollingFriction=0.0,
            restitution=0.0,
        )
        pybullet.changeDynamics(
            agent,
            3,
            lateralFriction=0.05,
            spinningFriction=0.0,
            rollingFriction=0.0,
            restitution=0.0,
        )
        pybullet.changeDynamics(
            agent,
            5,
            lateralFriction=0.05,
            spinningFriction=0.0,
            rollingFriction=0.0,
            restitution=0.0,
        )
        pybullet.changeDynamics(
            agent,
            7,
            lateralFriction=0.05,
            spinningFriction=0.0,
            rollingFriction=0.0,
            restitution=0.0,
        )
        ###########Applying Linear Adaptive Controller##############
        context_state_ON = 0 #set lr= 0.01 and buffer_size=7000
        # For  context_state_ON = 0 set lr = 0.001 and buffer_size=5000
        mrac_gain = 0.000001
        if pratik_cntrl_ON:
            if adaptive_cntrl_ON:
                if context_state_ON:
                    vehicl = MRAC(5,1,mrac_gain)#state_dim = 4 original 0.00001
                else:
                    vehicl = MRAC(4,1,mrac_gain)#state_dim = 4 original 0.00001
            else:
                vehicl = MRAC(4,1,mrac_gain)#state_dim = 4 original 0.00001
                context_state_ON = 0


        total_cntrl_prev = 0.0
        disturbance = 0.0
        #plt.axis([0, 10, 0, 1])

        pos_rec_x = [0.0]
        pos_rec_y = [0.0]
        pos_rec_z = [0.0]
        orn_rec = [0.0]
        point_rec_x= [0.0]
        point_rec_z= [0.0]
        steering_angle_rec= [0.0]

        e1_rec = [0.0]
        e2_rec = [0.0]
        e1_dot_rec = [0.0]
        e2_dot_rec = [0.0]
        delta_rec = [0.0]
        lrnd_unct_rec = [0.0]


        if pratik_cntrl_ON:
        #Read pre-stored vehicle states
            with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/pos_rec_x.npy', 'rb') as f:
                pos_sto_x = np.load(f)

            with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/pos_rec_y.npy', 'rb') as f:
                pos_sto_y = np.load(f)
            with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/pos_rec_z.npy', 'rb') as f:
                pos_sto_z = np.load(f)

            with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/orn_rec.npy', 'rb') as f:
                orn_sto = np.load(f)


            e1_prev = 0.0
            e2_prev =0.0
            steering_angle_prev = 0.0
        
            #self.sim_num_ts = len(pos_sto_x)
            #ref_cmd = refSignal(int(self.sim_num_ts))
            ref_cmd.regCMD()

            e1_rec = [0.0]
            e2_rec = [0.0]
            e1_dot_rec = [0.0]
            e2_dot_rec = [0.0]
            delta_rec = [0.0]
            lrnd_unct_rec = [0.0]
            ii=0
        ###########Applying Linear Adaptive Controller##############
          
        for i in np.arange(self.sim_num_ts):
            pybullet.stepSimulation()
            print('Simulation Time Step: {}'.format(i))
            # time.sleep(0.000001)

            if not (i % int((1 / self.controller_freq) * (1 / self.sim_ts))):
                pybullet.setJointMotorControl2(
                    bodyUniqueId=agent,
                    jointIndex=2,
                    controlMode=pybullet.VELOCITY_CONTROL,
                    targetVelocity=long_velocity,
                )
                pybullet.setJointMotorControl2(
                    bodyUniqueId=agent,
                    jointIndex=3,
                    controlMode=pybullet.VELOCITY_CONTROL,
                    targetVelocity=long_velocity,
                )

                pybullet.setJointMotorControl2(
                    bodyUniqueId=agent,
                    jointIndex=5,
                    controlMode=pybullet.VELOCITY_CONTROL,
                    targetVelocity=long_velocity,
                )
                pybullet.setJointMotorControl2(
                    bodyUniqueId=agent,
                    jointIndex=7,
                    controlMode=pybullet.VELOCITY_CONTROL,
                    targetVelocity=long_velocity,
                )

            # cg_pos_3d_xyz = np.array(pybullet.getLinkState(agent,8)[0])
            agent_pos, agent_orn = pybullet.getBasePositionAndOrientation(agent)
            cg_pos_3d_xyz = np.array(agent_pos)
            self.veh_2d_world_pos = cg_pos_3d_xyz[[0, 2]]
            # cg_heading_3d_quat = R.from_quat(np.array(pybullet.getLinkState(agent,8)[1]))
            cg_heading_3d_quat = R.from_quat(np.array(agent_orn))
            self.veh_2d_world_heading = -cg_heading_3d_quat.as_euler(
                "YZX", degrees=False
            )[0]
            
            # Setup camera positioning.
            xA, yA, zA = agent_pos
            yA = yA + 0.3  #height, stays constant for planes
            xB = xA + np.cos(self.veh_2d_world_heading) * self.distance
            zB = zA + np.sin(self.veh_2d_world_heading) * self.distance
            yB = yA

            view_matrix = pybullet.computeViewMatrix(
                cameraEyePosition=[xA, yA, zA],
                cameraTargetPosition=[xB, yB, zB],
                cameraUpVector=[0.0, 1.0, 0.0],
            )

            projection_matrix = pybullet.computeProjectionMatrixFOV(
                fov=90, aspect=1.5, nearVal=0.02, farVal=3.5
            )

            if not (i % self.controller_freq):
                imgs = pybullet.getCameraImage(
                    640,
                    480,
                    view_matrix,
                    projection_matrix,
                    shadow=True,
                    renderer=pybullet.ER_BULLET_HARDWARE_OPENGL,
                )

                true_scan_depths = self.get_true_depth_values(imgs[3][240, :])
                _, steering_angle,angle_dist, max_dist = self.gf.process_lidar(true_scan_depths)
                #self.target_steering_angle = np.deg2rad(self.linear_Cntrl(self.K,e_state))
                
                ###########Applying Linear Adaptive Controller##############
                #Setup for feed back control states from Rajamani work
                #e1, e1_dot, e2, e2_dot
                if pratik_cntrl_ON:
                    
                    pos_sto_mag = np.sqrt(np.square(pos_sto_x[ii])+np.square(pos_sto_z[ii]))
                    pos_read_mag = np.sqrt(np.square(xA)+np.square(zA))

                    #e1= (pos_sto_mag - pos_read_mag)
                    e1= np.sqrt(np.square(pos_sto_x[ii]-xA)+np.square(pos_sto_z[ii]-zA))
                    #e1= (pos_sto_x[ii] - xA)/np.sin(self.veh_2d_world_heading)
            ###############check for spike in heading values ###############

                   
                    

                    e2 = (-((orn_sto[ii])*(np.pi/180) ) +((self.veh_2d_world_heading)))#in radians
                  
            ###############check for spike in heading values ###############
                    #e2 = (orn_sto[ii] -self.veh_2d_world_heading*(180/np.pi))#in degrees
                    e1_dot = (e1- e1_prev)/(1/self.controller_freq)
                    e2_dot = (e2- e2_prev)/(1/self.controller_freq)

                  


                    #if e2_dot >0.1 or e2_dot <-0.1:
                            #e2_dot =0.0

                    e1_prev = e1
                    e2_prev = e2

                    #vehicl_state = np.reshape([e1,e1_dot,e2,e2_dot],(4,1))
                    vehicl_state = np.zeros((4,1), dtype = float)
                    vehicl_state[0][0]  = e1
                    vehicl_state[1][0]  = e1_dot
                    vehicl_state[2][0]  = e2
                    vehicl_state[3][0]  = e2_dot
                    
                                    ###############Control Function Calls###################
                                    ###############Control Function Calls###################
                                    ###############Control Function Calls###################
                                    ###############Control Function Calls###################
                                    ###############Control Function Calls###################
                    env.applyCntrl(total_cntrl_prev,adaptive_cntrl_ON,vehicl_state.astype('float'))
                    if context_state_ON:
                          #ci = angle_dist - orn_sto[ii]*(np.pi/180)
                          ci = angle_dist - self.veh_2d_world_heading
                    else :
                          ci = 0.0
                    
                    #total_cntrl, pred_cntrl = vehicl.total_Cntrl(vehicl_state.astype('float'), ref_env.state.astype('float'), ref_cmd.refsignal[ii],adaptive_cntrl_ON, total_cntrl_prev)
                    total_cntrl, pred_cntrl, lrnd_unct = vehicl.total_Cntrl(env.state.astype('float'), ref_env.state.astype('float'), ref_cmd.refsignal[ii],adaptive_cntrl_ON, total_cntrl_prev,context_state_ON,ci)
                    
                    
                    
                    steering_angle =total_cntrl[0]
                    '''delta_steering_angle = abs(steering_angle - steering_angle_prev)/(1/self.controller_freq)
                    if delta_steering_angle >10:
                      steering_angle = steering_angle_prev'''
                    print('Steering Angle: {}'.format(steering_angle))                  
                    
                    ref_env.stepRefModel(ref_cmd.refsignal[ii],pred_cntrl,adaptive_cntrl_ON)

                    total_cntrl_prev = total_cntrl

                    steering_angle_prev = steering_angle
                                    ###############Control Function Calls###################
                                    ###############Control Function Calls###################
                                    ###############Control Function Calls###################
                                    ###############Control Function Calls###################
                                    ###############Control Function Calls###################
                                    ###############Control Function Calls###################


                    ii=ii+1
                    

                  #computing x,z points for farthest lidar reading distance 
                xC = xA + np.cos(angle_dist) * max_dist
                zC = zA + np.sin(angle_dist) * max_dist


                '''Trying to plot in real-time'''
                '''yy = np.random.random()
                plt.scatter(i, yy)
                plt.pause(0.05)
                plt.show()'''

                

                ###########Applying Linear Adaptive Controller##############
                #trueWeights = np.array([0.5314, 0.16918, -0.6245, 0.1095])
                trueWeights = np.array([0.2314, 0.16918, -0.5245, 0.1095])#works for deep L1  montreal
                #trueWeights = np.array([0.5314, 0.16918, -0.4245, 0.1095])#works for deep L1  gbr and berlin
               
                #trueWeights = np.array([0.0, 0.0, 0.0, 0.0])
                delta = np.dot(trueWeights,env.state)
                if disturbance_on:
                    disturbance = random.uniform(0,0.15) + delta#works for deep L1 montreal
                    #disturbance = random.uniform(0,0.2) + delta#works for deep L1 gbr and berlin
                    
                    #disturbance = random.uniform(0,0.2) + delta #works for only l1 adap
                
                self.target_steering_angle = -steering_angle+(disturbance) 

                pybullet.setJointMotorControl2(
                    bodyUniqueId=agent,
                    jointIndex=4,
                    controlMode=pybullet.POSITION_CONTROL,
                    targetPosition=self.target_steering_angle,
                )
                pybullet.setJointMotorControl2(
                    bodyUniqueId=agent,
                    jointIndex=6,
                    controlMode=pybullet.POSITION_CONTROL,
                    targetPosition=self.target_steering_angle,
                )

                #Record data for plotting
                pos_rec_x.append(xA)
                pos_rec_y.append(yA)
                pos_rec_z.append(zA)
                orn_rec.append( self.veh_2d_world_heading *(180/np.pi))#converting to degrees
                steering_angle_rec.append(steering_angle*(180/np.pi))

                point_rec_x.append(xC)
                point_rec_z.append(zC)

                if pratik_cntrl_ON:
                    e1_rec.append(e1)
                    e2_rec.append(e2)
                    e1_dot_rec.append(e1_dot)
                    e2_dot_rec.append(e2_dot)
                    
                    if disturbance_on:
                        delta_rec.append(disturbance[0])
                    if adaptive_cntrl_ON:
                        lrnd_unct_rec.append(lrnd_unct[0][0])

                
        #Trying to plot after code shuts down
        '''plt.figure(1)
        plt.plot(pos_rec, color='red', label='$x(t)$')
        plt.grid(True)
        plt.xlabel('time (s)')
        plt.ylabel('Lateral Error $(m)$')
        #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
        plt.legend()

        plt.show()'''
        
        '''file = open("file1.txt", "w+")
        content = str(pos_rec)
        file.write(content)
        file.close()'''
          #Data Collection
        with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/pos_rec_x.npy', 'wb') as f:
            np.save(f, pos_rec_x)
        with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/pos_rec_y.npy', 'wb') as f:
            np.save(f, pos_rec_y)
        with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/orn_rec.npy', 'wb') as f:
            np.save(f, orn_rec)
        with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/pos_rec_z.npy', 'wb') as f:
            np.save(f, pos_rec_z)
        with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/steering_angle_rec.npy', 'wb') as f:
            np.save(f, steering_angle_rec)


        with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/point_rec_x.npy', 'wb') as f:
            np.save(f, point_rec_x)

        with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/point_rec_z.npy', 'wb') as f:
            np.save(f, point_rec_z)

        if pratik_cntrl_ON:
            with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/e1_rec.npy', 'wb') as f:
                np.save(f, e1_rec)
            with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/e1_dot_rec.npy', 'wb') as f:
                np.save(f, e1_dot_rec)
            with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/e2_rec.npy', 'wb') as f:
                np.save(f, e2_rec)
            with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/e2_dot_rec.npy', 'wb') as f:
                np.save(f, e2_dot_rec)
            if disturbance_on:
                with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/uncnt_rec.npy', 'wb') as f:
                    np.save(f, delta_rec)
            if adaptive_cntrl_ON:
                with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/lrnd_uncnt_rec.npy', 'wb') as f:
                    np.save(f, lrnd_unct_rec)

        return
     

    def get_true_depth_values(self, input):
        return (
            self.cam_far
            * self.cam_near
            / (self.cam_far - (self.cam_far - self.cam_near) * input)
        )


if __name__ == "__main__":
    bs = BulletSim()
    bs.run_sim()
    # bs.plot_error_states()
    quit()
