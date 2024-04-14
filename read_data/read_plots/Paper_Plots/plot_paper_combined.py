import numpy as np
import matplotlib.pyplot as plt
plt.rc('xtick', labelsize=15)    # fontsize of the tick labels
plt.rc('ytick', labelsize=15)    # fontsize of the tick labels
plt.rc('axes', labelsize=18)    # fontsize of the x and y labels
plt.rc('legend', fontsize=16)    # legend fontsize

from numpy import loadtxt


def main():
    ###############################################Linear Feedback###########################################################
    '''x = np.array([[1, 2, 3], 
              [4, 5, 6],
              [7, 8, 9]], np.int32)  
    with open('/home/mukhe027/workspace/f1tenth_bullet/file2.npy', 'wb') as f:
        np.save(f, x)'''
 
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Linear_Feedback/pos_rec_x.npy', 'rb') as f:
        pos_rec_x = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Linear_Feedback/pos_rec_y.npy', 'rb') as f:
        pos_rec_y = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Linear_Feedback/pos_rec_z.npy', 'rb') as f:
        pos_rec_z = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Linear_Feedback/orn_rec.npy', 'rb') as f:
        orn_rec = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Linear_Feedback/steering_angle_rec.npy', 'rb') as f:
        steering_angle_rec = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Linear_Feedback/point_rec_x.npy', 'rb') as f:
        point_rec_x = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Linear_Feedback/point_rec_z.npy', 'rb') as f:
        point_rec_z = np.load(f)


        #Read Lateral Dynamics control states

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Linear_Feedback/e1_rec.npy', 'rb') as f:
                e1_rec = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Linear_Feedback/e1_dot_rec.npy', 'rb') as f:
                e1_dot_rec =np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Linear_Feedback/e2_rec.npy', 'rb') as f:
                e2_rec = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Linear_Feedback/e2_dot_rec.npy', 'rb') as f:
                e2_dot_rec = np.load(f)        

    
        ###############################################L1_Adaptive###########################################################
    '''x = np.array([[1, 2, 3], 
              [4, 5, 6],
              [7, 8, 9]], np.int32)  
    with open('/home/mukhe027/workspace/f1tenth_bullet/file2.npy', 'wb') as f:
        np.save(f, x)'''
 
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/pos_rec_x.npy', 'rb') as f:
        pos_rec_x_1 = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/pos_rec_y.npy', 'rb') as f:
        pos_rec_y_1 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/pos_rec_z.npy', 'rb') as f:
        pos_rec_z_1 = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/orn_rec.npy', 'rb') as f:
        orn_rec_1 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/steering_angle_rec.npy', 'rb') as f:
        steering_angle_rec_1 = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/point_rec_x.npy', 'rb') as f:
        point_rec_x_1 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/point_rec_z.npy', 'rb') as f:
        point_rec_z_1 = np.load(f)


        #Read Lateral Dynamics control states

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/e1_rec.npy', 'rb') as f:
                e1_rec_1 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/e1_dot_rec.npy', 'rb') as f:
                e1_dot_rec_1 =np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/e2_rec.npy', 'rb') as f:
                e2_rec_1 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/e2_dot_rec.npy', 'rb') as f:
                e2_dot_rec_1 = np.load(f)   

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/uncnt_rec.npy', 'rb') as f:
        uncnt_rec_1 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/L1_Adaptive/lrnd_uncnt_rec.npy', 'rb') as f:
        lrnd_uncnt_rec_1 = np.load(f)

       ###############################################Deep_MRAC###########################################################
    '''x = np.array([[1, 2, 3], 
              [4, 5, 6],
              [7, 8, 9]], np.int32)  
    with open('/home/mukhe027/workspace/f1tenth_bullet/file2.npy', 'wb') as f:
        np.save(f, x)'''
 
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/pos_rec_x.npy', 'rb') as f:
        pos_rec_x_2 = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/pos_rec_y.npy', 'rb') as f:
        pos_rec_y_2 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/pos_rec_z.npy', 'rb') as f:
        pos_rec_z_2 = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/orn_rec.npy', 'rb') as f:
        orn_rec_2 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/steering_angle_rec.npy', 'rb') as f:
        steering_angle_rec_2 = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/point_rec_x.npy', 'rb') as f:
        point_rec_x_2 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/point_rec_z.npy', 'rb') as f:
        point_rec_z_2 = np.load(f)


        #Read Lateral Dynamics control states

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/e1_rec.npy', 'rb') as f:
                e1_rec_2= np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/e1_dot_rec.npy', 'rb') as f:
                e1_dot_rec_2 =np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/e2_rec.npy', 'rb') as f:
                e2_rec_2 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/e2_dot_rec.npy', 'rb') as f:
                e2_dot_rec_2 = np.load(f)   

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/uncnt_rec.npy', 'rb') as f:
        uncnt_rec_2 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_MRAC/lrnd_uncnt_rec.npy', 'rb') as f:
        lrnd_uncnt_rec_2 = np.load(f)


     ###############################################Deep_L1_Adaptive###########################################################
    '''x = np.array([[1, 2, 3], 
              [4, 5, 6],
              [7, 8, 9]], np.int32)  
    with open('/home/mukhe027/workspace/f1tenth_bullet/file2.npy', 'wb') as f:
        np.save(f, x)'''
 
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/pos_rec_x.npy', 'rb') as f:
        pos_rec_x_3 = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/pos_rec_y.npy', 'rb') as f:
        pos_rec_y_3 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/pos_rec_z.npy', 'rb') as f:
        pos_rec_z_3 = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/orn_rec.npy', 'rb') as f:
        orn_rec_3 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/steering_angle_rec.npy', 'rb') as f:
        steering_angle_rec_3= np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/point_rec_x.npy', 'rb') as f:
        point_rec_x_3 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/point_rec_z.npy', 'rb') as f:
        point_rec_z_3= np.load(f)


        #Read Lateral Dynamics control states

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/e1_rec.npy', 'rb') as f:
                e1_rec_3= np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/e1_dot_rec.npy', 'rb') as f:
                e1_dot_rec_3 =np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/e2_rec.npy', 'rb') as f:
                e2_rec_3= np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/e2_dot_rec.npy', 'rb') as f:
                e2_dot_rec_3 = np.load(f)   

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/uncnt_rec.npy', 'rb') as f:
        uncnt_rec_3 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive/lrnd_uncnt_rec.npy', 'rb') as f:
        lrnd_uncnt_rec_3 = np.load(f)

       ###############################################Deep_L1_Adaptive_w_c_state###########################################################
    '''x = np.array([[1, 2, 3], 
              [4, 5, 6],
              [7, 8, 9]], np.int32)  
    with open('/home/mukhe027/workspace/f1tenth_bullet/file2.npy', 'wb') as f:
        np.save(f, x)'''
 
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/pos_rec_x.npy', 'rb') as f:
        pos_rec_x_4 = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/pos_rec_y.npy', 'rb') as f:
        pos_rec_y_4 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/pos_rec_z.npy', 'rb') as f:
        pos_rec_z_4 = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/orn_rec.npy', 'rb') as f:
        orn_rec_4 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/steering_angle_rec.npy', 'rb') as f:
        steering_angle_rec_4= np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/point_rec_x.npy', 'rb') as f:
        point_rec_x_4 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/point_rec_z.npy', 'rb') as f:
        point_rec_z_4= np.load(f)


        #Read Lateral Dynamics control states

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/e1_rec.npy', 'rb') as f:
                e1_rec_4= np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/e1_dot_rec.npy', 'rb') as f:
                e1_dot_rec_4 =np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/e2_rec.npy', 'rb') as f:
                e2_rec_4= np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/e2_dot_rec.npy', 'rb') as f:
                e2_dot_rec_4 = np.load(f)   

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/uncnt_rec.npy', 'rb') as f:
        uncnt_rec_4 = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Deep_L1_Adaptive_w_c_state/lrnd_uncnt_rec.npy', 'rb') as f:
        lrnd_uncnt_rec_4 = np.load(f)

    ##################################################Reference Trajectories##########################################################################
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Reference/pos_rec_x.npy', 'rb') as f:
        ref_pos_rec_x = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Reference/pos_rec_y.npy', 'rb') as f:
        ref_pos_rec_y = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Reference/pos_rec_z.npy', 'rb') as f:
        ref_pos_rec_z = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Reference/orn_rec.npy', 'rb') as f:
        ref_orn_rec = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Reference/steering_angle_rec.npy', 'rb') as f:
        ref_steering_angle_rec = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Reference/point_rec_x.npy', 'rb') as f:
        ref_point_rec_x = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/Paper_Plots/Reference/point_rec_z.npy', 'rb') as f:
        ref_point_rec_z = np.load(f)
    time_rec = [0.0]
    time = 0.0
    time_step= 1/25 #controller frequency
    for idx in range(0,len(ref_pos_rec_y)):
           
            time = idx*time_step
            time_rec.append(time)


   
    
    
  
    plt.figure(1)
    
    ax1 = plt.subplot(511)
    plt.plot(time_rec[0:len(pos_rec_x)],pos_rec_x, color='red', label='$x_{LF}$')
    plt.plot(time_rec[0:len(pos_rec_x_1)],pos_rec_x_1, color='blue', label='$x_{L1adap}$')
    plt.plot(time_rec[0:len(pos_rec_x_2)],pos_rec_x_2, color='green', label='$x_{Deep-MRAC}$')
    plt.plot(time_rec[0:len(pos_rec_x_3)],pos_rec_x_3, color='pink', label='$x_{Deep-L1}$')
    plt.plot(time_rec[0:len(pos_rec_x_4)],pos_rec_x_4, color='grey', label='$x_{Deep-L1-con}$')
    plt.plot(time_rec[0:len(ref_pos_rec_x)],ref_pos_rec_x, color='black', label='$x_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    #plt.xlabel('time (s)')
    plt.ylabel('Position  $x(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend( loc="upper left")

    ax2 = plt.subplot(412)
    plt.plot(time_rec[0:len(pos_rec_z)],pos_rec_z, color='red', label='$z_{LF}$')
    plt.plot(time_rec[0:len(pos_rec_z_1)],pos_rec_z_1, color='blue', label='$z_{L1adap}$')
    plt.plot(time_rec[0:len(pos_rec_z_2)],pos_rec_z_2, color='green', label='$z_{Deep-MRAC}$')
    plt.plot(time_rec[0:len(pos_rec_z_3)],pos_rec_z_3, color='pink', label='$z_{Deep-L1}$')
    plt.plot(time_rec[0:len(pos_rec_z_4)],pos_rec_z_4, color='grey', label='$z_{Deep-L1-con}$')
    plt.plot(time_rec[0:len(ref_pos_rec_z)],ref_pos_rec_z, color='black', label='$z_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    #plt.xlabel('time (s)')
    plt.ylabel('Position $z(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend( loc="upper left")

    '''ax3 = plt.subplot(513)
    plt.plot(time_rec[0:len(pos_rec_z)],pos_rec_z, color='red', label='$z$')
    plt.plot(time_rec[0:len(pos_rec_z_1)],pos_rec_z_1, color='blue', label='$z_1$')
    plt.plot(time_rec[0:len(pos_rec_z_2)],pos_rec_z_2, color='green', label='$z_2$')
    plt.plot(time_rec[0:len(pos_rec_z_3)],pos_rec_z_3, color='pink', label='$z_3$')
    plt.plot(time_rec[0:len(pos_rec_z_4)],pos_rec_z_4, color='grey', label='$z_4$')
    plt.plot(time_rec[0:len(ref_pos_rec_z)],ref_pos_rec_z, color='black', label='$z_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Position $z(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()'''


    ax4 = plt.subplot(413)
    plt.plot(time_rec[0:len(orn_rec)],orn_rec*(np.pi/180), color='red', label='$\Theta_{LF}$')
    plt.plot(time_rec[0:len(orn_rec_1)],orn_rec_1*(np.pi/180), color='blue', label='$\Theta_{L1adap}$')
    plt.plot(time_rec[0:len(orn_rec_2)],orn_rec_2*(np.pi/180), color='green', label='$\Theta_{Deep-MRAC}$')
    plt.plot(time_rec[0:len(orn_rec_3)],orn_rec_3*(np.pi/180), color='pink', label='$\Theta_{Deep-L1}$')
    plt.plot(time_rec[0:len(orn_rec_4)],orn_rec_4*(np.pi/180), color='grey', label='$\Theta_{Deep-L1-con}$')
    plt.plot(time_rec[0:len(ref_orn_rec)],ref_orn_rec*(np.pi/180), color='black', label='$\Theta_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    #plt.xlabel('time (s)')
    plt.ylabel('Heading $\Theta^{\circ}$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend( loc="upper left")

    ax5 = plt.subplot(414)
    #plt.plot(time_rec[0:len(orn_rec)],orn_rec, color='red', label='$\Theta$')
    #plt.plot(time_rec[0:len(orn_rec)],ref_orn_rec, color='black', label='$\Theta_{ref}$')
    
    plt.plot(time_rec[0:len(steering_angle_rec)],steering_angle_rec, color='red', label='$\delta_{LF}$')
    plt.plot(time_rec[0:len(steering_angle_rec_1)],steering_angle_rec_1, color='blue', label='$\delta_{L1adap}$')
    plt.plot(time_rec[0:len(steering_angle_rec_2)],steering_angle_rec_2, color='green', label='$\delta_{Deep-MRAC}$')
    plt.plot(time_rec[0:len(steering_angle_rec_3)],steering_angle_rec_3, color='pink', label='$\delta_{Deep-L1}$')
    plt.plot(time_rec[0:len(steering_angle_rec_4)],steering_angle_rec_4, color='grey', label='$\delta_{Deep-L1-con}$')
    plt.plot(time_rec[0:len(ref_steering_angle_rec)],ref_steering_angle_rec, color='black', label='$\delta_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Steering Angle $\delta^{\circ}$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend( loc="upper left")

    
    ###########################################################################################################
    '''plt.figure(2)
    
    ax1 = plt.subplot(211)
    plt.plot(time_rec[0:len(point_rec_x)],point_rec_x, color='red', label='$x$')
    plt.plot(time_rec[0:len(point_rec_x)],ref_point_rec_x, color='black', label='$x_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Position Lidar Point  $x(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()

    ax2 = plt.subplot(212)
    plt.plot(time_rec[0:len(point_rec_z)],point_rec_z, color='red', label='$z$')
    plt.plot(time_rec[0:len(point_rec_z)],ref_point_rec_z, color='black', label='$z_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Position Lidar Point $z(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()'''

    ###########################################################################################################

    plt.figure(2)
    
    ax1 = plt.subplot(411)
    #plt.plot(time_rec[0:len(e1_rec)],e1_rec, color='red', label='$e_{1_{LF}}$')
    plt.plot(time_rec[0:len(e1_rec_1)],e1_rec_1, color='blue', label='$e_{1_{L1adap}}$')
    #plt.plot(time_rec[0:len(e1_rec_2)],e1_rec_2, color='green', label='$e_{1_{Deep-MRAC}}$')
    plt.plot(time_rec[0:len(e1_rec_3)],e1_rec_3, color='pink', label='$e_{1_{Deep-L1}}$')
    plt.plot(time_rec[0:len(e1_rec_4)],e1_rec_4, color='grey', label='$e_{1_{Deep-L1-con}}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    #plt.xlabel('time (s)')
    plt.ylabel('  $e_1 (m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend( loc="upper left")

    ax1 = plt.subplot(412)
    #plt.plot(time_rec[0:len(e1_dot_rec)],e1_dot_rec, color='red', label='$\dot{e_{1_{LF}}}$')
    plt.plot(time_rec[0:len(e1_dot_rec_1)],e1_dot_rec_1, color='blue', label='$\dot{e_{1_{L1adap}}}$')
    #plt.plot(time_rec[0:len(e1_dot_rec_2)],e1_dot_rec_2, color='green', label='$\dot{e_{1_{Deep-MRAC}}}$')
    plt.plot(time_rec[0:len(e1_dot_rec_3)],e1_dot_rec_3, color='pink', label='$\dot{e_{1_{Deep-L1}}}$')
    plt.plot(time_rec[0:len(e1_dot_rec_4)],e1_dot_rec_4, color='grey', label='$\dot{e_{1_{Deep-L1-con}}}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    #plt.xlabel('time (s)')
    plt.ylabel('  $\dot{e1} (m/s)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend( loc="upper left")

    ax1 = plt.subplot(413)
    #plt.plot(time_rec[0:len(e2_rec)],e2_rec, color='red', label='$e_{2_{LF}}$')
    plt.plot(time_rec[0:len(e2_rec_1)],e2_rec_1, color='blue', label='$e_{2_{L1adap}}$')
    #plt.plot(time_rec[0:len(e2_rec_2)],e2_rec_2, color='green', label='$e_{2_{Deep-MRAC}}$')
    plt.plot(time_rec[0:len(e2_rec_3)],e2_rec_3, color='pink', label='$e_{2_{Deep-L1}}$')
    plt.plot(time_rec[0:len(e2_rec_4)],e2_rec_4, color='grey', label='$e_{2_{Deep-L1-con}}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    #plt.xlabel('time (s)')
    plt.ylabel('  $e_2 (radians)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend( loc="upper left")


    ax1 = plt.subplot(414)
    #plt.plot(time_rec[0:len(e2_dot_rec)],e2_dot_rec, color='red', label='$\dot{e_{2_{LF}}}$')
    plt.plot(time_rec[0:len(e2_dot_rec_1)],e2_dot_rec_1, color='blue', label='$\dot{e_{2_{L1adap}}}$')
    #plt.plot(time_rec[0:len(e2_dot_rec_2)],e2_dot_rec_2, color='green', label='$\dot{e_{2_{Deep-MRAC}}}$')
    plt.plot(time_rec[0:len(e2_dot_rec_3)],e2_dot_rec_3, color='pink', label='$\dot{e_{2_{Deep-L1}}}$')
    plt.plot(time_rec[0:len(e2_dot_rec_4)],e2_dot_rec_4, color='grey', label='$\dot{e_{2_{Deep-L1-con}}}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('  $\dot{e2} (radians/s)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend( loc="upper left")

    

###########################################################################################################

    plt.figure(3)
    
    #ax1 = plt.subplot(411)
    plt.plot(pos_rec_z,pos_rec_x, color='red', label='$traj_{act_{LF}}$')
    plt.plot(pos_rec_z_1,pos_rec_x_1, color='blue', label='$traj_{act_{L1adap}}$')
    plt.plot(pos_rec_z_2,pos_rec_x_2, color='green', label='$traj_{act_{Deep-MRAC}}$')
    plt.plot(pos_rec_z_3,pos_rec_x_3, color='pink', label='$traj_{act_{Deep-L1}}$')
    plt.plot(pos_rec_z_4,pos_rec_x_4, color='grey', label='$traj_{act_{Deep-L1-con}}$')
    plt.plot(ref_pos_rec_z,ref_pos_rec_x, color='black', label='$traj_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('Position $z(m)$')
    plt.ylabel('Position  $x(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend( loc="lower center")

    ###########################################################################################################
    plt.figure(4)
    
    
    plt.plot(time_rec[0:len(uncnt_rec_3)],uncnt_rec_3, color='black', label='$\Delta(x)+ \overline{\Delta}(x)$ Act')
    plt.plot(time_rec[0:len(lrnd_uncnt_rec_1)],lrnd_uncnt_rec_1, color='blue', label='$\hat{\Delta}(x)+ \hat{\overline{\Delta}}(x)$ L1adap')
    plt.plot(time_rec[0:len(lrnd_uncnt_rec_2)],lrnd_uncnt_rec_2, color='green', label='$\hat{\Delta}(x)+ \hat{\overline{\Delta}}(x)$ Deep-MRAC')
    plt.plot(time_rec[0:len(lrnd_uncnt_rec_3)],lrnd_uncnt_rec_3, color='pink', label='$\hat{\Delta}(x)+ \hat{\overline{\Delta}}(x)$ Deep-L1')
    plt.plot(time_rec[0:len(lrnd_uncnt_rec_4)],lrnd_uncnt_rec_4, color='grey', label='$\hat{\Delta}(x)+ \hat{\overline{\Delta}}(x)$ with $c_i$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)',fontsize="16")
    plt.ylabel('Uncertainty $(radians)$',fontsize="16")
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend(loc="upper left")





    plt.show()





if __name__ == '__main__':
    main()
    print("Finished")
    exit()