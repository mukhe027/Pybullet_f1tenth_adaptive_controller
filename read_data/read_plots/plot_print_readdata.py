import numpy as np
import matplotlib.pyplot as plt
from numpy import loadtxt


def main():
    #sample test data for printing
    '''x = np.array([[1, 2, 3], 
              [4, 5, 6],
              [7, 8, 9]], np.int32)  
    with open('/home/mukhe027/workspace/f1tenth_bullet/file2.npy', 'wb') as f:
        np.save(f, x)'''
 
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/pos_rec_x.npy', 'rb') as f:
        pos_rec_x = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/pos_rec_y.npy', 'rb') as f:
        pos_rec_y = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/pos_rec_z.npy', 'rb') as f:
        pos_rec_z = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/orn_rec.npy', 'rb') as f:
        orn_rec = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/steering_angle_rec.npy', 'rb') as f:
        steering_angle_rec = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/point_rec_x.npy', 'rb') as f:
        point_rec_x = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/point_rec_z.npy', 'rb') as f:
        point_rec_z = np.load(f)


        #Read Lateral Dynamics control states

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/e1_rec.npy', 'rb') as f:
                e1_rec = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/e1_dot_rec.npy', 'rb') as f:
                e1_dot_rec =np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/e2_rec.npy', 'rb') as f:
                e2_rec = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/read_plots/e2_dot_rec.npy', 'rb') as f:
                e2_dot_rec = np.load(f)        


    #Reference Trajectories
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/pos_rec_x.npy', 'rb') as f:
        ref_pos_rec_x = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/pos_rec_y.npy', 'rb') as f:
        ref_pos_rec_y = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/pos_rec_z.npy', 'rb') as f:
        ref_pos_rec_z = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/orn_rec.npy', 'rb') as f:
        ref_orn_rec = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/steering_angle_rec.npy', 'rb') as f:
        ref_steering_angle_rec = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/point_rec_x.npy', 'rb') as f:
        ref_point_rec_x = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/read_data/point_rec_z.npy', 'rb') as f:
        ref_point_rec_z = np.load(f)
    time_rec = [0.0]
    time = 0.0
    time_step= 1/25 #controller frequency
    for idx in range(0,len(pos_rec_y)):
           
            time = idx*time_step
            time_rec.append(time)



  
    plt.figure(1)
    
    ax1 = plt.subplot(511)
    plt.plot(time_rec[0:len(pos_rec_x)],pos_rec_x, color='red', label='$x$')
    plt.plot(time_rec[0:len(pos_rec_x)],ref_pos_rec_x, color='black', label='$x_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Position  $x(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()

    ax2 = plt.subplot(512)
    plt.plot(time_rec[0:len(pos_rec_y)],pos_rec_y, color='red', label='$y$')
    plt.plot(time_rec[0:len(pos_rec_y)],ref_pos_rec_y, color='black', label='$y_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Position $y(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()

    ax3 = plt.subplot(513)
    plt.plot(time_rec[0:len(pos_rec_z)],pos_rec_z, color='red', label='$z$')
    plt.plot(time_rec[0:len(pos_rec_z)],ref_pos_rec_z, color='black', label='$z_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Position $z(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()


    ax4 = plt.subplot(514)
    plt.plot(time_rec[0:len(orn_rec)],orn_rec, color='red', label='$\Theta$')
    plt.plot(time_rec[0:len(orn_rec)],ref_orn_rec, color='black', label='$\Theta_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Heading $\Theta^{\circ}$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()

    ax5 = plt.subplot(515)
    #plt.plot(time_rec[0:len(orn_rec)],orn_rec, color='red', label='$\Theta$')
    #plt.plot(time_rec[0:len(orn_rec)],ref_orn_rec, color='black', label='$\Theta_{ref}$')
    plt.plot(time_rec[0:len(ref_steering_angle_rec)],ref_steering_angle_rec, color='black', label='$\delta_{ref}$')
    plt.plot(time_rec[0:len(steering_angle_rec)],steering_angle_rec, color='red', label='$\delta$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Steering Angle $\delta^{\circ}$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()


    ###########################################################################################################
    plt.figure(2)
    
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
    plt.legend()

    ###########################################################################################################

    plt.figure(3)
    
    ax1 = plt.subplot(411)
    plt.plot(time_rec[0:len(e1_rec)],e1_rec, color='red', label='$x$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('  $e_1 (m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()

    ax1 = plt.subplot(412)
    plt.plot(time_rec[0:len(e1_dot_rec)],e1_dot_rec, color='red', label='$x$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('  $\dot{e1} (m/s)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()

    ax1 = plt.subplot(413)
    plt.plot(time_rec[0:len(e2_rec)],e2_rec, color='red', label='$x$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('  $e_2 (radians)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()


    ax1 = plt.subplot(414)
    plt.plot(time_rec[0:len(e2_dot_rec)],e2_dot_rec, color='red', label='$x$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('  $\dot{e2} (radians/s)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()

    

###########################################################################################################

    plt.figure(4)
    
    #ax1 = plt.subplot(411)
    plt.plot(pos_rec_z,pos_rec_x, color='red', label='$traj_{act}$')
    plt.plot(ref_pos_rec_z,ref_pos_rec_x, color='black', label='$traj_{ref}$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('Position $z$')
    plt.ylabel('Position  $x$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()








    plt.show()





if __name__ == '__main__':
    main()
    print("Finished")
    exit()