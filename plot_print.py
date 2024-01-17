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
 
    with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/pos_rec_x.npy', 'rb') as f:
        pos_rec_x = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/pos_rec_y.npy', 'rb') as f:
        pos_rec_y = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/pos_rec_z.npy', 'rb') as f:
        pos_rec_z = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/orn_rec.npy', 'rb') as f:
        orn_rec = np.load(f)
    
    with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/steering_angle_rec.npy', 'rb') as f:
        steering_angle_rec = np.load(f)

    with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/point_rec_x.npy', 'rb') as f:
        point_rec_x = np.load(f)
    with open('/home/mukhe027/workspace/f1tenth_bullet/data_collection/point_rec_z.npy', 'rb') as f:
        point_rec_z = np.load(f)
    
    time_rec = [0.0]
    time = 0.0
    time_step= 1/25 #controller frequency
    for idx in range(0,len(pos_rec_y)):
           
            time = idx*time_step
            time_rec.append(time)



  
    plt.figure(1)
    
    ax1 = plt.subplot(511)
    plt.plot(time_rec[0:len(pos_rec_x)],pos_rec_x, color='red', label='$x$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Position  $x(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()

    ax2 = plt.subplot(512)
    plt.plot(time_rec[0:len(pos_rec_y)],pos_rec_y, color='black', label='$y$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Position $y(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()

    ax3 = plt.subplot(513)
    plt.plot(time_rec[0:len(pos_rec_z)],pos_rec_z, color='black', label='$z$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Position $z(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()


    ax4 = plt.subplot(514)
    plt.plot(time_rec[0:len(orn_rec)],orn_rec, color='black', label='$\Theta$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Heading $\Theta^{\circ}$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()


    ax5 = plt.subplot(515)
    plt.plot(time_rec[0:len(steering_angle_rec)],steering_angle_rec, color='black', label='$\delta$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Steering Angle  $\delta^{\circ}$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()


    ###########################################################################################################
    plt.figure(2)
    
    ax1 = plt.subplot(211)
    plt.plot(time_rec[0:len(point_rec_x)],point_rec_x, color='red', label='$x$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Position Lidar Point  $x(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()

    ax2 = plt.subplot(212)
    plt.plot(time_rec[0:len(point_rec_z)],point_rec_z, color='black', label='$z$')
    #plt.plot(ref_pos_rec, color='black', linestyle='--', label='$x_{rm}(t)$')
    plt.grid(True)
    plt.xlabel('time (s)')
    plt.ylabel('Position Lidar Point $z(m)$')
    #plt.title('Deep-MRAC with $\\nu_{ad}=W^{T}\phi^\sigma_{n}(x)$')
    plt.legend()


    


    ###########################################################################################################

    plt.figure(3)
    
    #ax1 = plt.subplot(411)
    plt.plot(pos_rec_z,pos_rec_x, color='red', label='traj')
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