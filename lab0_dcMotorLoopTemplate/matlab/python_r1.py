#under construction!!!

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
from matplotlib import rc
import numpy as np
import random
import serial
import datetime
import re
import struct

BUFF_SIZE=200

#initialize serial port
ser = serial.Serial()
ser.port = 'COM4'#'COM59' #Arduino serial port
ser.baudrate = 115200 #500000 #115200
ser.timeout = 10 #specify timeout when using readline()
ser.open()
if ser.is_open==True:
	print("\nAll right, serial port now open. Configuration:\n")
	print(ser, "\n") #print serial parameters

#global vars
t_vector = []
ies = []
r_vector = []
angle_vector = []
u_vector = []
roll_vector = []
distance_vector = []
data = []
t = []
circ_buffer1 = []
circ_buffer2 = []
circ_buffer3 = []
circ_buffer4 = []
circ_buffer5 = []
circ_buffer6 = []
circ_buffer7 = []
debug_data1= []

#plot colors
color_1=(0.00,0.45,0.74)
color_2=(0.85,0.33,0.10)
color_3=(0.93,0.69,0.13)
color_4=(0.49,0.18,0.56)
color_5=(0.47,0.67,0.19)
color_6=(0.30,0.75,0.93)
color_7=(0.64,0.08,0.18)
color_grid=(0.8, 0.8, 0.8)



try:
    #figure 1: debug OSC data online
    plt.figure()
    plt.ion()
    plt.style.use("tableau-colorblind10")
    plt.xlabel('data points')
    plt.ylabel('OSC data')
    plt.plot(t_vector,r_vector, 'b-',label='r')
    plt.plot(t_vector,angle_vector, 'r-',label='angle')
    plt.plot(t_vector,u_vector, 'g-',label='u')
    plt.plot(t_vector,roll_vector, 'k-',label='roll')
    plt.plot(t_vector,distance_vector, 'm-',label='distance')
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys(),loc="upper left")
    plt.grid(color = color_grid, linestyle = '-', linewidth = 0.5)

    for i in range(200):
        line=ser.readline().decode('utf-8').rstrip()
        #print(line)
        if (line == "OSC"):
            osc_data=ser.readline().decode('utf-8').rstrip()
            osc_data_as_list=osc_data.split(',')
            #print(data_as_list)
            t_vector.append(i)
            #print(distance_vector)
            data0_vector.append(float(osc_data_as_list[0]))
            data1_vector.append(float(osc_data_as_list[1]))
            data2_vector.append(float(osc_data_as_list[2]))
            data3_vector.append(float(osc_data_as_list[3]))
            data4_vector.append(float(osc_data_as_list[4]))
            plt.plot(t_vector,r_vector, 'b-',label='t')
            plt.plot(t_vector,angle_vector, 'r-',label='data1')
            plt.plot(t_vector,u_vector, 'g-',label='data2')
            plt.plot(t_vector,roll_vector, 'k-',label='data3')
            plt.plot(t_vector,distance_vector, 'm-',label='data4')
            plt.xlim([i-50, i])
            #plt.ylim([-180, 1000])
            plt.autoscale(enable=True, axis='y')
            plt.show
            plt.pause(0.005)
        if (line == "DAT"):
            break

    for i in range(BUFF_SIZE):
        line=ser.readline().decode('utf-8').rstrip()
        #print(line)
        if (line == "DAT"):
            dat_data=ser.readline().decode('utf-8').rstrip()
            data.append(dat_data)
            if(i%50==0):
                print("adquired " + str(i) + " samples");



    #print(data)
    ser.close()

    #arrange received data
    for i in range(0,len(data)):
        #print(data[i])
        temp=data[i].split(",")
        t.append(float(temp[0])/1000)
        circ_buffer1.append(float(ord(temp[1])))
        circ_buffer2.append(float(ord(temp[2])))
        circ_buffer3.append(float(ord(temp[3])))
        circ_buffer4.append(float(ord(temp[4])))
        circ_buffer5.append(float(ord(temp[5])))
        circ_buffer6.append(float(ord(temp[6])))
        circ_buffer7.append(float(ord(temp[7])))
        debug_data1.append(float(temp[8]))

    min_t = min(t)
    pos_min_t = t.index(min_t)
    t=np.roll(t,-pos_min_t)
    circ_buffer1=np.roll(circ_buffer1,-pos_min_t)
    circ_buffer2=np.roll(circ_buffer2,-pos_min_t)
    circ_buffer3=np.roll(circ_buffer3,-pos_min_t)
    circ_buffer4=np.roll(circ_buffer4,-pos_min_t)
    circ_buffer5=np.roll(circ_buffer5,-pos_min_t)
    circ_buffer6=np.roll(circ_buffer6,-pos_min_t)
    circ_buffer7=np.roll(circ_buffer7,-pos_min_t)
    debug_data1=np.roll(debug_data1,-pos_min_t)

    #change FreeRTOS states to something more appealing in plots
    eRunning=0.0;
    eReady=1.0;
    eBlocked=2.0;
    eSuspended=3.0;
    eDeleted=4.0;
    eInvalid=5.0;

    val_old  = np.array([eRunning,eReady,eBlocked,eSuspended])
    val_new  = np.array([2.0, 1.0, 0.0, 2.5])
    d = dict(zip(val_old, val_new))
    circ_buffer1 = np.array([d.get(e, e) for e in circ_buffer1])
    circ_buffer2 = np.array([d.get(e, e) for e in circ_buffer2])
    circ_buffer3 = np.array([d.get(e, e) for e in circ_buffer3])
    circ_buffer4 = np.array([d.get(e, e) for e in circ_buffer4])
    circ_buffer5 = np.array([d.get(e, e) for e in circ_buffer5])
    circ_buffer6 = np.array([d.get(e, e) for e in circ_buffer6])
    circ_buffer7 = np.array([d.get(e, e) for e in circ_buffer7])

    #figure 2: debug DAT data offline
    plt.figure()
    plt.xlabel("t(s)")
    plt.ylabel("debug data")
    plt.grid(color = color_grid, linestyle = '-', linewidth = 0.5)
    plt.plot(t,debug_data1, 'b-',label='debug data')
    plt.legend()

    #figure 3: debug DAT task states offline
    fig=plt.figure()#figsize=(4,2))
    ax = fig.add_subplot(111)
    ax.set_yticks([0, 1, 2, 2.5,  5, 6, 7, 7.5,  10, 11, 12, 12.5,  15, 16, 17, 17.5,  20, 21, 22, 22.5,  25, 26, 27, 27.5, 30, 31, 32, 32.5])
    ax.set_yticklabels(['Blocked','Ready','Running','Suspended','Blocked','Ready','Running','Suspended','Blocked','Ready','Running','Suspended','Blocked','Ready','Running','Suspended','Blocked','Ready','Running','Suspended','Blocked','Ready','Running','Suspended','Blocked','Ready','Running','Suspended'],fontname="Arial", fontsize=8)
    ax.yaxis.label.set_size(40)
    plt.xlabel("t(s)")
    plt.grid(color = color_grid, linestyle = '-', linewidth = 0.5)

    #plt.stairs(circ_buffer7[0:-1]+30,t, color=color_7,baseline=None,label=r'$\tau_{7}$')
    #plt.stairs(circ_buffer6[0:-1]+25,t, color=color_6,baseline=None,label=r'$\tau_{6}$')
    #plt.stairs(circ_buffer5[0:-1]+20,t, color=color_5,baseline=None,label=r'$\tau_{5}$')
    plt.stairs(circ_buffer4[0:-1]+15,t, color=color_4,baseline=None,label=r'$\tau_{Supervision}$')
    plt.stairs(circ_buffer3[0:-1]+10,t, color=color_3,baseline=None,label=r'$\tau_{3}$')
    plt.stairs(circ_buffer2[0:-1]+5,t, color=color_2,baseline=None,label=r'$\tau_{2}$')
    plt.stairs(circ_buffer1[0:-1],t, color=color_1,baseline=None,label=r'$\tau_{1}$')

    ax.legend()


except serial.SerialException as e:
    print(e)
except TypeError as e:
    print(e)
else:
    print("Script finished...")
    plt.show(block=True) # block=True lets the window stay open at the end of the animation.
    #plt.show(block=False)






