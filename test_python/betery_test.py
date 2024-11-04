import numpy as np
import matplotlib.pyplot as plt
battery_Low_pass_filter_data11 = np.loadtxt('battery_Low_pass_filter_data11.text')
battery_Median_Filter_data50 = np.loadtxt('battery_Median_Filter_data50.text')
battery_Moving_Average_Filter_data50 = np.loadtxt('battery_Moving_Average_Filter_data50.text')
battery_Kalman_Filter_dataQ00001_R1 = np.loadtxt('battery_Kalman_Filter_dataQ00001_R1.text')
#plt.scatter(turn_i, udata)
plt.xlabel('t')
plt.ylabel('battery_filter_data')
line1, = plt.plot(battery_Low_pass_filter_data11,'g-', color='g')
line2, = plt.plot(battery_Median_Filter_data50,'r-', color='r')
line3, = plt.plot(battery_Moving_Average_Filter_data50,'b-', color='b')
line4, = plt.plot(battery_Kalman_Filter_dataQ00001_R1,'y-', color='y')

plt.legend(handles=[line1,line2,line3,line4],labels=["battery_Low_pass_filter_data11","battery_Median_Filter_data50","battery_Moving_Average_Filter_data50","battery_Kalman_Filter_dataQ00001_R1"],loc="best",fontsize=6)
plt.show()
