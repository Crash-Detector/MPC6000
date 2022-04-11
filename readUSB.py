import serial.tools.list_ports as port_list
ports = list(port_list.comports())
print("live ports: ")
for p in ports:
    print (p);

import datetime as dt
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import math

DATA_POINT=8
TIME_SPAN=1*10

# Create figure for plotting
xs = list(range(DATA_POINT*TIME_SPAN))
gx = [0] * DATA_POINT*TIME_SPAN
gy = [0] * DATA_POINT*TIME_SPAN
gz = [0] * DATA_POINT*TIME_SPAN
g = [0] * DATA_POINT*TIME_SPAN
accx = [0] * DATA_POINT*TIME_SPAN
accy = [0] * DATA_POINT*TIME_SPAN
accz = [0] * DATA_POINT*TIME_SPAN
acc = [0] * DATA_POINT*TIME_SPAN
fallwin = [DATA_POINT*TIME_SPAN, DATA_POINT*TIME_SPAN]
fall_detected = [0] *DATA_POINT*TIME_SPAN
fall = 0

# to run GUI event loop
plt.ion()
 
# here we are creating sub plots
figure = plt.figure()
acc_plt = figure.add_subplot(3,1,1)
acc_linex, = acc_plt.plot(xs, accx, label="Acc x", linewidth=0.5)
acc_liney, = acc_plt.plot(xs, accy, label="Acc y", linewidth=0.5)
acc_linez, = acc_plt.plot(xs, accz, label="Acc z", linewidth=0.5)
acc_line, = acc_plt.plot(xs, acc, label="Acc", linewidth=1.0)
span_fallwin = acc_plt.axvspan(fallwin[0], fallwin[1], facecolor='b', alpha=0.2)
acc_plt.axhline(y=0.3, label="LFT", linewidth=0.5)
acc_plt.axhline(y=2.77, label="UFT", linewidth=0.5)
fall_line = acc_plt.axvline(x=fall, label="FALL", linewidth=1, color="orange")
gyr = figure.add_subplot(3,1,2)
gyr_linex, = gyr.plot(xs, gx, label="Gyr x", linewidth=0.5)
gyr_liney, = gyr.plot(xs, gy, label="Gyr y", linewidth=0.5)
gyr_linez, = gyr.plot(xs, gz, label="Gyr z", linewidth=0.5)
gyr_line, = gyr.plot(xs, g, label="Gyr", linewidth=1)
gyr.axhline(y=254.4, label="UFT", linewidth=0.5)

fall_graph = figure.add_subplot(3,1,3)
fall_output, = fall_graph.plot(xs, fall_detected, linewidth = 1)

acc_plt.set_ylim([-8, 8])
gyr.set_ylim([-1000, 1000])
fall_graph.set_ylim([-0.5, 1.5])


update_count = 0
# This function is called periodically from FuncAnimation
def update_info(mpu_data):
    if len(mpu_data) < 6:
        return
    # Add x and y to lists
    global xs, accx, accy, accz, acc, gx, gy, gz, g, fallwin, fall, fall_detected
    global update_count
    
    accx.append(mpu_data[0])
    accy.append(mpu_data[1])
    accz.append(mpu_data[2])
    acc.append(math.sqrt(mpu_data[0]**2 + mpu_data[1]**2 + mpu_data[2]**2))
    gx.append(mpu_data[3])
    gy.append(mpu_data[4])
    gz.append(mpu_data[5])
    g.append(math.sqrt(mpu_data[3]**2 + mpu_data[4]**2 + mpu_data[5]**2))
    fall_detected.append(mpu_data[7])
    fallwin[0] -= 1
    fallwin[1] -= 1
    fall -= 1
    # rewind! 
    if (fallwin[1] <= 0):
        fallwin[0] = DATA_POINT*TIME_SPAN
        fallwin[1] = DATA_POINT*TIME_SPAN
        
    if mpu_data[6] >= 0.5:
        fallwin[1] = DATA_POINT*TIME_SPAN
    if (mpu_data[6] >= 39):
        fallwin[1] = DATA_POINT*TIME_SPAN
        fallwin[0] = DATA_POINT*TIME_SPAN

    if (mpu_data[7] >= 0.5 and fall <= 0):
        fall = DATA_POINT*TIME_SPAN

    
    update_count += 1
    if update_count >= DATA_POINT:
        update_count = 0
        update_graph()
        

def update_graph():
    global xs, accx, accy, accz, acc, gx, gy, gz, g, fallwin, fall, fall_detected
    global acc_linex, acc_liney, acc_linez, acc_line, gyr_linex, gyr_liney, gyr_linez, gyr_line, span_fallwin, fall_line, fall_output

    # Limit x and y lists to 20 items
    accx = accx[-DATA_POINT*TIME_SPAN:]
    accy = accy[-DATA_POINT*TIME_SPAN:]
    accz = accz[-DATA_POINT*TIME_SPAN:]
    acc = acc[-DATA_POINT*TIME_SPAN:]
    gx = gx[-DATA_POINT*TIME_SPAN:]
    gy = gy[-DATA_POINT*TIME_SPAN:]
    gz = gz[-DATA_POINT*TIME_SPAN:]
    g = g[-DATA_POINT*TIME_SPAN:]
    fall_detected = fall_detected[-DATA_POINT*TIME_SPAN:]

    # Draw x and y lists
    acc_linex.set_ydata(accx)
    acc_liney.set_ydata(accy)
    acc_linez.set_ydata(accz)
    acc_line.set_ydata(acc)
    span_fallwin.set_xy([[fallwin[0], 0], [fallwin[0], 1], [
                        fallwin[1], 1], [fallwin[1], 0], [fallwin[0], 0]])
    fall_line.set_xdata(fall)

    gyr_linex.set_ydata(gx)
    gyr_liney.set_ydata(gy)
    gyr_linez.set_ydata(gz)
    gyr_line.set_ydata(g)    
    # gyrtick_line.set_xdata(gyrtick)
    
    fall_output.set_ydata(fall_detected)
    
    
    # plt.show()
    figure.canvas.draw()
    figure.canvas.flush_events()

    # Format plot
    # plt.pause(0.1)
    

import serial, sys
port = "COM4"
print("listening on port COM4")
baudrate = 115200
ser = serial.Serial(port,baudrate,timeout=1)
line = bytes()
while True:
    mpu_data = []
    try:    
        line = ser.readline()
        mpu_data = [float(x) for x in line.decode().split(",")]
    except ValueError:
        print("ValueError")
        print(line)
    update_info(mpu_data)
