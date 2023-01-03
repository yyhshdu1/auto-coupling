import copy
import os
import numpy as np
import serial
import matplotlib.pyplot as plt
import time as time
from New_Focus_8742 import Controller
from labjack import ljm                       # labjack
import LabJackAnalog as LJA


if __name__ == "__main__":
    # test
    import math
    import numpy as np
    import matplotlib.pyplot as plt
    print('\n\n')
    print('#'*80)
    print('#\tPython controller for NewFocus Picomotor Controller')
    print('#'*80)
    print('\n')

    idProduct = '0x4000'
    idVendor = '0x104d'

    if not (idProduct or idVendor):
        print('Run the following command in a new terminal window:')
        print('\t$ system_profiler SPUSBDataType\n')
        print('Enter Product ID:')
        idProduct = input('> ')
        print('Enter Vendor ID:')
        idVendor = input('> ')
        print('\n')

    # convert hex value in string to hex value
    idProduct = int(idProduct, 16)
    idVendor = int(idVendor, 16)

# Initialize the controller
controller = Controller(idProduct=idProduct, idVendor=idVendor)
controller.show_command()

# Initialize the LabJack
# first two lines open and close the LJ, to get rid of errors.
lj = LJA.LabJackAnalog(identifier="ANY")
lj.close()
lj = LJA.LabJackAnalog(identifier="ANY")
handle = ljm.openS("ANY", "ANY", "ANY")

print('684_labjack loaded')

# Use AIN2 channel to read the voltage from photodetector
input_channel = 2

# Set initial coupling power and the threshold for re-coupling power
# initial coupling power(in watt), if use photo detector this should be voltage
P_max = 3.14
P_thr = P_max*0.85                      # Threshold for start re-coupling
time_start = time.time()                # record the starting time

# Set Nelder-Mead function parameters
step = 50                               # initial step
V_int = P_max
no_improve_thr = 0.9
V_goal = V_int * no_improve_thr
no_improv_break = 3
max_iter = 100
alpha = 1.  # Reflection param
gamma = 2.  # Expansion param
rho = -0.5  # Contraction param
sigma = 0.5  # Reduction param
'''
    @param V_int(float): initial voltage read from photo detector
    @param step (float): look-around radius in initial step
    @no_improv_thr,  no_improv_break (float, int): break after no_improv_break iterations with
            an improvement lower than no_improv_thr
    @max_iter (int): always break after this number of iterations.
            Set it to 0 to loop indefinitely.
    @alpha, gamma, rho, sigma (floats): parameters of the algorithm
            (see Wikipedia page for reference)
    return: tuple (best parameter array, best score)
'''

# Start Nelder-Mead loop
# Set the current position as home position for all axis
controller.command("1DH")
controller.command("2DH")
controller.command("3DH")
controller.command("4DH")

# Start all four axis at home position
x_start = np.array([0., 0., 0., 0.])
# record the starting time
time_start = time.time()
# init
dim = len(x_start)
##
# read the inverse first power
prev_best = -1*lj.analog_in(input_channel)
no_improv = 0
res = [[x_start, prev_best]]
best_his = [prev_best]
print("the initial coupling efficiency is: "+str(-prev_best / V_int))
x_his = []

# get 4 test points, with the (0,0,0,0) one we get 5 initial test points
for i in range(dim):
    x = copy.copy(x_start)
    x[i] = x[i] + step
    for j in range(1, dim+1):
        # move the current axis to test position
        controller.command(str(j)+"PA"+str(x[j-1]))
        time.sleep(0.5)
    score = -1*lj.analog_in(input_channel)  # read the inverse testing power
    res.append([x, score])

# simplex iter
iters = 0
while True:
    # order
    # sort by power from minimum to maximum
    res.sort(key=lambda x: x[1])
    # the mimimum power within 5 testing points
    best = res[0][1]
    # position of the miminum point
    x_best = res[0][0]
    best_his.append(best)
    x_his.append(x_best)
    print('...best so far:', -res[0][1] / V_int)

    # break after max_iter
    if max_iter and iters >= max_iter:
        break
    iters += 1

    # break if all no_improv_break iterations(10) satisfy the goal efficiency
    if best > -1 * V_int * no_improve_thr:                   # goal efficiency: 0.9 V_int
        no_improv = 0
        prev_best = best
    else:
        no_improv += 1
    if no_improv >= no_improv_break:
        break

    # centroid                                            #calculate the center of all points except the maximum one
    x0 = [0.] * dim
    for tup in res[:-1]:
        for i, c in enumerate(tup[0]):
            x0[i] += c / (len(res)-1)

    # reflection
    xr = x0 + alpha*(x0 - res[-1][0])
    for j in range(1, dim+1):
        # move the current axis to reflection position
        controller.command(str(j)+"PA"+str(xr[j-1]))
        time.sleep(0.5)
    # read the inverse power at reflection position
    rscore = -1*lj.analog_in(input_channel)
    # if new power lays between min and max, keep the new point
    if res[0][1] <= rscore < res[-2][1]:
        del res[-1]
        res.append([xr, rscore])
        continue

    # expansion
    if rscore < res[0][1]:
        xe = x0 + gamma*(x0 - res[-1][0])
        for j in range(1, dim+1):
            # move the current axis to expansion position
            controller.command(str(j)+"PA"+str(xe[j-1]))
            time.sleep(0.5)
        # read the inverse power at expansion position
        escore = -1*lj.analog_in(input_channel)
        if escore < rscore:                                  # if the new point is better, keep it
            del res[-1]
            res.append([xe, escore])
            continue
        # if the new point is worse, keep the original(reflection) one
        else:
            del res[-1]
            res.append([xr, rscore])
            continue

    # contraction
    xc = x0 + rho*(x0 - res[-1][0])
    for j in range(1, dim+1):
        # move the current axis to contraction position
        controller.command(str(j)+"PA"+str(xc[j-1]))
        time.sleep(0.5)
    # read the inverse power at contraction position
    cscore = -1*lj.analog_in(input_channel)
    # if the new point is better, keep it
    if cscore < res[-1][1]:
        del res[-1]
        res.append([xc, cscore])
        continue

    # reduction
    x1 = res[0][0]
    nres = []
    for tup in res:
        redx = x1 + sigma*(tup[0] - x1)
        for j in range(1, dim+1):
            # move the current axis to reduction position to every point
            controller.command(str(j)+"PA"+str(redx[j-1]))
            time.sleep(0.5)
        # read the inverse power at reduction position to every point
        score = -1*lj.analog_in(input_channel)
        nres.append([redx, score])
    res = nres

# record the ending time
time_end = time.time()
# sort by power from minimum to maximum
res.sort(key=lambda x: x[1])
print(res)
# position of the miminum point
x_best_final = res[0][0]
for j in range(1, dim+1):
    # move the current axis to final position to every point
    controller.command(str(j)+"PA"+str(x_best_final[j-1]))
    time.sleep(0.5)

score_final = -1*lj.analog_in(input_channel)
print("the final efficiency is:", -score_final/V_int)
# Set the current position as home position for all axis
controller.command("1DH")
controller.command("2DH")
controller.command("3DH")
controller.command("4DH")
# plot
time_run = np.linspace(0, time_end - time_start, len(best_his))
N_C = -1*np.array(best_his)/V_int
plt.plot(time_run, N_C, 'b.-')
plt.axhline(y=no_improve_thr, color='r', linestyle='-')
plt.ylabel("normalized coupling efficiency")
plt.xlabel("time/s")
plt.show()

# save scan data

full_folder = 'C:\Chicago RA\\CeNTREX\\auto alignment\\Nelder-Mead method'
# os.mkdir(full_folder)
filename_data = full_folder + "\\data10.txt"
#np.savetxt(filename, np.transpose([x_forward, B_forward]), header = "x \t B_for[V]")
np.savetxt(filename_data, N_C)
filename_time = full_folder + "\\time10.txt"
np.savetxt(filename_time, time_run)
