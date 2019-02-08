import time
import sys
import numpy as np
import matplotlib.pyplot as plt
import urx
import bezier
from pyswarm import pso


iteration_count = 1
figure, aaa = plt.subplots(3, 2)
# aaa = axarr.flatten()
#plt.axis([0, 10, 0, 1])
number_of_optimized_points = 5
ex_time = 3.0
samples_per_second = 10.0
dt = 1.0 / 125.0 * 10.0
N = ex_time / dt   # funguje do cca 15 s , potom je asi prilis velka velikost posilaneho programu

time_axis = np.linspace(0.0, ex_time, int(N) , dtype='float')
time_axis_control_points = np.linspace(0.0, ex_time, int(number_of_optimized_points + 2) , dtype='float')
time_axis_control_points[1] = 0.2
time_axis_control_points[-2] = ex_time - 0.2
time_axis_control_points[2] = 0.8
time_axis_control_points[-3] = 2.2


data_to_save = []

# plt.ion()

def trajektorie(body) :
    global iteration_count, data_to_save
    body = body.reshape((number_of_optimized_points, 6))
    # jjA = np.array(rob.getj())

    #### zleva doprava
    jjA = np.array([ 3.60267305,  0.18439186, -1.32002622,  0.07805479,  0.38413787, -0.67564518])  ## A
    jjB = np.array([ 2.05382657, -0.16434461, -0.60126812, -2.27893335, -0.77898008, -0.66919119])  ## B
    #### zdola nahoru, zhora dolu
    # jjB = np.array([3.099792242050171, 0.20784461498260498, -0.5279377142535608, -2.0288899580584925, -0.9865768591510218, -0.6691191832171839])  ## C
    # jjA = np.array([3.1011946201324463, -1.510937515889303, 0.008538246154785156, -1.552093807850973, -1.5738137404071253, -0.669143025075094])   ## D
    #### zdola nahoru, zhora dolu
    # jjA = np.array([3.099792242050171 + 1.088, 0.0, -0.5279377142535608, -2.0288899580584925, -0.9865768591510218, -0.6691191832171839])  ## C
    # jjB = np.array([0.38893118500709534 + 1.088, 0.0, -0.6022990385638636, -2.1837270895587366, -1.3906963507281702, -0.6691792646991175])   ## E


    jjj = np.zeros((6, int(N)  ))

    s_vals = np.linspace(0.0, 1.0, int(N) , dtype='float')
    control_points = np.zeros( (6, number_of_optimized_points + 2 ) )

    for i in range(6) :
        # control_points = list( [jjA[i]] +   list( (jjA[i] + jjB[i])/2.0  + 0.5* body[:,i]) + [jjB[i]] )
        pp = list( [jjA[i]] + [jjA[i] + body[0,i]] + list( (jjA[i] + jjB[i])/2.0  + body[1:-1,i]) + [jjB[i] + body[-1,i]] + [jjB[i]] )
        control_points[i, :] = pp
        nodes = np.asfortranarray([ time_axis_control_points , pp ], dtype='float')
        curve = bezier.Curve(nodes, degree=2)
        jjj[i,:] = curve.evaluate_multi(s_vals)[1,:]

    figure.suptitle('Iteration number {}'.format(iteration_count), fontsize=16)
    # for ii, aa in enumerate(aaa.flat) :
    #     aa.cla()
    #     aa.set_title("Joint {}".format(ii))
    #     aa.plot(time_axis_control_points ,control_points[ii,:],"x")
    #     aa.plot(time_axis,jjj[ii,:],":")
    #     aa.plot(time_axis_real,measured_trajectory[ii,:])
    #     aa.axis( ymin=-4.0 , ymax=4.0)
    # plt.pause(0.01)
    try :
        rob.movej(jjA,acc=1.5, vel=1.5, )
    except :
        aa = raw_input("Countinue?")
        rob.movej(jjA,acc=1.5, vel=1.5, )
    time.sleep(0.1)

    program = """def trajektorie():\n"""
    program += """\ttextmsg("START")\n"""

    for i in range(int(N) ) :
        program +="""\tservoj({}, t={}, lookahead_time = 0.05 , gain = 300)\n""".format( list(jjj[:,i]) , dt )
        # program +="""\tsleep(0.008)\n"""
    program += "end\n"

    # print program
    rob.send_program(program)

    energy = 0.0
    power = 0.0
    measured_trajectory = []
    time_axis_real = []
    traj = []

    j0 = rob.rtmon.get_all_data()['qActual']
    while True :
        j = rob.rtmon.get_all_data()['qActual']
        if np.linalg.norm(np.array(j)-np.array(j0) ) > 1.0e-3 :
            break
        j0 = j

    data = rob.rtmon.get_all_data()
    t0 = data['timestamp']
    # for i in range(int(N) ) :
    # for i in range( int((ex_time+0.0)/dt) ) :
    while True:
        pass
        data = rob.rtmon.get_all_data()
        power = np.dot( np.abs(data['currents']), np.abs(data['voltages']) )
        energy += power * dt
        measured_trajectory += [data['qActual']]
        time_axis_real += [data['timestamp'] - t0]
        traj += [ [data['timestamp'] - t0] + list(data['qActual']) ]

        print "QA ", data['qActual']
        print [ [data['timestamp'] - t0] + list(data['qActual']) ]

        if data['timestamp'] - t0 >= ex_time:
            break
        time.sleep(dt)

    measured_trajectory = np.array(measured_trajectory).T
    time_axis_real = np.array(time_axis_real)

    for ii, aa in enumerate(aaa.flat) :
        aa.cla()
        aa.set_title("Joint {}".format(ii))
        aa.plot(time_axis_control_points ,control_points[ii,:],"x")
        aa.plot(time_axis_real,measured_trajectory[ii,:])
        aa.plot(time_axis,jjj[ii,:],":")
        # aa.axis(xmin=-0.2, xmax=ex_time+0.2, ymin=-4.0 , ymax=4.0)
        aa.axis(xmin=-0.2, xmax=ex_time+0.2, ymin=(jjA[ii]+jjB[ii])/2.0-2.0 , ymax=(jjA[ii]+jjB[ii])/2.0+2.0 )
    # plt.show()
    plt.pause(0.01)

    # plt.pause(ex_time + 0.5)
    # time.sleep(16.0)
    iteration_count += 1
    print iteration_count, " - ", energy

    control_points = list( control_points.reshape( (number_of_optimized_points+2) * 6 ) )
    data_to_save += [ [energy] + control_points]
    data = np.array(data_to_save)
    np.save("./data_bezier_new_02",data)
    np.save("./data_bezier/trajs/traj_02_{}".format( iteration_count ) , np.array(traj) )

    return energy

rob = urx.Robot("192.168.1.10", use_rt=True)

lb = -np.ones(6*number_of_optimized_points)
lb[:6] *= 0.2
lb[-6:] *= 0.1
ub = np.ones(6*number_of_optimized_points)
ub[:6] *= 0.2
ub[-6:] *= 0.1
ub[1::6] = 0.0


xopt, fopt = pso(trajektorie, lb, ub, swarmsize=30, maxiter=230, debug=True )
