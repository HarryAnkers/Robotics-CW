import numpy as np
import matplotlib.pyplot as plt


def plotgraph(filename):
    with open(filename) as f:
        lines = f.readlines()
        t = [float(line.split()[0]) for line in lines]
        leftrefangle = [float(line.split()[1]) for line in lines]
        leftangle = [float(line.split()[2]) for line in lines]
        rightrefangle = [float(line.split()[3]) for line in lines]
        rightangle = [float(line.split()[4]) for line in lines]

    fig = plt.figure()

    ax1 = fig.add_subplot(221)

    ax1.set_title("Left motor before tuning")
    ax1.set_xlabel('Time(ns)')
    ax1.set_ylabel('Angle(radian)')

    ax1.plot(t,leftrefangle, c='r', label='Reference angle')
    ax1.plot(t,leftangle, c='b', label='Angle')

    # ax1.set_xticks(np.arange(min(t), max(t) + 1, 0.5))

    leg = ax1.legend()

    ax2 = fig.add_subplot(223)

    ax2.set_title("Right motor before tuning")
    ax2.set_xlabel('Time(ns)')
    ax2.set_ylabel('Angle(radian)')

    ax2.plot(t,rightrefangle, c='r', label='Reference angle')
    ax2.plot(t,rightangle, c='b', label='Angle')

    leg = ax2.legend()


    ax3 = fig.add_subplot(222)

    ax3.set_title("Error in Left Motor")
    ax3.set_xlabel('Time(ns)')
    ax3.set_ylabel('Angle(radian)')

    ax3.plot(t,[x1 - x2 for (x1, x2) in zip(leftrefangle, leftangle)], c='r', label='Error in Left')

    leg=ax3.legend()

    ax4 = fig.add_subplot(224)

    ax4.set_title("Error in Right Motor")
    ax4.set_xlabel('Time(ns)')
    ax4.set_ylabel('Angle(radian)')

    ax4.plot(t,[x1 - x2 for (x1, x2) in zip(rightrefangle, rightangle)], c='r', label='Error in Right')

    leg=ax4.legend()




    plt.show()
    plt.savefig('./graphs/{}.png'.format(filename[filename.find("/")+1:filename.find(".")]))
