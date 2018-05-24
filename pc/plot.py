import time
from matplotlib import pyplot as plt
import numpy as np
from collections import deque


def live_update_demo(blit = False):
    x = np.linspace(0, 50., num=100)
    # X,Y = np.meshgrid(x,x)

    fig = plt.figure()

    ax1 = fig.add_subplot(3, 1, 1)
    ax2 = fig.add_subplot(3, 1, 2)
    ax3 = fig.add_subplot(3, 1, 3)

    fig.canvas.draw()   # note that the first draw comes before setting data 

    # h1 = ax1.imshow(X, vmin=-1, vmax=1, interpolation="None", cmap="RdBu")

    h1, = ax1.plot(x, lw=2)

    # text1 = ax2.text(0.0,1.5, "")
    ax1.set_ylim([-0.6,0.6])

    # text1.set_text("heeelo")


    h2, = ax2.plot(x, lw=2)
    # text = ax2.text(0.8,1.5, "")
    ax2.set_ylim([-0.6,0.6])


    h3, = ax3.plot(x, lw=2)
    # text = ax2.text(0.8,1.5, "")
    ax3.set_ylim([-0.3,0.3])

    if blit:
        # cache the background
        axbackground = fig.canvas.copy_from_bbox(ax1.bbox)
        ax2background = fig.canvas.copy_from_bbox(ax2.bbox)
        ax3background = fig.canvas.copy_from_bbox(ax3.bbox)

    k=0.

    filePath = '/home/fardin/devel/ltzvisor/pc/cmake-build-debug/state.csv'
    lastLine = '--'
    time_vector = np.linspace(0,1,100)

    elevation = deque([0] * 100)
    pitch = deque([0] * 100)
    travel = deque([0] * 100)

    f = open(filePath,'w+')
    f.close()


    while(True):
        # h1.set_data(np.sin(X/3.+k)*np.cos(Y/3.+k))

        f = open(filePath,'r+')
        lines = f.readlines()

        if lines and lines[-1] != lastLine:
            lastLine = lines[-1]
            line = lastLine.split(',')

            elevation_sample = float(line[0])
            elevation.popleft()
            elevation.append(elevation_sample)

            pitch_sample = float(line[1])
            pitch.popleft()
            pitch.append(pitch_sample)

            travel_sample = float(line[2])
            travel.popleft()
            travel.append(travel_sample)

        h1.set_ydata(elevation)
        h2.set_ydata(pitch)
        h3.set_ydata(travel)
        # h2.set_ydata(np.sin(x/3.+k))
        #print tx
        k+=0.11
        if blit:
            # restore background
            fig.canvas.restore_region(axbackground)
            fig.canvas.restore_region(ax2background)
            fig.canvas.restore_region(ax3background)

            # redraw just the points
            ax1.draw_artist(h1)
            ax2.draw_artist(h2)
            ax3.draw_artist(h3)

            # fill in the axes rectangle
            fig.canvas.blit(ax1.bbox)
            fig.canvas.blit(ax2.bbox)
            fig.canvas.blit(ax3.bbox)
            # in this post http://bastibe.de/2013-05-30-speeding-up-matplotlib.html
            # it is mentionned that blit causes strong memory leakage. 
            # however, I did not observe that.

        else:
            # redraw everything
            fig.canvas.draw()
            fig.canvas.flush_events()


        plt.pause(0.001)
        #plt.pause calls canvas.draw(), as can be read here:
        #http://bastibe.de/2013-05-30-speeding-up-matplotlib.html
        #however with Qt4 (and TkAgg??) this is needed. It seems,using a different backend, 
        #one can avoid plt.pause() and gain even more speed.


live_update_demo(True) # 28 fps
