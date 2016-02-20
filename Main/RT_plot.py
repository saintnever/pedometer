import numpy as np   
from matplotlib import pyplot as plt   
from matplotlib import animation   
  
# first set up the figure, the axis, and the plot element we want to animate   
fig = plt.figure()
ax1 = fig.add_subplot(3,1,1,xlim=(0, 2), ylim=(-4, 4))
ax2 = fig.add_subplot(3,1,2,xlim=(0, 2), ylim=(-4, 4))
ax3 = fig.add_subplot(3,1,3,xlim=(0, 2), ylim=(-4, 4))
line, = ax1.plot([], [], lw=2)  
line2, = ax2.plot([], [], lw=2)  
line3, = ax3.plot([], [], lw=2)  
def plot_init():  
  line.set_data([], [])  
  line2.set_data([], [])  
  line3.set_data([], [])  
  return line,line2,line3

# animation function.  this is called sequentially   
def animate(i):

  x = np.linspace(0, 2, 100)   
  y = np.sin(2 * np.pi * (x - 0.01 * i))  
  line.set_data(x, y)	  


  x2 = np.linspace(0, 2, 100)   
  y2 = np.cos(2 * np.pi * (x2 - 0.01 * i))* np.sin(2 * np.pi * (x2 - 0.01 * i))  
  line2.set_data(x2, y2)   
  
  x3 = np.linspace(0, 2, 100)   
  y3 = np.cos(2 * np.pi * (x3 - 0.01 * i))* np.sin(2 * np.pi * (x3 - 0.01 * i))  
  line3.set_data(x3, y3)   
  return line,line2,line3

anim1=animation.FuncAnimation(fig, animate, init_func=plot_init,  frames=50, interval=10)  
