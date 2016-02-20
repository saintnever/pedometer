import serial 
import numpy as np   
from matplotlib import pyplot as plt   
from matplotlib import animation   
  
# first set up the figure, the axis, and the plot element we want to animate   
fig = plt.figure()
ax1 = fig.add_subplot(3,1,1,xlim=(0, 99), ylim=(-10000, 10000))
ax2 = fig.add_subplot(3,1,2,xlim=(0, 99), ylim=(-10000, 10000))
ax3 = fig.add_subplot(3,1,3,xlim=(0, 99), ylim=(-10000, 10000))
y = list(np.linspace(0, 0, 100))
y2 = list(np.linspace(0, 0, 100))
y3 = list(np.linspace(0, 0, 100))
y_s = list(np.linspace(0, 0, 100))
y2_s = list(np.linspace(0, 0, 100))
y3_s = list(np.linspace(0, 0, 100))
line, = ax1.plot(y, lw=2)  
line2, = ax2.plot(y2, lw=2)  
line3, = ax3.plot(y3, lw=2)  
def plot_init():  
  line.set_data([], [])  
  line2.set_data([], [])  
  line3.set_data([], [])  
  return line,line2,line3
  
# animation function.  this is called sequentially   
def animate(data):

  # x = np.linspace(0, 2, 100)   
  y.insert(0,data[0])
  y.pop()
  linearSmooth3(y,y_s,100)
  line.set_ydata(y_s)	  

  y2.insert(0,data[1])
  y2.pop()
  linearSmooth3(y2,y2_s,100)
  line2.set_ydata(y2_s)	 
  
  y3.insert(0,data[2])
  y3.pop()
  linearSmooth3(y3,y3_s,100)
  line3.set_ydata(y3_s)
  return line,line2,line3
  
def data_gen(): 
	# open and init port
	ser = serial.Serial()
	# Set port number to 3. Counting starts at 0
	ser.port = 6
	#check if the port is already opened
	if ser.isOpen():
		print("COM port is not available! ERROR!")
	#if the port is available, listen to it
	else:
		ser.open()
		ser.baudrate = 115200
		ser.rtscts = False
		ser.dsrdtr = False
		
		# nrf52 dev board
		# ser.baudrate = 38400
		# ser.rtscts = True
		# ser.dsrdtr = True
		
		while 1:
			raw_data = list(ser.readline())
			# temprary fix
			if(len(raw_data)<7):
				continue
			data_array = data_process(raw_data)
			# print("X:0x%4x, Y:0x%4x, Z:0x%4x" % (x_data,y_data,z_data))
			print("X:%.02f, Y:%.02f, Z:%.02f" % (data_array[0],data_array[1],data_array[2]))
			# if  data.isdigit():
				# data = int(data)
				# if data>=-4 and data<=4:
			yield data_array
					
def data_process(raw_data): 
	raw_data.pop()
	#format data
	if(len(raw_data)>6):
		if raw_data[0] == 0x00:
			data = raw_data[1:]
	else:
		data = raw_data
	x_axis = (data[1]<<8)+data[0]
	y_axis = (data[3]<<8)+data[2]
	z_axis = (data[5]<<8)+data[4]
	# print("X:0x%4x, Y:0x%4x, Z:0x%4x" % (x_axis,y_axis,z_axis))
	if((x_axis&0xF000) == 0xF000):
		x_axis = -(((-x_axis)&0x0FFF)*3.9)+950
	else:
		x_axis = (x_axis&0x0FFF)*3.9+500
	if(y_axis&0xF000 == 0xF000):
		y_axis = -(((-y_axis)&0x0FFF)*3.9)+800
	else:
		y_axis = (y_axis&0x0FFF)*3.9+400
	if(z_axis&0xF000 == 0xF000):
		z_axis = -(((-z_axis)&0x0FFF)*3.9)-5000
	else:
		z_axis = (z_axis&0x0FFF)*3.9-5000
	return x_axis,y_axis,z_axis
	
def linearSmooth3(data_in, data_out, n):
	if n<3:
		for i in range(0,n-1,1):
			data_out[i] = data_in[i]
	else:
		data_out[0] = (5*data_in[0]+2*data_in[1]-data_in[2])/6
		for i in range(1,n-2,1):
			data_out[i] = (data_in[i-1]+data_in[i]+data_in[i+1])/3
	data_out[n-1] = (5*data_in[n-1]+2*data_in[n-2]-data_in[n-3])/6
			
# data_gen()
anim1=animation.FuncAnimation(fig, animate, data_gen, interval=1) 		
plt.show() 		


