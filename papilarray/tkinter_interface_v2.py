#!/usr/bin/env python3

# Graphic user interface using tkinter to display data from the tactile sensors

import time
import tkinter
import rospy
from std_msgs.msg import *
from papillarray_ros_v2.msg import SensorState
from papillarray_ros_v2.srv import BiasRequest
import matplotlib.pyplot as plt
import numpy
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg)
# Implement the default Matplotlib key bindings.
from matplotlib.backend_bases import key_press_handler
from matplotlib.figure import Figure



class interface:

    def __init__(self):

        ## Listener

        self.sub = rospy.Subscriber('/hub_0/sensor_0', SensorState, self.callback)

        ## Reset previous bias

        rospy.ServiceProxy('/hub_0/send_bias_request', BiasRequest)()

        ## Variables

        self.msgZ = [0,0,0,0,0,0,0,0,0] # Vectors collecting the 9 force values along each axis
        self.msgY = [0,0,0,0,0,0,0,0,0]
        self.msgX = [0,0,0,0,0,0,0,0,0]
        self.msgGF = [0,0,0] # Vector for the 3 global force values
        self.dec = 65 # Space between the pillars on the drawing
        self.size = "1500x1000" # Main window size
        self.width = 30 # Column width
        self.buffer_size = 130 # Number of points to display on each graph, 150 for 20s, 120 for 15s, 80 for 10s
        self.timelist = numpy.linspace(-5, 0, self.buffer_size)
        self.graphX = [[0 for s in range(self.buffer_size)] for p in range (9)]
        self.graphY = [[0 for s in range(self.buffer_size)] for p in range (9)]
        self.graphZ = [[0 for s in range(self.buffer_size)] for p in range (9)]

        self.graphGX = [0 for s in range(self.buffer_size)]
        self.graphGY = [0 for s in range(self.buffer_size)]
        self.graphGZ = [0 for s in range(self.buffer_size)]

        self.arrowcoordX = [125, 125, 125, 190, 190, 190, 255, 255, 255]
        self.arrowcoordY = [215, 150, 85, 215, 150, 85, 215, 150, 85]

        
        
        ## Making the main window

        self.window = tkinter.Tk()
        self.window.title("Sensor data")
        self.window.geometry(self.size)
        self.window.resizable()

        ## Canvas for the sensor drawings

        self.canvas0 = tkinter.Canvas()
        #canvas1 = Canvas()

        label0 = tkinter.Label(self.window, text="Sensor 0", font = ("Arial Bold", 25))
        label0.grid(column=1, row=0)

        #label1 = Label(self.window, text="Sensor 1", font = ("Arial Bold", 25))
        #label1.grid(column=6, row=0)

        self.reset_text = tkinter.Label(self.window, text="")
        self.reset_text.grid(column=4, row=1)

        self.canvas0.create_rectangle(90,50,290,250, outline="grey", fill="light grey", width=2)

        self.circle_list = []

        for j in range(3):
            for i in range(3):
                self.circle_list.append(self.canvas0.create_oval(100+i*self.dec,60+j*self.dec,150+i*self.dec,110+j*self.dec, outline="black",fill="pink", width=2))
        points = [90,50,290,50,270,30,110,30]
        self.canvas0.create_polygon(points, outline = "grey", fill = "light grey", width = 2)

        # Display the canvas

        self.canvas0.grid(column=1, row=1)

        # Create a legend

        self.canvasT = tkinter.Canvas()
        self.canvasT.create_rectangle(125, 65, 275, 215, fill = 'white')
        self.canvasT.create_text(230, 100, text = 'X force', font = ('Arial', 12))
        self.canvasT.create_text(230, 140, text = 'Y force', font = ('Arial', 12))
        self.canvasT.create_text(230, 180, text = 'Z force', font = ('Arial', 12))
        self.canvasT.create_line(150, 100, 190, 100, fill = 'red', width = 2)
        self.canvasT.create_line(150, 140, 190, 140, fill = 'green', width = 2)
        self.canvasT.create_line(150, 180, 190, 180, fill = 'blue', width = 2)
        self.canvasT.grid(column=0, row=1)

        # Second sensor

        #canvas1.create_rectangle(90,50,290,250, outline="grey", fill="light grey", width=2)
        #for i in range(3):
            #for j in range(3):
                #canvas1.create_oval(100+i*self.dec,60+j*self.dec,150+i*self.dec,110+j*self.dec, outline="black",fill="pink", width=2)
        #points = [290,50,290,250,310,240,310,60]
        #canvas1.create_polygon(points, outline = "grey", fill = "light grey", width = 2)

        #canvas1.grid(column=6, row=1)

        ## Displaying values

        # First sensor 

        k = 0

        self.pillars = [tkinter.Label(self.window, text = 'Pillar 0', font = ("Arial", 14), width = self.width), 
                        tkinter.Label(self.window, text = 'Pillar 1', font = ("Arial", 14), width = self.width), 
                        tkinter.Label(self.window, text = 'Pillar 2', font = ("Arial", 14), width = self.width), 
                        tkinter.Label(self.window, text = 'Pillar 3', font = ("Arial", 14), width = self.width), 
                        tkinter.Label(self.window, text = 'Pillar 4', font = ("Arial", 14), width = self.width), 
                        tkinter.Label(self.window, text = 'Pillar 5', font = ("Arial", 14), width = self.width), 
                        tkinter.Label(self.window, text = 'Pillar 6', font = ("Arial", 14), width = self.width), 
                        tkinter.Label(self.window, text = 'Pillar 7', font = ("Arial", 14), width = self.width), 
                        tkinter.Label(self.window, text = 'Pillar 8', font = ("Arial", 14), width = self.width)]
        self.displayX = [tkinter.Label(self.window, text = self.msgX[0], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgX[1], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgX[2], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgX[3], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgX[4], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgX[5], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgX[6], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgX[7], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgX[8], font = ("Arial", 14))]
        self.displayY = [tkinter.Label(self.window, text = self.msgY[0], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgY[1], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgY[2], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgY[3], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgY[4], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgY[5], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgY[6], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgY[7], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgY[8], font = ("Arial", 14))]
        self.displayZ = [tkinter.Label(self.window, text = self.msgZ[0], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgZ[1], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgZ[2], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgZ[3], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgZ[4], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgZ[5], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgZ[6], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgZ[7], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgZ[8], font = ("Arial", 14))]
        for j in range(3):
            for i in range(3):
                self.pillars[k].grid(column=j, row=12-5*i)
                # Uncomment to display numerical force values for each pillar
                # /!\ you may need to resize figures for the interface to fit in the screen
                """ self.displayX[k].grid(column=j, row=14-5*i)
                self.displayY[k].grid(column=j, row=15-5*i)
                self.displayZ[k].grid(column=j, row=16-5*i) """
                k += 1 



        self.GFlabel = tkinter.Label(self.window, text = 'Global Forces', font = ("Arial", 14))
        self.GFlabel.grid(column=4, row=7)
        self.displayGF = [tkinter.Label(self.window, text = self.msgGF[0], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgGF[1], font = ("Arial", 14)), 
                         tkinter.Label(self.window, text = self.msgGF[2], font = ("Arial", 14))]


        # Uncommemt to display global force values as numbers    
        """ self.displayGF[0].grid(column=4, row=9)
        self.displayGF[1].grid(column=4, row=10)
        self.displayGF[2].grid(column=4, row=11) """

        ## Matplotlib graphs

        self.plots = []
        self.lines = []
        self.canvasG = []

        x = 0
        for j in range(3):
            for i in range (3):
                plt.figure()
                self.fig, self.ax= plt.subplots(figsize=(1.8, 1.8))
                self.plots.append([self.fig, self.ax])
                self.line, = self.ax.plot(self.timelist, self.graphX[x], color = 'red')
                self.lines.append(self.line)
                self.line, = self.ax.plot(self.timelist, self.graphY[x], color = 'green')
                self.lines.append(self.line)
                self.line, = self.ax.plot(self.timelist, self.graphZ[x], color = 'blue')
                self.lines.append(self.line)
                plt.ylim(-1, 10)
                plt.tick_params(axis='x', which='both', bottom=True, top=False, labelbottom=True, labelsize=8)
                plt.tick_params(axis='y', which='both', labelsize=8)
                plt.xticks([-5, -3.333, -1.667, 0], ['0s', '5s', '10s', '15s'])

                self.canvasG.append(FigureCanvasTkAgg(self.fig, master=self.window))
                self.canvasG[x].draw()
                self.canvasG[x].get_tk_widget().grid(column=j, row=13-5*i)
                x += 1

        
        plt.figure()
        self.figGF, self.axGF = plt.subplots(figsize=(1.8, 1.8))
        self.lineGFX, = self.axGF.plot(self.timelist, self.graphGX, color = 'red')
        self.lineGFY, = self.axGF.plot(self.timelist, self.graphGY, color = 'green')
        self.lineGFZ, = self.axGF.plot(self.timelist, self.graphGZ, color = 'blue')
        plt.ylim(-5, 20)
        plt.tick_params(axis='x', which='both', bottom=True, top=False, labelbottom=True, labelsize=8)
        plt.tick_params(axis='y', which='both', labelsize=8)
        plt.xticks([-5, -3.333, -1.667, 0], ['0s', '5s', '10s', '15s'])
        self.canvasGF = FigureCanvasTkAgg(self.figGF, master=self.window)
        self.canvasGF.draw()
        self.canvasGF.get_tk_widget().grid(column=4, row=8)



        # Make arrows
        self.arrowX = [None for i in range(18)]
        self.arrowY = [None for i in range(18)]
        self.fillcolor = [None for i in range(9)]
        for l in range(9):
            self.arrowX[l] = self.canvas0.create_line(self.arrowcoordX[l], self.arrowcoordY[l], self.arrowcoordX[l], self.arrowcoordY[l]+20, arrow=tkinter.LAST)
            self.arrowY[l] = self.canvas0.create_line(self.arrowcoordX[l], self.arrowcoordY[l], self.arrowcoordX[l]-20, self.arrowcoordY[l], arrow=tkinter.LAST)
            self.arrowX[l+9] = self.canvas0.create_line(self.arrowcoordX[l], self.arrowcoordY[l], self.arrowcoordX[l], self.arrowcoordY[l]-20, arrow=tkinter.LAST)
            self.arrowY[l+9] = self.canvas0.create_line(self.arrowcoordX[l], self.arrowcoordY[l], self.arrowcoordX[l]+20, self.arrowcoordY[l], arrow=tkinter.LAST)

                

        #print(self.timelist)
    
        ## Button linked to the request_bias service in ROS, used to calibrate the sensors

        self.reset_button = tkinter.Button(self.window, text= "Request Bias", command = self.button_press) #rosservice call /hub_0/send_bias_request
        self.reset_button.grid(column=4, row=0)

        ## Button to close the window

        self.close_button = tkinter.Button(self.window, text= "Close", bg = "red", fg = "yellow", command = self.close_interface)
        self.close_button.grid(column=4, row=2)

        

        self.update()





    ## Functions

    def callback(self, data):
        self.msgX = [round(data.pillars[i].fX, 4) for i in range(9)]
        self.msgY = [round(data.pillars[i].fY, 4) for i in range(9)]
        self.msgZ = [round(data.pillars[i].fZ, 4) for i in range(9)]
        self.msgGF = [round(data.gfX, 4), round(data.gfY, 4), round(data.gfZ, 4)]
        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)
        #print(self.msg)

    def refresh_valuesZ(self):
        return self.msgZ
    
    def refresh_valuesY(self):
        return self.msgY
    
    def refresh_valuesX(self):
        return self.msgX
    
    def refresh_valuesGF(self):
        return self.msgGF
    
    def button_press(self):
        self.reset_text.config(text="Sensors calibrated")
        rospy.ServiceProxy('/hub_0/send_bias_request', BiasRequest)()
        self.window.after(1000, self.button_reset)

    def button_reset(self):
        self.reset_text.config(text=" ")
        #print('reset')
    
    def update(self):
        # Get new message from node
        self.msgX = self.refresh_valuesX()
        self.msgY = self.refresh_valuesY()
        self.msgZ = self.refresh_valuesZ()
        self.msgGF = self.refresh_valuesGF()
        #print(self.msg)
        
        # refresh plotted values
        for k in [2, 5, 8, 1, 4, 7, 0, 3, 6]:
            for i in range(len(self.graphZ[k])-1):
                self.graphX[k][i] = self.graphX[k][i+1]
                self.graphY[k][i] = self.graphY[k][i+1]
                self.graphZ[k][i] = self.graphZ[k][i+1]
            self.graphX[k][len(self.graphX[k])-1] = self.msgX[k]
            self.graphY[k][len(self.graphY[k])-1] = self.msgY[k]
            self.graphZ[k][len(self.graphZ[k])-1] = self.msgZ[k]
            self.lines[3*k].set_data(self.timelist, self.graphX[k])
            self.lines[3*k+1].set_data(self.timelist, self.graphY[k])
            self.lines[3*k+2].set_data(self.timelist, self.graphZ[k])
            self.canvasG[k].draw()

        for i in range(len(self.graphGX)-1):
            self.graphGX[i] = self.graphGX[i+1]
            self.graphGY[i] = self.graphGY[i+1]
            self.graphGZ[i] = self.graphGZ[i+1]
        self.graphGX[len(self.graphGX)-1] = self.msgGF[0]
        self.graphGY[len(self.graphGY)-1] = self.msgGF[1]
        self.graphGZ[len(self.graphGZ)-1] = self.msgGF[2]
        self.lineGFX.set_data(self.timelist, self.graphGX)
        self.lineGFY.set_data(self.timelist, self.graphGY)
        self.lineGFZ.set_data(self.timelist, self.graphGZ)
        self.canvasGF.draw()


        # Set pillar color according to force

        self.change_color()

        # Check if an arrow should be displayed

        self.add_arrow()

        # Display new values

        for k in range(9):
            self.displayX[k].config(text = self.msgX[k])
            self.displayY[k].config(text = self.msgY[k])
            self.displayZ[k].config(text = self.msgZ[k])

        
        self.displayGF[0].config(text = self.msgGF[0])
        self.displayGF[1].config(text = self.msgGF[1])
        self.displayGF[2].config(text = self.msgGF[2])

        # Update tkinter window

        self.window.after(1, self.update) 

    def change_color(self):
        for l in range(len(self.msgZ)):
            if l == 0:
                k = 6
            elif l == 1:
                k = 3
            elif l == 2:
                k = 0
            elif l == 3:
                k = 7
            elif l == 4:
                k = 4
            elif l == 5:
                k = 1
            elif l == 6:
                k = 8
            elif l == 7:
                k = 5
            elif l == 8:
                k = 2
            if self.msgZ[l] < -0.1:
                self.canvas0.itemconfig(self.circle_list[k], fill = 'grey')
                self.fillcolor[l] = 'grey'
            elif -0.1 < self.msgZ[l] < 0.1:
                self.canvas0.itemconfig(self.circle_list[k], fill = '#fcc3d5')
                self.fillcolor[l] = '#fcc3d5'
            elif 0.1 < self.msgZ[l] < 0.5:
                self.canvas0.itemconfig(self.circle_list[k], fill = '#f592b5')
                self.fillcolor[l] = '#f592b5'
            elif 0.5 < self.msgZ[l] < 1:
                self.canvas0.itemconfig(self.circle_list[k], fill = '#f3719e')
                self.fillcolor[l] = '#f3719e'
            elif 1 < self.msgZ[l] < 2:
                self.canvas0.itemconfig(self.circle_list[k], fill = '#f05088')
                self.fillcolor[l] = '#f05088'
            elif 2 < self.msgZ[l] < 3:
                self.canvas0.itemconfig(self.circle_list[k], fill = '#ee2f71')
                self.fillcolor[l] = '#ee2f71'
            elif 3 < self.msgZ[l] < 4:
                self.canvas0.itemconfig(self.circle_list[k], fill = '#e7125c')
                self.fillcolor[l] = '#e7125c'
            elif 4 < self.msgZ[l] < 5:
                self.canvas0.itemconfig(self.circle_list[k], fill = '#c6104f')
                self.fillcolor[l] = '#c6104f'
            elif 5 < self.msgZ[l] < 6:
                self.canvas0.itemconfig(self.circle_list[k], fill = '#a50d42')
                self.fillcolor[l] = '#a50d42'
            elif 6 < self.msgZ[l] < 7:
                self.canvas0.itemconfig(self.circle_list[k], fill = '#840a34')
                self.fillcolor[l] = '#840a34'
            elif 7 < self.msgZ[l] < 8:
                self.canvas0.itemconfig(self.circle_list[k], fill = '#630827')
                self.fillcolor[l] = '#630827'
            elif 8 < self.msgZ[l] < 10:
                self.canvas0.itemconfig(self.circle_list[k], fill = '#42051a')
                self.fillcolor[l] = '#42051a'
    

    
    def add_arrow(self):
        for l in range(9):
            if self.msgX[l] > 0.3:
                self.canvas0.itemconfig(self.arrowX[l], fill = 'black')
                self.canvas0.itemconfig(self.arrowX[l+9], fill = self.fillcolor[l])
            if self.msgY[l] > 0.3:
                self.canvas0.itemconfig(self.arrowY[l], fill = 'black')
                self.canvas0.itemconfig(self.arrowY[l+9], fill = self.fillcolor[l])
            if -0.3 < self.msgX[l] < 0.3:
                self.canvas0.itemconfig(self.arrowX[l], fill = self.fillcolor[l])
                self.canvas0.itemconfig(self.arrowX[l+9], fill = self.fillcolor[l])
            if -0.3 < self.msgY[l] < 0.3:
                self.canvas0.itemconfig(self.arrowY[l], fill = self.fillcolor[l])
                self.canvas0.itemconfig(self.arrowY[l+9], fill = self.fillcolor[l])
            if self.msgX[l] < -0.3:
                self.canvas0.itemconfig(self.arrowX[l+9], fill = 'black')
                self.canvas0.itemconfig(self.arrowX[l], fill = self.fillcolor[l])
            if self.msgY[l] < -0.3:
                self.canvas0.itemconfig(self.arrowY[l+9], fill = 'black')
                self.canvas0.itemconfig(self.arrowY[l], fill = self.fillcolor[l])

 



    def close_interface(self):
        plt.close('all')
        self.window.destroy()
 


# Main program

if __name__ == '__main__':

    rospy.init_node('sensor_display', anonymous = False)

    sensor = interface()
    sensor.window.mainloop()
