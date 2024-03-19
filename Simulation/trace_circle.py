import numpy as np
import math
from matplotlib import pyplot as plt, animation
from matplotlib.widgets import Slider

class RobotArmVisualizer:
    def __init__(self):

        self.length1 = 2
        self.length2 = 2
        self.length3 = 2
        self.fig = plt.figure(figsize=(7,7))
        self.ax = self.fig.add_subplot(projection = '3d')
        
        plt.subplots_adjust(bottom=0.35)
        self.ax.set_xlim3d(-(self.length2+self.length3)*1.25, (self.length2+self.length3)*1.25)
        self.ax.set_ylim3d(-(self.length2+self.length3)*1.25, (self.length2+self.length3)*1.25)
        self.ax.set_zlim3d(-(self.length1+self.length2+self.length3)*1.25, (self.length1+self.length2+self.length3)*1.25)
        self.ax.set_aspect('equal', adjustable='box')

        self.l, = self.ax.plot([0, 0, self.length2, self.length2+self.length3], [0, 0, 0, 0], [0, self.length1, self.length2, self.length3], marker='o', color = "b")

        self.dh_params = np.array([[0, self.length1, 0, np.pi/2],
                                   [0, 0, self.length2, 0],
                                   [0, 0, self.length3, 0]]) # theta, d, a, alpha

        self.axradius = plt.axes([0.25, 0.15, 0.65, 0.03])
        self.radius_slider = Slider(self.axradius, 'Radius', 0, self.length2 + self.length3, 2, valstep=0.1)
        self.axbeta = plt.axes([0.25, 0.1, 0.65, 0.03])
        self.beta_slider = Slider(self.axbeta, 'φ', -math.pi, math.pi, 0, valstep=0.01)
        self.axgamma = plt.axes([0.25, 0.05, 0.65, 0.03])
        self.gamma_slider = Slider(self.axgamma, 'θ', -math.pi, math.pi, 0, valstep=0.01)

        self.a = self.length2 + self.length3
        self.radius = 2
        self.alpha = np.arcsin(self.radius/self.a)
        self.beta = 0
        self.gamma = 0

        self.center = [float(input("Enter center X coordinate : ")), float(input("Enter center Y coordinate : ")), float(input("Enter center Z coordinate : "))]

        self.theta = np.linspace(-math.pi, math.pi, 100)
        self.x = self.fx()
        self.y = self.fy()
        self.z = self.fx()
        self.beta_slider.on_changed(self.update)
        self.gamma_slider.on_changed(self.update)
        self.radius_slider.on_changed(self.update)

        self.g, = self.ax.plot(self.x, self.y, self.z, color="r")

        self.anim = animation.FuncAnimation(self.fig, self.move, interval=1, frames=100 - 1)

        plt.show()

    def fx(self):   
        x = []
        for i in self.theta:
            x.append(self.center[0] + (self.radius*np.cos(self.beta)*np.cos(self.gamma))*np.cos(i) + (self.radius*np.sin(self.gamma))*np.sin(i) - (np.cos(self.alpha)*np.sin(self.beta)*np.cos(self.gamma)))
        return x
    

    def fy(self):
        y = []
        for i in self.theta:
            y.append(self.center[1] - (self.radius*np.cos(self.beta)*np.sin(self.gamma))*np.cos(i) + (self.radius*np.cos(self.gamma))*np.sin(i) + (np.cos(self.alpha)*np.sin(self.beta)*np.sin(self.gamma)))
        return y
    

    def fz(self):
        z = []
        for i in self.theta:
            z.append(self.center[2] + (self.radius*np.sin(self.beta))*np.cos(i) + (np.cos(self.alpha)*np.cos(self.beta)))
        return z


    def transform_wrt_base(self, params, frame):
        tmat = np.array([[np.cos(params[frame - 1][0]), -(np.cos(params[frame - 1][3]) * np.sin(params[frame - 1][0])), np.sin(params[frame - 1][3]) * np.sin(params[frame - 1][0]), (params[frame - 1][2]) * np.cos(params[frame - 1][0])],
                         [np.sin(params[frame - 1][0]), np.cos(params[frame - 1][3]) * np.cos(params[frame - 1][0]), -(np.sin(params[frame - 1][3]) * np.cos(params[frame - 1][0])), params[frame - 1][2] * np.sin(params[frame - 1][0])],
                         [0, np.sin(params[frame - 1][3]), np.cos(params[frame - 1][3]), params[frame - 1][1]],
                         [0, 0, 0, 1]])
        if frame == 1:
            return tmat
        else:
            return (self.transform_wrt_base(params, frame - 1)).dot(tmat)

    def update(self, val):
        self.beta = self.beta_slider.val
        self.gamma = self.gamma_slider.val
        self.radius = self.radius_slider.val
        self.alpha = np.arcsin(self.radius/self.a)
        self.x = self.fx()
        self.y = self.fy()
        self.z = self.fz()
        self.g.remove()
        self.g, = self.ax.plot(self.x, self.y, self.z, color="r")


    def move(self, i):
        x_cord = self.x[i]
        y_cord = self.y[i]
        z_cord = self.z[i]

        try:
            t1 = np.arctan2(y_cord, x_cord)

            a = self.length3
            b = self.length2
            c = x_cord / np.cos(t1)
            d = z_cord - self.length1
            c3 = (c**2 + d**2 - a**2 - b**2) / (2*a*b)
            s3 = math.sqrt(1-(c3**2))
            t3 = np.arctan2(s3, c3)

            r = a*c3 + b
            s = a*s3
            t2 = np.arctan2(r*d - s*c, r*c + s*d)

            self.dh_params = np.array([[t1, self.length1, 0, np.pi/2],
                                    [t2, 0, self.length2, 0],
                                    [t3, 0, self.length3, 0]]) # theta, d, a, alpha
            
        except ValueError:
            print("Coordinate outside work envelope")

        self.l.remove()
        self.l, = self.ax.plot([0, self.transform_wrt_base(self.dh_params, 1)[0][3], self.transform_wrt_base(self.dh_params, 2)[0][3], self.transform_wrt_base(self.dh_params, 3)[0][3]], [0, self.transform_wrt_base(self.dh_params, 1)[1][3], self.transform_wrt_base(self.dh_params, 2)[1][3], self.transform_wrt_base(self.dh_params, 3)[1][3]], [0, self.transform_wrt_base(self.dh_params, 1)[2][3], self.transform_wrt_base(self.dh_params, 2)[2][3], self.transform_wrt_base(self.dh_params, 3)[2][3]], marker = "o", color = "b")

if __name__ == "__main__":
    robot_arm = RobotArmVisualizer()
