import numpy as np
import math
import matplotlib.pyplot as plt
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

        self.axx = plt.axes([0.25, 0.15, 0.65, 0.03])
        self.axy = plt.axes([0.25, 0.1, 0.65, 0.03])
        self.axz = plt.axes([0.25, 0.05, 0.65, 0.03])

        self.xslider = Slider(self.axx, 'X', -(self.length2+self.length3), self.length2+self.length3, self.length2+self.length3, valstep=0.1)
        self.yslider = Slider(self.axy, 'Y', -(self.length2+self.length3), self.length2+self.length3, 0, valstep=0.1)
        self.zslider = Slider(self.axz, 'Z', self.length1-self.length2-self.length3, self.length1+self.length2+self.length3, self.length1, valstep=0.1)

        self.dh_params = np.array([[0, self.length1, 0, np.pi/2],
                                   [0, 0, self.length2, 0],
                                   [0, 0, self.length3, 0]]) # theta, d, a, alpha

        self.xslider.on_changed(self.update)
        self.yslider.on_changed(self.update)
        self.zslider.on_changed(self.update)

        plt.show()

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
        x = self.xslider.val
        y = self.yslider.val
        z = self.zslider.val

        try:
            t1 = np.arctan2(y, x)

            a = self.length3
            b = self.length2
            c = x / np.cos(t1)
            d = z - self.length1
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