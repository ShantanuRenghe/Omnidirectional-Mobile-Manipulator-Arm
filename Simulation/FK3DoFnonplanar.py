import numpy as np
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

        self.l, = self.ax.plot([0, 0, self.length2, self.length2+self.length3], [0, 0, 0, 0], [0, self.length1, self.length1, self.length1], marker='o', color = "b")

        self.axtheta1 = plt.axes([0.25, 0.15, 0.65, 0.03])
        self.axtheta2 = plt.axes([0.25, 0.1, 0.65, 0.03])
        self.axtheta3 = plt.axes([0.25, 0.05, 0.65, 0.03])

        self.thetaslider1 = Slider(self.axtheta1, 'θ1', -180.0, 180.0, 0, valstep=1)
        self.thetaslider2 = Slider(self.axtheta2, 'θ2', -180.0, 180.0, 0, valstep=1)
        self.thetaslider3 = Slider(self.axtheta3, 'θ3', -180.0, 180.0, 0, valstep=1 )

        self.dh_params = np.array([[0, self.length1, 0, np.pi/2],
                                   [0, 0, self.length2, 0],
                                   [0, 0, self.length3, 0]]) # theta, d, a, alpha

        self.thetaslider1.on_changed(self.update)
        self.thetaslider2.on_changed(self.update)
        self.thetaslider3.on_changed(self.update)

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
        t1 = self.thetaslider1.val
        t1 = (t1 * np.pi) / 180
        t2 = self.thetaslider2.val
        t2 = (t2 * np.pi) / 180
        t3 = self.thetaslider3.val
        t3 = (t3 * np.pi) / 180

        self.dh_params = np.array([[t1, self.length1, 0, np.pi/2],
                                   [t2, 0, self.length2, 0],
                                   [t3, 0, self.length3, 0]])

        self.l.remove()
        self.l, = self.ax.plot([0, self.transform_wrt_base(self.dh_params, 1)[0][3], self.transform_wrt_base(self.dh_params, 2)[0][3], self.transform_wrt_base(self.dh_params, 3)[0][3]], 
                     [0, self.transform_wrt_base(self.dh_params, 1)[1][3], self.transform_wrt_base(self.dh_params, 2)[1][3], self.transform_wrt_base(self.dh_params, 3)[1][3]], 
                     [0, self.transform_wrt_base(self.dh_params, 1)[2][3], self.transform_wrt_base(self.dh_params, 2)[2][3], self.transform_wrt_base(self.dh_params, 3)[2][3]],
                     marker = "o", color = "b")


if __name__ == "__main__":
    robot_arm = RobotArmVisualizer() 