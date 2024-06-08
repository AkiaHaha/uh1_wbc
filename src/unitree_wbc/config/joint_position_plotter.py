import rospy
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
import numpy as np

class JointPositionPlotter:
    def __init__(self):
        rospy.init_node('joint_position_plotter', anonymous=True)
        self.joint_positions = []

        # Create a subscriber
        rospy.Subscriber('joint_positions', Float64MultiArray, self.callback)

        # Create a plot
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.lines = [self.ax.plot([], [])[0] for _ in range(19)]
        self.ax.set_xlim(0, 100)  # Adjust as necessary
        self.ax.set_ylim(-1, 1)  # Adjust as necessary

    def callback(self, msg):
        self.joint_positions.append(msg.data)

        # Keep only the last 100 positions for plotting
        if len(self.joint_positions) > 100:
            self.joint_positions.pop(0)

        # Convert list to numpy array for easy slicing
        data = np.array(self.joint_positions)
        if data.shape[0] > 0:
            for i in range(19):
                self.lines[i].set_xdata(np.arange(data.shape[0]))
                self.lines[i].set_ydata(data[:, i])

        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        plt.pause(0.01)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    plotter = JointPositionPlotter()
    plotter.run()
