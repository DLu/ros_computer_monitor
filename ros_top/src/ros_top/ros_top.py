import rclpy
from rclpy.node import Node
from computer_msgs.msg import Process, CPUUsage

import os
import pathlib
import psutil

NODE_FIELDS = ['pid', 'get_cpu_percent', 'get_memory_percent', 'get_num_threads']

class ROSTop(Node):

    def __init__(self):
        super().__init__('ros_top')
        self.pub = self.create_publisher(CPUUsage, '/cpu_usage', 1)
        self.declare_parameter('workspace', str(pathlib.Path('~/ros2_ws').expanduser()))

        distro = os.environ['ROS_DISTRO']
        self.prefixes = [f'/opt/ros/{distro}', self.get_parameter('workspace').value]

        self.processes = {}

        self.poll()

        self.declare_parameter('poll_frequency', 0.2)
        self.declare_parameter('pub_frequency', 1.0)

        freq = self.get_parameter('poll_frequency').value
        self.poll_timer = self.create_timer(1.0 / freq, self.poll)

        freq = self.get_parameter('pub_frequency').value
        self.pub_timer = self.create_timer(1.0 / freq, self.publish)

    def is_ros(self, process):
        if process.info['name'] == 'ros2':
            return False
        keys = process.info['cmdline'][:2] + [process.info['exe']]
        for key in keys:
            if not key:
                return False
            for prefix in self.prefixes:
                if key.startswith(prefix):
                    return True

    def poll(self, event=None):
        for p in psutil.process_iter(['name', 'cmdline', 'exe']):
            if self.is_ros(p):
                self.processes[p.pid] = p

    def publish(self, event=None):
        cpu = CPUUsage()
        for pid, process in self.processes.items():
            p = Process()
            p.pid = process.pid
            p.command = ' '.join(process.cmdline())
            for field in ['name', 'cpu_percent', 'memory_percent', 'num_threads']:
                setattr(p, field, getattr(process, field)())
            cpu.processes.append(p)
        self.pub.publish(cpu)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ROSTop())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
