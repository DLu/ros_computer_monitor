import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from computer_msgs.msg import CPUUsage
import datetime

class UsageViz:
    def __init__(self, interactive=True):
        if interactive:
            plt.ion()

        self.fig, self.axes = plt.subplots(2)

        self.t = []
        self.cpu = {}
        self.mem = {}
        self.labels = {}
        self.interactive = interactive

        if interactive:
            self.timedelta = datetime.timedelta(seconds=30)
        else:
            self.timedelta = None

    def update(self, t, msg):
        for proc in msg.processes:
            if proc.pid not in self.cpu:
                self.labels[proc.pid] = proc.name.lstrip('_')
                self.cpu[proc.pid] = [0.0] * len(self.t)
                self.mem[proc.pid] = [0.0] * len(self.t)
            self.cpu[proc.pid].append(proc.cpu_percent)
            self.mem[proc.pid].append(proc.memory_percent)
        self.t.append(datetime.datetime.fromtimestamp(t))

        while self.timedelta and self.t[-1] - self.t[0] > self.timedelta:
            self.t.pop(0)
            for key in self.cpu:
                self.cpu[key].pop(0)
                self.mem[key].pop(0)

        if self.timedelta:
            for key in sorted(self.labels.keys()):
                if sum(self.cpu[key]) == 0.0 and sum(self.mem[key]) == 0.0:
                    del self.labels[key]
                    del self.cpu[key]
                    del self.mem[key]

        for pid in self.labels:
            while len(self.cpu[pid]) < len(self.t):
                self.cpu[pid].append(0.0)
                self.mem[pid].append(0.0)

        pids = sorted(self.labels.keys())
        labels = [self.labels[pid] for pid in pids]
        for ax, data in zip(self.axes, [self.cpu, self.mem]):
            ax.clear()
            ax.stackplot(self.t, *[data[pid] for pid in pids], labels=labels)
            ax.legend()

        if self.interactive:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

    def show(self):
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = Node('ros_top')
    uv = UsageViz()

    def usage_cb(msg):
        now = node.get_clock().now().nanoseconds / 1e9
        uv.update(now, msg)

    node.create_subscription(CPUUsage, '/cpu_usage', usage_cb, 1)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
