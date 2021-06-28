import rclpy
from rclpy.node import Node

from rclpy.qos import QoSReliabilityPolicy, QoSProfile
from cf_messages.msg import Ping
from std_msgs.msg import UInt8


import csv
import sys
import time
import numpy as np
import os

class LatencyTest(Node):

    def __init__(self):
        super().__init__('latency_test')
        self.declare_parameter('num_samples', 255)
        self.declare_parameter('num_bytes', 16)
        self.declare_parameter('frequency', 1)
        self.declare_parameter('id', 1)
        self.declare_parameter('filename', 'rtt_unknown_50cm_gs_cf')

        self.frequency = self.get_parameter('frequency').value
        self.num_samples  = self.get_parameter('num_samples').value
        self.num_bytes = self.get_parameter('num_bytes').value
        self.id = self.get_parameter('id').value
        self.csv_filename = f'results/{self.get_parameter("filename").value}_f{self.frequency}hz_N{self.num_samples}_{self.num_bytes}B_{self.id}.csv'
        timer_period = 1/self.frequency

        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.publisher = self.create_publisher(
            Ping,
            '/microROS/ping',
            QoSProfile(depth=3, reliability=QoSReliabilityPolicy.BEST_EFFORT))

        self.subscription = self.create_subscription(
			Ping,
			'/microROS/pong',
			self.pong_callback,
			QoSProfile(depth=3, reliability=QoSReliabilityPolicy.BEST_EFFORT))

        self.seq_num = 0

        #jsut for testing

        self.published_msgs = {}
        self.received_msgs = {}
        self.latencies = []


    def timer_callback(self):
        if self.seq_num < self.num_samples: 
            msg = Ping()
            for i in range(self.num_bytes):
                msg.data.append(np.uint8(i % 256) )
            msg.data[0] = self.seq_num
            self.get_logger().info(f"published: {msg} at {self.get_clock().now()}")
            self.published_msgs[msg.data[0]] = time.time()
            self.publisher.publish(msg)
            self.seq_num = self.seq_num + 1
        else:
            self.export_results()
            pdr = len(self.received_msgs) / len(self.published_msgs) * 100 
            self.get_logger().info(f'Latency tests finished. PDR = {pdr} %')
            os._exit(0)

    def control_pong_callback(self, msg):
        self.get_logger().info('Received control message')

    def pong_callback(self, msg):
        self.get_logger().info(f'Received {msg} at {self.get_clock().now()}')
        self.received_msgs[msg.data[0]] = time.time()
        latency = self.received_msgs[msg.data[0]] - self.published_msgs[msg.data[0]]
        self.get_logger().info(f'Latency: {latency*1000}')
        self.latencies.append([latency])

    def export_results(self):
        with open(self.csv_filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            #writer.writerow([HEADER])
            writer.writerows(self.latencies)

def main(args=None):
    rclpy.init(args=args)

    latency_test = LatencyTest()

    rclpy.spin(latency_test)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()