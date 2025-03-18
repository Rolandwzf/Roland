import rclpy
from rclpy.node import Node
from status_interfaces.msg import SystemStatus   #导入自定义的 SystemStatus 消息类型
import psutil            #系统信息
import platform          #主机信息

class SysStatusPub(Node):
    #定义一个 ROS 2 节点类，继承自 rclpy.node.Node
    def __init__(self,node_name):
        #super().__init__(node_name)：初始化 ROS 2 节点，node_name 是节点的名称
        super().__init__(node_name)
        #创建一个发布者，发布 SystemStatus 消息类型的数据，话题名称为 sys_status，10 是消息队列的大小（队列存储最多 10 条未被订阅的消息）
        self.status_publisher_ = self.create_publisher(SystemStatus, 'sys_status', 10)
        #创建一个定时器，每 1 秒 触发一次 timer_callback 方法
        self.timer = self.create_timer(1, self.timer_callback)
    def timer_callback(self):
        #获取 CPU 使用率（%）。
        cpu_percent = psutil.cpu_percent()
        #获取内存信息（总大小、已用、可用等）
        memory_info = psutil.virtual_memory() 
        #获取网络 I/O 统计（发送/接收字节数）
        net_io_counters = psutil.net_io_counters()
        #创建 SystemStatus 消息对象，填充数据
        msg = SystemStatus()
        #时间戳
        msg.stamp = self.get_clock().now().to_msg()
        #获取主机名
        msg.host_name = platform.node()
        #CPU 使用率（%）
        msg.cpu_percent = cpu_percent
        #内存使用率（%）
        msg.memory_percent = memory_info.percent
        #内存总大小（MB）
        msg.memory_total = memory_info.total / 1024 / 1024
        #内存已用大小（MB）
        msg.memory_available = memory_info.available / 1024 / 1024
        #网络发送字节数（MB）
        msg.net_sent = net_io_counters.bytes_sent / 1024 / 1024
        #已接收的网络数据（MB）。
        msg.net_recv = net_io_counters.bytes_recv / 1024 / 1024
        #打印发布的消息
        self.get_logger().info('Publishing: "%s"' % msg)
        #通过 publish(msg) 方法向 ROS 2 话题 sys_status 发送 SystemStatus 消息
        self.status_publisher_.publish(msg)

def main():
    rclpy.init()
    #创建 sys_status_pub 节点
    node = SysStatusPub('sys_status_pub')
    rclpy.spin(node)
    rclpy.shutdown()