import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Accel
from std_msgs.msg import Bool
from jetracer.nvidia_racecar import NvidiaRacecar


class JetracerDriver(Node):
    def __init__(self):
        super().__init__("jetracer_driver")
        self.create_subscription(Accel, "/vehicle/ctrl", self.ctrlCallback, qos_profile_sensor_data)
        self.create_subscription(Bool, "/vehicle/enable", self.enableCallback, qos_profile_sensor_data)
        # mode ? "auto" : "manual"
        self.mode = False
        # create instance for control
        self.car = NvidiaRacecar()
    
    def ctrlCallback(self, msg):
        '''
        msg :Twist
        linear
            x: throttle_value(float64)
            y: throttle_gain(float64)
            z: throttle_offset(float64)
        angular
            x: steering_value(float64)
            y: steering_gain(float64)
            z: steering_offset(float64)
        '''
        self.car.throttle = msg.linear.x
        self.car.throttle_gain = msg.linear.y
        self.car.throttle_offset = msg.linear.z

        self.car.steering = msg.angular.x
        self.car.steering_gain = msg.angular.y
        self.car.steering_offset = msg.angular.z

    def enableCallback(self, msg):
        message = "Auto" if msg.data == True else "Manual"
        print(message)



def main():
    rclpy.init()
    node = JetracerDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
