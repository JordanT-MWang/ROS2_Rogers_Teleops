#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point
import serial
from serial import Serial
import time

class MyNode(Node):

    def __init__(self):
        super().__init__("spring_com_node")
        self.publisher = self.create_publisher(PointStamped, "encoder_counter_spring", 10)
        self.pwm_subscriber_ = self.create_subscription(Point, "pwm_values_spring", self.write, 1)
        self.get_logger().info("hello from ros2d")
        self.create_timer(0.00040, self.timer_callback)
        self.comport = serial.Serial(port='/dev/ttyUSB0', baudrate=500000, 
                                     timeout=0.1, xonxoff=False, rtscts=False, write_timeout=None, 
                                     dsrdtr=False, inter_byte_timeout=None, exclusive=None)
        self.delta_time = 0.0
    def write(self, msg: Point):
        start_time = time.time()
        
        data = str(int(msg.x)) + ":" + str(int(msg.y)) + ":" +str(int(msg.z)) + ";\n"
        #self.get_logger().info('data not full enoug to %s' % (data.encode('UTF-8')))#size 13
        #while self.comport.out_waiting < 8:
         #   pass
        #self.get_logger().info('data not full enoug to %s' % (data.encode('UTF-8'),))
        self.comport.flush()
        self.comport.write(data.encode('UTF-8'))
        #self.delta_time = time.time() - start_time
    def timer_callback(self):
        
        #serial_data = str(self.comport.readline(), 'UTF-8')
        #start_time = time.time()
        """
        serial_data = str(self.comport.read())
        process_data = [-1,-1,-1]
comport
        process_string = ""
        joint_enc_val = 0
        
        while len(serial_data):
            
            if serial_data[2] == ':':
                process_data[joint_enc_val] = int(process_string)
                joint_enc_val +=1
                process_string = ""
                
            elif serial_data[2] == ';':
                process_data[joint_enc_val] = int(process_string)
                process_string = ""
                #self.comport.reset_input_buffer()
                #self.get_logger().info('his t %s' % (serial_data[entry],))
                break
            elif serial_data[2] != '\\':
                process_string = process_string + serial_data[2]
            serial_data = str(self.comport.read())
            #self.get_logger().info('Time to publish: %s' %(serial_data[2]))
        #self.get_logger().info('Time to publish: %s' %(serial_data))
        
        """
        
        """
        process_data = [-1,-1,-1]
        serial_data = str(self.comport.readline(), "UTF-8")
        process_string = ""
        joint_enc_val = 0

        for entry in range(len(serial_data)):
            self.get_logger().info('his t %s' % (serial_data[entry],))
            if serial_data[entry] == ':':
                process_data[joint_enc_val] = int(process_string)
                joint_enc_val +=1
                process_string = ""
                
            elif serial_data[entry] == ';':
                process_data[joint_enc_val]
                process_string = ""
                
                #self.get_logger().info('his t %s' % (serial_data[entry],))
                break
            else:
                process_string = process_string + serial_data[entry]
        """
        #serial_data_string = str(self.comport.readline(), "UTF-8")
        #############working at 2000 hz
        start_time = time.time()
        serial_data = str(self.comport.readline(), "UTF-8")
        
        #self.get_logger().info('his t %s' % (serial_data,))
        process_data = serial_data.split(':')
        for entry in range(len(process_data)):
            process_data[entry] = process_data[entry].strip('\n').strip().strip(';')
            if process_data[entry]:
               process_data[entry] = int(process_data[entry])
            
        #Working 1
        
        #self.get_logger().info('his t %s' % (process_data,))
        
        msg = PointStamped()
        current_time = time.time()
        msg.header.stamp.sec = int(current_time)
        msg.header.stamp.nanosec = int((current_time % 1) * (10**9))
        #msg.header.stamp = self.get_clock().now().to_msg()
    
        try: 
            
            msg.point.x = float(process_data[0])
            msg.point.y = float(process_data[1])
            msg.point.z = float(process_data[2])
            self.publisher.publish(msg)
        except:
            self.get_logger().info('data not full enoug to %s' % (process_data,))
        
        end_time = time.time()
        #msg.point.x = 0.0#int(process_data[0])
        #msg.point.y = 0.0#int(process_data[1])
        msg.point.z +=  self.delta_time
        #self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()