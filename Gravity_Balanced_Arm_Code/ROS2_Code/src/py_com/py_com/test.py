#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import serial
from serial import Serial
import time

class MyNode(Node):

    def __init__(self):
        super().__init__("serial_com_gravity")
        self.publisher = self.create_publisher(PointStamped, "encoder_counter_gravity", 10)
        self.pwm_subscriber_ = self.create_subscription(Point, "pwm_values_gravity", self.write, 1)
        self.get_logger().info("hello from ros2")
        self.create_timer(0.000300, self.timer_callback)
        self.comport = serial.Serial(port='/dev/ttyUSB0', baudrate=921600, 
                                     timeout=0.1, xonxoff=False, rtscts=False, write_timeout=None, 
                                     dsrdtr=False, inter_byte_timeout=None, exclusive=None)
    
    def write(self, msg: Point):
        
    
        data = str(int(msg.x)) + ":" + str(int(msg.y)) + ":" +str(int(msg.z)) + ";\n"
        #self.get_logger().info('data not full enoug to %s' % (data.encode('UTF-8')))#size 13
        #while self.comport.out_waiting < 8:
         #   pass
        self.comport.write(data.encode('UTF-8'))

    def timer_callback(self):
        
        #serial_data = str(self.comport.readline(), 'UTF-8')
        #start_time = time.time()
        """
        serial_data = str(self.comport.read())
        process_data = [-1,-1,-1]
        
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
            msg.point.x += int(process_data[0])
            msg.point.y += int(process_data[1])
            msg.point.z += int(process_data[2])
            
            self.publisher.publish(msg)
        except:
            self.get_logger().info('data not full enoug to %s' % (process_data,))

        
        #end_time = time.time()
        #self.get_logger().info('Time to publish: %f' %(end_time- start_time))

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()