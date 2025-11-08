#!/usr/bin/env python3

"""
Check Battery State
Prints the current battery charge percentage once and exits.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32MultiArray


class BatteryChecker(Node):
    def __init__(self):
        super().__init__('battery_checker')
        self.battery_msg_received = False
        self.robot_status_received = False
        self.best_percentage = None
        self.best_voltage = None
        self.exit_scheduled = False
        self.battery_received_at = None
        
        # Subscribe to battery status topic
        self.subscription = self.create_subscription(
            BatteryState,
            '/rover_mini/battery_status',
            self.battery_callback,
            10)
        
        # Also subscribe to robot_status for raw battery data
        self.status_subscription = self.create_subscription(
            Float32MultiArray,
            '/robot_status',
            self.robot_status_callback,
            10)
        
        self.get_logger().info('Waiting for battery status message...')
        
        # Set timeout to avoid hanging indefinitely
        self.timer = self.create_timer(0.1, self.check_timeout)
        self.timeout_count = 0
        self.max_timeout = 150  # 15 seconds max wait
        self.exit_after_ticks = 20  # Exit 2 seconds (20 * 0.1s) after receiving battery message

    def battery_callback(self, msg):
        if self.battery_msg_received:
            return
            
        self.battery_msg_received = True
        self.battery_received_at = self.timeout_count
        
        # Store values from battery topic
        if hasattr(msg, 'percentage') and msg.percentage is not None:
            percentage = msg.percentage
            voltage = msg.voltage if hasattr(msg, 'voltage') and msg.voltage is not None else 0.0
            
            # Store the values (even if 0)
            if self.best_percentage is None:
                self.best_percentage = percentage
                self.best_voltage = voltage
            
            # If we get valid non-zero data, use it immediately
            if percentage > 0 or voltage > 0:
                self.best_percentage = percentage
                self.best_voltage = voltage
                self.print_and_exit()
                return
            else:
                # Wait a bit for robot_status topic, but schedule exit after short delay
                self.get_logger().info('Battery topic shows 0% - checking robot_status for raw data...')

    def robot_status_callback(self, msg):
        # Robot status has battery data starting at index 20 (after 20 motor values)
        # Index 20: battery1_voltage
        # Index 26: battery1_SOC
        if len(msg.data) >= 27:
            voltage = msg.data[20]
            soc = msg.data[26]
            
            self.robot_status_received = True
            
            # Use robot_status data if it's valid and better than battery topic
            if voltage > 0 or soc > 0:
                if self.best_percentage is None or (soc > 0 and self.best_percentage == 0):
                    self.best_percentage = soc
                    self.best_voltage = voltage
                    
                    # If we have good data, print it immediately
                    if soc > 0 or voltage > 0:
                        self.print_and_exit()
                        return
            else:
                # Store 0 values if we don't have anything yet
                if self.best_percentage is None:
                    self.best_percentage = soc
                    self.best_voltage = voltage

    def print_and_exit(self):
        if self.exit_scheduled:
            return
        self.exit_scheduled = True
        
        if self.best_percentage is not None:
            print(f"Battery Charge: {self.best_percentage:.1f}%")
            if self.best_voltage is not None and self.best_voltage > 0:
                print(f"Battery Voltage: {self.best_voltage:.2f} V")
            else:
                print("Battery Voltage: Not available")
            
            # Check if data seems valid
            if self.best_percentage == 0.0 and (self.best_voltage is None or self.best_voltage == 0.0):
                print("\n⚠️  Warning: Battery reading is 0%.")
                print("This could mean:")
                print("  1. Robot is not physically connected")
                print("  2. Robot driver is not communicating with hardware")
                print("  3. Battery is actually discharged")
                print("\nCheck:")
                print("  - Is the robot powered on?")
                print("  - Is USB cable connected?")
                print("  - Check robot driver logs for connection errors")
        else:
            print("Error: Could not retrieve battery data")
        
        rclpy.shutdown()

    def check_timeout(self):
        self.timeout_count += 1
        
        # If we received battery message, exit after short delay to allow robot_status to arrive
        if self.battery_msg_received and not self.exit_scheduled:
            ticks_since_battery = self.timeout_count - self.battery_received_at
            if ticks_since_battery >= self.exit_after_ticks:
                # Time to exit after giving robot_status a chance
                self.print_and_exit()
                return
        
        # Absolute timeout
        if self.timeout_count >= self.max_timeout:
            # Try to print whatever we have
            if self.best_percentage is not None:
                self.print_and_exit()
            else:
                print("Error: Timeout waiting for battery status message.")
                print("\nTroubleshooting:")
                print("  1. Check if robot driver is running:")
                print("     ros2 launch roverrobotics_driver mini.launch.py")
                print("  2. Check if topics exist:")
                print("     ros2 topic list | grep battery")
                print("  3. Check robot connection:")
                print("     ls -l /dev/ttyACM0")
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        battery_checker = BatteryChecker()
        rclpy.spin(battery_checker)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

