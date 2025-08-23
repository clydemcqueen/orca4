#!/usr/bin/env python3

"""
Auto-connect script for multi-AUV base controllers.
This script automatically enables all base controllers when the simulation starts.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import time

class AutoConnector(Node):
    def __init__(self):
        super().__init__('auto_connector')
        self.get_logger().info('Auto Connector started - waiting for base controllers...')
        
        # Wait a bit for services to come up
        time.sleep(5.0)
        
        # Connect to all base controllers
        self.connect_base_controllers()
        
    def connect_base_controllers(self):
        auv_names = ['auv1', 'auv2', 'auv3']
        
        for auv in auv_names:
            service_name = f'/{auv}/conn'
            self.get_logger().info(f'Connecting to {service_name}...')
            
            try:
                client = self.create_client(SetBool, service_name)
                
                if client.wait_for_service(timeout_sec=10.0):
                    request = SetBool.Request()
                    request.data = True
                    
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(self, future)
                    
                    if future.result() is not None:
                        response = future.result()
                        if response.success:
                            self.get_logger().info(f'{auv} base controller connected successfully!')
                        else:
                            self.get_logger().error(f'Failed to connect {auv} base controller: {response.message}')
                    else:
                        self.get_logger().error(f'Service call failed for {auv}')
                else:
                    self.get_logger().error(f'Service {service_name} not available')
                    
            except Exception as e:
                self.get_logger().error(f'Exception while connecting {auv}: {str(e)}')
        
        self.get_logger().info('Auto connector finished - all base controllers should be connected!')

def main():
    rclpy.init()
    auto_connector = AutoConnector()
    
    # Keep the node running for a bit to complete all connections
    time.sleep(2.0)
    
    auto_connector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
