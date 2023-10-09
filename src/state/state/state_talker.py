import rclpy
from rclpy.node import Node
from state_server_srv.srv import StateService
from pymycobot import MyCobot, PI_PORT, PI_BAUD


class StateServer(Node):
    def __init__(self):
        super().__init__('state_server')

        mc = MyCobot(PI_PORT, PI_BAUD)

        self.srv = self.create_service(StateService, 'state_service', self.on_request)
    
    def on_request(self):
        