import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pygame


class ControllerTalker(Node):
    def __init__(self):
        super().__init__('controller_talker')

        pygame.init()
        pygame.joystick.init()
        try:
            joystick = pygame.joystick.Joystick(0)
        except Exception as e:
            print(e)
            print('Please connect the controller.')
            exit()
        joystick.init()

        self.publisher = self.create_publisher(String, 'controller_topic', 10)
        self.timer = self.create_timer(0.1, self.ontick)
    
    def ontick(self):
        msg = String()
        msg.data = 'stop'
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN:
                if event.button == 5:
                    msg.data = 'move_to_default'
                    break
                if event.button == 10:
                    msg.data = 'move_to_home'
                    break
            
            elif event.type == pygame.JOYAXISMOTION:
                if event.axis == 0:
                    if event.value > 0.2:
                        msg.data = 'angle0+'
                        break
                    elif event.value < -0.2:
                        msg.data = 'angle0-'
                        break
                    else:
                        msg.data = 'stop'
                # if event.axis == 1:
                #     if event.value > 0.2:
                #         action = 3
                #         break
                #     elif event.value < -0.2:
                #         action = 4
                #         break
                #     else:
                #         action = -1
        
        self.get_logger().info(f'{msg.data}')
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = ControllerTalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()