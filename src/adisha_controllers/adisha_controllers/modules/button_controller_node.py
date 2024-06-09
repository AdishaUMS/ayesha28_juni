import RPi.GPIO as gpio
import rclpy
from rclpy.node import Node
import adisha_interfaces.msg as adisha_interfaces



class ButtonControllerNode(Node):

    def __init__(self) -> None:
        super().__init__('ButtonControllerNode')
        self.declare_parameter('id', rclpy.Parameter.Type.STRING)
        self.declare_parameter('master_clock', rclpy.Parameter.Type.DOUBLE)
        self.declare_parameter('input_pin', rclpy.Parameter.Type.INTEGER_ARRAY)

        self.ID             = self.get_parameter('id').value
        self.MASTER_CLOCK   = self.get_parameter('master_clock').value
        self.INPUT_PIN      = self.get_parameter('input_pin').value

        gpio.setmode(gpio.BCM)

        for pin in self.INPUT_PIN:
            gpio.setup(pin, gpio.IN, pull_up_down=gpio.PUD_UP)

        self.button_input_pub = self.create_publisher(
            msg_type    = adisha_interfaces.Button,
            topic       = f'{self.ID}/button_input',
            qos_profile = 1000
        )

        self.button_timer = self.create_timer(
            self.MASTER_CLOCK,
            self.buttonTimerCallback
        )


    
    def buttonTimerCallback(self) -> None:
        button_input_msg    = adisha_interfaces.Button()
        input_temp          = []

        for pin in self.INPUT_PIN:
            input_temp.append(gpio.input(pin))

        button_input_msg.val = input_temp.copy()
        self.button_input_pub.publish(button_input_msg)