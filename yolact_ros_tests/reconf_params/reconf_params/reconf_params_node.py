#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
import sys
import threading

class MinimalClientAsync(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.client_srv_ = self.create_client(
            SetParameters, 'yolact_ros2_node/set_parameters')
        self.req_ = None
        self.future = None
        self.req_sended = False
        self.finish = False
        self.th_ = threading.Thread(target=self.readerConf_)
        self.th_.start()

    def readerConf_(self):
        try:
            while (not self.finish):
                self.printMenu_()
                option = input("Select an Option: ")
                if (not self.optionOk_(option)):
                    self.get_logger().error('Invalid Option\n')
                    continue
                if (int(option) == 7):
                    self.finish = True
                    break
                value = input("Introduce a Value: ")
                if (not self.valueOk_(int(option), value)):
                    self.get_logger().error('Invalid Value\n')
                self.send_request(int(option), value)

        except KeyboardInterrupt:
            pass

    def optionOk_(self, option):
        try:
            n = int(option)
        except ValueError:
            return False
        return n >= 1 and n <= 7

    def valueOk_(self, option, value):
        if (option >= 2 and option <= 6):
            return value == "True" or value == "False"
        return True

    def printMenu_(self):
        print("1) Change Model Path")
        print("2) Display Masks")
        print("3) Display Bounding Boxes")
        print("4) Display Text")
        print("5) Display Probabilities")
        print("6) Dispay FPS")
        print("7) Exit")

    def send_request(self, option, value):
        self.req_ = SetParameters.Request()

        # Create one parameter:

        param = Parameter()

        if (option == 1):
            param.value.type = ParameterType.PARAMETER_STRING
            param.value.string_value = value
        else:
            param.value.type = ParameterType.PARAMETER_BOOL
            v = value == "True"
            param.value.bool_value = v

        if (option == 1):
            param.name = "model_path"
        elif (option == 2):
            param.name = "display_masks"
        elif (option == 3):
            param.name = "display_bboxes"
        elif (option == 4):
            param.name = "display_text"
        elif (option == 5):
            param.name = "display_scores"
        elif (option == 6):
            param.name = "display_fps"

        print(param.name)

        # Append to de list:

        self.req_.parameters.append(param)

        self.future = self.client_srv_.call_async(self.req_)

def main(args=None):
    rclpy.init(args=args)

    client_node = MinimalClientAsync("reconf_params_node")
    # client_node.send_request()

    loop_rate = client_node.create_rate(5)

    try:
        while (rclpy.ok() and not client_node.finish):
            rclpy.spin_once(client_node)
            if (not client_node.req_sended):
                loop_rate.sleep()
                continue
            if (client_node.future.result() is None):
                client_node.get_logger().info(
                    'Service call failed %r' % (client_node.future.exception(),))

            else:
                response = client_node.future.result()
                client_node.get_logger().info(
                    'Result of set parameters: for %s' % (str(response)))
                client_node.future = None
            loop_rate.sleep()
    except KeyboardInterrupt:
        pass

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main(sys.argv)
