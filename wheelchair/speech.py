#!/usr/bin/python3.10

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import speech_recognition as sr
import threading

class Speech(Node):
    def __init__(self):
        super().__init__('wheelchair')
        self.publisher_ = self.create_publisher(String, 'speech', 10)
        self.recognized = "Nothing"
        timer_period = 0.2
        self.timer = self.create_timer(timer_period, self.timer_callback)
        speech_thread = threading.Thread(target=self.speech)
        speech_thread.start()

    def timer_callback(self):
        msg = String()
        msg.data = self.recognized
        if self.recognized != "Nothing":
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.recognized = "Nothing"

    def speech(self):
        while True:
            try:
                r = sr.Recognizer()
                with sr.Microphone() as source:
                    print("Speak Anything :")
                    audio = r.listen(source)
                    text = r.recognize_google(audio)
                    print("You said : {}".format(text))
                    self.recognized = text
            except Exception as e:
                print(e)

if __name__ == '__main__':
    rclpy.init()
    node = Speech()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
