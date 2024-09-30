import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from dotenv import load_dotenv
import os
import base64
from groq import Groq
import json

load_dotenv()
GROQ_API_KEY = os.getenv("GROQ_API_KEY")

if not GROQ_API_KEY:
    raise ValueError("GROQ_API_KEY is not set in the .env file")

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.cv_bridge = CvBridge()
        
        # Initialize Groq client
        self.client = Groq()
        self.get_logger().info('Image subscriber node has been initialized')

        # Timer to control the rate of processing
        self.timer_period = 20.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.process_image)
        self.latest_image_msg = None

    def image_callback(self, msg):
        self.get_logger().info('Received image message')
        self.latest_image_msg = msg

    def process_image(self):
        if self.latest_image_msg is None:
            self.get_logger().info('No image message to process')
            return

        try:
            self.get_logger().info('Processing image')
            # image process block
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image_msg, "bgr8")
            _, buffer = cv2.imencode('.jpg', cv_image)
            img_base64 = base64.b64encode(buffer).decode('utf-8')
            
            # the more structured the prompt, the better 
            user_prompt='''
                You are an image captioning API which specializes in extracting the single most prominent object in the image and providing a respone in JSON. What is the caption for for this image ?
                Return in the following JSON format:
                {
                "caption": "string (e.g., 'a speed limit sign 16 km/h', 'a bicycle', 'a red car' , 'a box with 7 written on it')"
                }
                '''
            message_content = [
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{img_base64}",
                    },
                },
                {"type": "text", "text": user_prompt},
            ]
            
            self.get_logger().info('Sending image to Groq API for captioning')
            chat_completion = self.client.chat.completions.create(
                model="llama-3.2-11b-vision-preview",
                messages=[
                    {
                        "role": "user",
                        "content": message_content,
                    }
                ],
                temperature=1,
                max_tokens=1024,
                top_p=1,
                stream=False,
                response_format={"type": "json_object"},
                stop=None,
            )
            
            self.structured_output = json.loads(chat_completion.choices[0].message.content)
            self.get_logger().info('Structured output: %s' % self.structured_output)

        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
