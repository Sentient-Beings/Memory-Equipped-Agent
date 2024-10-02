import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from langchain_community.embeddings import HuggingFaceBgeEmbeddings
from groq import Groq
import cv2
import os
import base64
import json
from datetime import datetime
from dotenv import load_dotenv


load_dotenv()
GROQ_API_KEY = os.getenv("GROQ_API_KEY")
HUGGINGFACE_API_KEY = os.getenv("HUGGINGFACE_API_KEY")


if not GROQ_API_KEY:
    raise ValueError("GROQ_API_KEY is not set in the .env file")

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.image_callback_group = MutuallyExclusiveCallbackGroup()
        self.odometry_callback_group = MutuallyExclusiveCallbackGroup()
        
        # embedding model
        self.embedding_model_name = "BAAI/bge-m3"
        self.model_kwargs = {'device': 'cuda'}
        self.encode_kwargs = {"normalize_embeddings": True}
        self.hf = HuggingFaceBgeEmbeddings(
            model_name=self.embedding_model_name, model_kwargs=self.model_kwargs, encode_kwargs=self.encode_kwargs
        )
        
        # subscribers 
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.image_callback_group)
        self.cv_bridge = CvBridge()
        
        self.subscription_2 = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10,
            callback_group=self.odometry_callback_group)
        
        # Initialize Groq client
        self.client = Groq()
        self.get_logger().info('Creating Memories')

        # Timer to control the rate of processing
        self.timer_period = 10.0
        self.timer = self.create_timer(self.timer_period, self.process_image)
        self.latest_image_msg = None
        self.latest_odometry_msg = None
        self.embedding = None
        self.location_memory = {}
        
    def odometry_callback(self, msg):
        self.latest_odometry_msg = msg

    
    def get_location_memory(self):
        self.robot_x = self.latest_odometry_msg.pose.pose.position.x
        self.robot_y = self.latest_odometry_msg.pose.pose.position.y
        now = datetime.now()
        self.location_memory = {
            'Robot x coordinate': self.robot_x,
            'Robot y coordinate': self.robot_y,
            'Timestamp': now.strftime("%Y-%m-%d %H:%M:%S")
        }
        return self.location_memory
        
    def image_callback(self, msg):
        self.latest_image_msg = msg
    
    def embed_memory(self):
        self.get_logger().info('Embedding memory...')
        self.embedding = None
        while self.embedding is None:
            try:
                self.embedding = self.hf.embed_query(self.memory_chunk)
            except Exception as e:
                self.get_logger().error('Error embedding memory: %s' % str(e))
        return self.embedding
	
    def process_image(self):
        if self.latest_image_msg is None:
            self.get_logger().info('No image message to process')
            return

        try:
            self.get_logger().info('Image is being processed...')
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
            self.memory_chunk = json.dumps({**self.structured_output,**self.get_location_memory()}) 
            self.get_logger().info(f'Memory chunk: {self.memory_chunk}')
            self.embedding = self.embed_memory()
            self.get_logger().info(f'Embedding: {self.embedding}')
            self.memory_chunk = None
            
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    executor = MultiThreadedExecutor()
    executor.add_node(image_subscriber)
    try:
        executor.spin()
    finally:
        executor.shutdown() 
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
