# ros related imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import cv2

# langchain related imports
from langchain_community.embeddings import HuggingFaceBgeEmbeddings
from uuid import uuid4
from langchain_core.documents import Document
from langchain_pinecone import PineconeVectorStore

# model related imports
from groq import Groq
import os
import base64
import json
from datetime import datetime
from dotenv import load_dotenv

# pinecone related imports
from pinecone import Pinecone, ServerlessSpec
import time


load_dotenv()
GROQ_API_KEY = os.getenv("GROQ_API_KEY")
HUGGINGFACE_API_KEY = os.getenv("HUGGINGFACE_API_KEY")
PINECONE_API_KEY = os.getenv("PINECONE_API_KEY")


if not GROQ_API_KEY:
    raise ValueError("GROQ_API_KEY is not set in the .env file")
if not HUGGINGFACE_API_KEY:
    raise ValueError("HUGGINGFACE_API_KEY is not set in the .env file")
if not PINECONE_API_KEY:
    raise ValueError("PINECONE_API_KEY is not set in the .env file")

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.image_callback_group = MutuallyExclusiveCallbackGroup()
        self.odometry_callback_group = MutuallyExclusiveCallbackGroup()
        
        # embedding model
        self.get_logger().info('Initializing embedding model...')
        self.embedding_model_name = "BAAI/bge-m3"
        self.model_kwargs = {'device': 'cuda'}
        self.encode_kwargs = {"normalize_embeddings": True}
        self.hf_embedding = HuggingFaceBgeEmbeddings(
            model_name=self.embedding_model_name, model_kwargs=self.model_kwargs, encode_kwargs=self.encode_kwargs
        )
        
        # pinecone serverless 
        self.get_logger().info('Initializing pinecone serverless...')
        pc_store = Pinecone(api_key=PINECONE_API_KEY)
        self.index_name = "rag-for-robots"
        self.existing_indexes = [index_info["name"] for index_info in pc_store.list_indexes()]
        if self.index_name not in self.existing_indexes:
            pc_store.create_index(
                name=self.index_name,
                dimension=1024,
                metric="cosine",
                spec=ServerlessSpec(cloud="aws", region="us-east-1"),
            )
            while not pc_store.describe_index(self.index_name).status["ready"]:
                time.sleep(1)
        self.index = pc_store.Index(self.index_name)
        self.vector_store = PineconeVectorStore(index=self.index, embedding=self.hf_embedding)
        
        # subscribers 
        self.get_logger().info('Initializing subscribers...')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.image_callback_group)
        self.cv_bridge = CvBridge()
        
        self.subscription_2 = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.odometry_callback,
            10,
            callback_group=self.odometry_callback_group)
        
        # Initialize Groq client
        self.get_logger().info('Initializing Groq client...')
        self.client = Groq()

        # Timer to control the rate of processing
        self.timer_period = 10.0
        self.timer = self.create_timer(self.timer_period, self.process_image)
        self.latest_image_msg = None
        self.latest_odometry_msg = None
        self.embedding = None
        self.location_memory = {}
        self.get_logger().info('Creating Memories')
        
    def odometry_callback(self, msg):
        '''
        This function is called whenever the robot's odometry is updated.
        '''
        self.latest_odometry_msg = msg
    
    def get_location_memory(self):
        '''
        This function is used to get the current location of the robot and store it in the location_memory dictionary.
        Later, this location_memory dictionary will be used to create embeddings.
        '''
        self.robot_x = self.latest_odometry_msg.pose.pose.position.x
        self.robot_y = self.latest_odometry_msg.pose.pose.position.y
        now = datetime.now()
        self.location_memory = {
            'Robot x coordinate': self.robot_x + 1,
            'Robot y coordinate': self.robot_y + 1,
            'Timestamp': now.strftime("%Y-%m-%d %H:%M:%S")
        }
        return self.location_memory
        
    def image_callback(self, msg):
        '''
        This function is called whenever a new image is received.
        '''
        self.latest_image_msg = msg
    
    def embed_and_save_memory(self):
        '''
        This function is used to embed the memory chunk and save it to the vector store.
        '''
        self.get_logger().info('Embedding and saving memory...')
        if self.memory_chunk is not None:
            memory_chunk = Document(
                page_content=self.memory_chunk,
                metadata={"source": "visited location"},)
            documents = [memory_chunk]
            uuids = [str(uuid4())]
            '''
            get similar chunks with respect to my chunk 
            compare the similarity score with all the retrieved chunks ( according to the threshold)
            remove the old one and store the new one 
            '''
            self.vector_store.add_documents(documents=documents, ids=uuids)
            self.get_logger().info('Memory chunk saved')
            
    def process_image(self):
        '''
        This function is used to process the image and create a memory chunk.
        '''
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
            self.embed_and_save_memory()
            self.memory_chunk = None
            
        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

def main(args=None):
    '''
    This is the main function that initializes the ROS node and starts the processing loop.
    '''
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
