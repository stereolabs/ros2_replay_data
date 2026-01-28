import rclpy
from rclpy.node import Node
import message_filters
import yaml
from sensor_msgs.msg import *
from diagnostic_msgs.msg import *
from std_msgs.msg import *
from geometry_msgs.msg import *
from zed_msgs.msg import *
from zed_msgs.srv import *
from std_srvs.srv import Trigger
from rosidl_runtime_py.utilities import get_message
import sys
import os
import time
import threading
from pynput import keyboard
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from time import sleep, time
from threading import Thread
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

class DataSynchronizer(Node):
    def __init__(self):
        super().__init__('dual_camera_sync_data_replay_node')

        ## This sync node aims at synchronizing topics taken from a svo file and a rosbag. Users set up the topics they wish to sync together in a yaml file. 
        ## The rosbag is replayed directly from the node. The SVO is replayed in the ZED ROS wrapper with any camera parameters. Synchronized topics are published by the sync node. 
        ## SVOs are replayed in non realtime, and the replay rate in the wrapper is adjusted 
        ## At any time, users can press the keyboard "space" key to pause the replay of SVO/ ROS BAG and use the right arrow key to advance frame by frame while keeping the rosbag and svo topics synchronized.

        ################################### CLASS PARAMETERS #########################################
        # Get parameters (bag path and launcher param file)
        self.bag_path = self.declare_parameter("bag_path", "").get_parameter_value().string_value
        self.yaml_file = self.declare_parameter("synchronized_topic_config_file", "").get_parameter_value().string_value
        self.namespace = self.declare_parameter("namespace", "").get_parameter_value().string_value
        self.camera_name_1 = self.declare_parameter("camera_name_1", "").get_parameter_value().string_value
        self.camera_name_2 = self.declare_parameter("camera_name_2", "").get_parameter_value().string_value
        self.prefix = self.declare_parameter("prefix","").get_parameter_value().string_value
        self.camera_1_namespace = ""
        self.camera_2_namespace = ""
        if self.namespace == "":
            self.camera_1_namespace = self.camera_name_1+'/zed_node'
            self.camera_2_namespace = self.camera_name_2+'/zed_node'
        else:
            self.camera_1_namespace = self.namespace+'/'+self.camera_name_1
            self.camera_2_namespace = self.namespace+'/'+self.camera_name_2
        self.sync_queue_size = self.declare_parameter("sync_queue_size", 500).get_parameter_value().integer_value
        self.sync_slop = self.declare_parameter("sync_slop", 0.05).get_parameter_value().double_value
        self.seek_time_step = self.declare_parameter("seek_time_step", 0.5).get_parameter_value().double_value

        if not self.bag_path or not self.yaml_file:
            self.get_logger().error("Missing parameters: bag_path or yaml_file")
            sys.exit(1)

        #####################################TOPIC SUBSCRIBERS/PUBLISHERS#############################

        # Initialize publishers and subscribers
        self.sync_subscribers = []
        self.rosbag_topic_publishers = {}
        self.sync_topic_publishers = {}

        ############################################################################################

        ################################### SVOs REPLAY #############################################

        # Load topics from ZED wrapper to synchronize (list of topics present in the yaml config file)
        self.load_wrapper_topics(self.yaml_file)

        ########Set Up Svo topics / services

        #####Camera 1
        self.svo1_status_sub = self.create_subscription(SvoStatus,'/'+self.camera_1_namespace+'/status/svo',self.svo1_callback,10)
        self.svo1_current_frame = 0
        self.svo1_current_timestamp = 0.0
        self.svo1_started = False

        #Service to modify SVO replay rate
        self.cli_svo1 = self.create_client(SetParameters, '/'+self.camera_1_namespace+'/set_parameters')

        while not self.cli_svo1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for'+ '/'+self.camera_1_namespace+'/set_parameters service...')

        #Service to pause the SVO
        self.svo1_pause_client = self.create_client(Trigger, '/'+self.camera_1_namespace+'/toggle_svo_pause')
        while not self.svo1_pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /'+self.camera_1_namespace+'/toggle_svo_pause...')
        self.pause_request_svo1 = Trigger.Request()

        #Service to set frame position on SVO
        self.set_svo1_frame_position_client = self.create_client(SetSvoFrame, '/'+self.camera_1_namespace+'/set_svo_frame')
        while not self.set_svo1_frame_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service'+ '/'+self.camera_1_namespace+'/set_svo_frame...')
        self.frame_request_svo1 = SetSvoFrame.Request()

        #####Camera 2
        self.svo2_status_sub = self.create_subscription(SvoStatus,'/'+self.camera_2_namespace+'/status/svo',self.svo2_callback,10)
        self.svo2_current_frame = 0
        self.svo2_current_timestamp = 0.0
        self.svo2_started = False

        #Service to modify SVO replay rate
        self.cli_svo2 = self.create_client(SetParameters, '/'+self.camera_2_namespace+'/set_parameters')

        while not self.cli_svo2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for'+ '/'+self.camera_2_namespace+'/set_parameters service...')

        #Service to pause the SVO
        self.svo2_pause_client = self.create_client(Trigger, '/'+self.camera_2_namespace+'/toggle_svo_pause')
        while not self.svo2_pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /'+self.camera_2_namespace+'/toggle_svo_pause...')
        self.pause_request_svo2 = Trigger.Request()

        #Service to set frame position on SVO
        self.set_svo2_frame_position_client = self.create_client(SetSvoFrame, '/'+self.camera_2_namespace+'/set_svo_frame')
        while not self.set_svo2_frame_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service'+ '/'+self.camera_2_namespace+'/set_svo_frame...')
        self.frame_request_svo2 = SetSvoFrame.Request()

        # SVO Replay Rate parameters
        self.rate_svo1 = 1.0
        self.rate_svo2 = 1.0
        self.kp = 0.02  # Proportional gain

        ############################################################################################

        ################################### KEYBOARD THREAD ########################################

        
        # Start a separate thread to listen for keyboard input
        self.keyboard_thread = threading.Thread(target=self.listen_for_keypress, daemon=True)
        self.keyboard_thread.start()

        ############################################################################################


        ################################### ROSBAG REPLAY ##########################################

        # Create rosbag reader
        self.reader = rosbag2_py.SequentialReader()
        ##Get storage identifier
        info = rosbag2_py.Info()
        metadata = info.read_metadata(self.bag_path, "")
        storage_id = metadata.storage_identifier
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id=storage_id)
        converter_options = rosbag2_py.ConverterOptions('', '')
        self.reader.open(storage_options, converter_options)
        self.rosbag_current_timestamp = 0.0 
        self.rosbag_started = False
        self.rosbag_has_catched_up = False

        # Load topics from ROS Bag to synchronize (list of topics present in the yaml config file)
        self.load_rosbag_topics(self.yaml_file)

        # Get all topic types
        self.topic_types = {topic.name: topic.type for topic in self.reader.get_all_topics_and_types()}

        #Only read topics that are requested 
        topics = list(self.rosbag_topic_publishers.keys())
        self.reader.set_filter(rosbag2_py._storage.StorageFilter(topics=topics))

        # Initialize pause flag
        self.svo1_paused = False 
        self.svo2_paused = False 
        self.rosbag_paused = False
        self.keyboard_paused = False 
        self.system_initialized = False

        # Initialize rosbag rate
        self.rosbag_rate = 1.0 
        self.reset_time = False

         # Launch rosbag playback in a background thread
        self.playback_thread = Thread(target=self.play, daemon=True)
        self.playback_thread.start()

        ############################################################################################

         ################################### DATA SYNC ###############################################
  
        # Synchronize messages with ApproximateTimeSynchronizer
        if self.sync_subscribers:
            self.sync = message_filters.ApproximateTimeSynchronizer(self.sync_subscribers, self.sync_queue_size, self.sync_slop, allow_headerless=True)
            self.sync.registerCallback(self.synchronized_callback)

       ############################################################################################

       ############################################################################################



        # Print initial configuration
        self.get_logger().info("=====================================================")
        self.get_logger().info(f" Node '{self.get_name()}' started")
        self.get_logger().info(f" Namespace Camera 1: {self.camera_1_namespace}")
        self.get_logger().info(f" Namespace Camera 2: {self.camera_2_namespace}")
        self.get_logger().info(f"Opening ROS2 bag: {self.bag_path}")
        self.get_logger().info(f"Opening synchronized topic yaml file: {self.yaml_file}")
        self.get_logger().info(f"Sync queue size: {self.sync_queue_size}")
        self.get_logger().info(f"Sync slop: {self.sync_slop}")
        self.get_logger().info(f"Seek time step increment: {self.seek_time_step}")
        self.get_logger().info(" Keyboard controls:")
        self.get_logger().info("   [Space] Pause/Resume")
        self.get_logger().info("   [→]     Next Rosbag message (when paused)")
        self.get_logger().info("   [↑]     Increase seek time step increment")
        self.get_logger().info("   [↓]     Decrease seek time step increment")
        self.get_logger().info("=====================================================")

        ############################################################################################

    
    def get_qos_profiles_from_bag(self,bag_path, topic_name):
        metadata_file = os.path.join(bag_path, 'metadata.yaml')
        if not os.path.exists(metadata_file):
            raise FileNotFoundError("metadata.yaml not found in bag path")

        with open(metadata_file, 'r') as f:
            metadata = yaml.safe_load(f)

        for topic in metadata['rosbag2_bagfile_information']['topics_with_message_count']:
            name = topic['topic_metadata']['name']
            if name == topic_name:
                qos_profiles_str = topic['topic_metadata']['offered_qos_profiles']
                if not qos_profiles_str:
                    # Common choice: sensor data (LaserScan, Image, Imu...) => BEST_EFFORT, VOLATILE
                    return QoSProfile(
                        depth=10,
                        reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        durability=QoSDurabilityPolicy.VOLATILE
                    )
                qos_profiles = yaml.safe_load(qos_profiles_str)[0]  # Usually a list with one dict
                depth = qos_profiles.get('depth', 10)
                reliability = qos_profiles.get('reliability', 'RELIABLE')
                durability = qos_profiles.get('durability', 'VOLATILE')

                qos_profile = QoSProfile(
                    depth=depth,
                    reliability=reliability,
                    durability=durability
                )

                self.get_logger().debug(f"  Reliability     : {qos_profile.reliability.name}")
                self.get_logger().debug(f"  Durability      : {qos_profile.durability.name}")
                self.get_logger().debug(f"  Depth           : {qos_profile.depth}")
                return qos_profile

        raise ValueError(f"Topic {topic_name} not found in bag metadata.")

    def svo1_callback(self, msg):
        """Gives information about the current SVO status. The SVO timestamp is compared with the rosbag timestamp all the time to ensure they are not diverging too much. 
        If the svo is much faster than the rosbag, its replay rate is reduced by adjusting a dynamic ROS wrapper parameter. If the svo is much slower than the rosbag replay rate, its replay rate is increased.""" 
 
        if self.svo1_started is False:
            self.svo1_started = True
            self.reset_time = True
            self.svo1_current_frame = msg.frame_id
            self.svo1_current_timestamp = msg.frame_ts* 1e-9
            #Initialize the system with both svo and rosbag having timestamp match
            #self.initial_timestamp_match()
            

        
        self.svo1_current_frame = msg.frame_id
        self.svo1_current_timestamp = msg.frame_ts* 1e-9
        #self.get_logger().info(f"SVO 1 current frame : {self.svo1_current_frame}")
        #self.get_logger().info(f"SVO 1  current timestamp : {self.svo1_current_timestamp}")
        #self.get_logger().info(f"ROSBAG current timestamp : {self.rosbag_current_timestamp}")
        if self.svo1_paused:
            ## if the ROSbag/ SVO replyay is paused, no need to adjust the rate
            return

        ## Calculating difference between both ROSbag / SVO timestamps
        ## The new rate is adjusted based on a PI controller logic whose parameters can be tuned. 
        error = self.rosbag_current_timestamp - self.svo1_current_timestamp
        self.get_logger().info(f"Time difference between SVO 1 and rosbag : {error}")
        if abs(error)<0.25:
            self.rate_svo1 = 1
        adjustment = self.kp * error 
        self.rate_svo1 += adjustment
        ## Cap rate between [0.1,5]
        self.rate_svo1 = max(0.1, min(5, self.rate_svo1))
        self.get_logger().info(f"SVO 1 rate : {self.rate_svo1}")
        param = Parameter(name='svo.replay_rate', value=self.rate_svo1)
        req = SetParameters.Request()
        req.parameters = [param.to_parameter_msg()]

        ## As the new rate is calculated, it is sent to the wrapper.

        ##call set SVO replay rate
        try:
          self.rate_service_call = self.cli_svo1.call_async(req)
          response = self.rate_service_call.result()
          self.get_logger().debug(f"SVO 1  rate service call")
        except Exception as e:
          self.get_logger().error(f'Service call failed: {e}')
        

    def svo2_callback(self, msg):
        """Gives information about the current SVO status. The SVO timestamp is compared with the rosbag timestamp all the time to ensure they are not diverging too much. 
        If the svo is much faster than the rosbag, its replay rate is reduced by adjusting a dynamic ROS wrapper parameter. If the svo is much slower than the rosbag replay rate, its replay rate is increased.""" 
 
        if self.svo2_started is False:
            self.svo2_started = True
            self.reset_time = True
            self.svo2_current_frame = msg.frame_id
            self.svo2_current_timestamp = msg.frame_ts* 1e-9
            #Initialize the system with both svo and rosbag having timestamp match
            #self.initial_timestamp_match()
            

        
        self.svo2_current_frame = msg.frame_id
        self.svo2_current_timestamp = msg.frame_ts* 1e-9
        #self.get_logger().info(f"SVO2 current frame : {self.svo2_current_frame}")
        #self.get_logger().info(f"SVO2 current timestamp : {self.svo2_current_timestamp}")
        #self.get_logger().info(f"ROSBAG current timestamp : {self.rosbag_current_timestamp}")
        if self.svo2_paused:
            ## if the ROSbag/ SVO replyay is paused, no need to adjust the rate
            return

        ## Calculating difference between both ROSbag / SVO timestamps
        ## The new rate is adjusted based on a PI controller logic whose parameters can be tuned. 
        error = self.rosbag_current_timestamp - self.svo2_current_timestamp
        self.get_logger().info(f"Time difference between SVO 2 and rosbag : {error}")
        if abs(error)<0.25:
            self.rate_svo2 = 1
        adjustment = self.kp * error 
        self.rate_svo2 += adjustment
        ## Cap rate between [0.1,5]
        self.rate_svo2 = max(0.1, min(5, self.rate_svo2))
        self.get_logger().info(f"SVO 2 rate : {self.rate_svo2}")
        param = Parameter(name='svo.replay_rate', value=self.rate_svo2)
        req = SetParameters.Request()
        req.parameters = [param.to_parameter_msg()]

        

        ## As the new rate is calculated, it is sent to the wrapper.

        ##call set SVO replay rate
        try:
          self.rate_service_call = self.cli_svo2.call_async(req)
          response = self.rate_service_call.result()
          self.get_logger().debug(f"SVO 2 rate service call")
        except Exception as e:
          self.get_logger().error(f'Service call failed: {e}')
        
    def listen_for_keypress(self):
        """Listen for spacebar press to toggle pause"""
        def on_press(key):
            sleep(0.5)
            if key == keyboard.Key.space:
                self.keyboard_paused = not self.keyboard_paused
                state = "Paused" if self.keyboard_paused else "Resumed"
                self.get_logger().info(f"[KEYBOARD] {state}")
                ## Pause/Unpause SVO 1
                try:
                  self.pause_service_call = self.svo1_pause_client.call_async(self.pause_request_svo1)
                  response = self.pause_service_call.result()
                  self.get_logger().debug(f"SVO Toggle pause service call")
                except Exception as e:
                  self.get_logger().error(f'Service call failed: {e}')
                ## Pause/Unpause SVO 2
                try:
                  self.pause_service_call = self.svo2_pause_client.call_async(self.pause_request_svo2)
                  response = self.pause_service_call.result()
                  self.get_logger().debug(f"SVO Toggle pause service call")
                except Exception as e:
                  self.get_logger().error(f'Service call failed: {e}')
                if state == "Resumed":
                     self.reset_time = True
                     self.rosbag_paused = False
                     self.svo1_paused = False
                     self.svo2_paused = False
                     self.rosbag_has_catched_up = False
                elif state == "Paused":
                     self.rosbag_paused = True
                     self.svo1_paused = True
                     self.svo2_paused = True
            elif key == keyboard.Key.right:
                ## When the SVO is paused, it is possible to seek by a time step increment when the right arrow key button is pressed.
                #if not self.rosbag_has_catched_up:
                 #   self.seek_rosbag()
                #else: 
                    #self.seek_if_catchup()
                self.seek()
            elif key == keyboard.Key.up:
                self.seek_time_step = self.seek_time_step + 0.1
                self.get_logger().info(f"Seek time increment has increased to: {self.seek_time_step}")
            elif key == keyboard.Key.down:
                self.seek_time_step = self.seek_time_step - 0.1
                if self.seek_time_step < 0.1:
                    self.seek_time_step = 0.1
                self.get_logger().info(f"Seek time increment has increased to: {self.seek_time_step}")
                
        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

    def seek(self): 
        """In pause mode, check which svo / bag is the less advanced. and use this as reference for target time to advance data."""

        #Define target time, retrieve smallest timestamp
        timestamp = min(self.svo1_current_timestamp, self.svo2_current_timestamp, self.rosbag_current_timestamp)
        target_time = timestamp + self.seek_time_step

        self.get_logger().info(f"ROSBAG current timestamp : {self.rosbag_current_timestamp}")
        self.get_logger().info(f"SVO1 current timestamp : {self.svo1_current_timestamp}")
        self.get_logger().info(f"SVO2 current timestamp : {self.svo2_current_timestamp}")
        self.get_logger().info(f"Target timestamp : {target_time}")

        if not self.rosbag_paused or not self.svo1_paused or not self.svo2_paused:
           self.get_logger().info("Rosbag / SVO need to be paused in order for the seek frame feature to work.")
           return
        
        while self.rosbag_current_timestamp < target_time:
            
            if self.reader.has_next():
                topic, data, t = self.reader.read_next()
                if topic in self.topic_types:
                    try:
                        msg_type = self.topic_types[topic]
                        msg_class = get_message(msg_type)
                        msg = deserialize_message(data, msg_class)
                        self.rosbag_current_timestamp = t * 1e-9

                        if topic in self.rosbag_topic_publishers:
                            self.rosbag_topic_publishers[topic].publish(msg)
                            
                    except Exception as e:
                        self.get_logger().error(f'Error deserializing message for {topic}: {str(e)}')
            else:
                self.get_logger().info("Rosbag replay complete")
        
        while self.svo1_current_timestamp < target_time or self.svo2_current_timestamp < target_time:

            if self.svo1_current_timestamp < target_time:
                self.get_logger().debug(f"SVO1 current timestamp : {self.svo1_current_timestamp}")
                try:
                    self.frame_request_svo1.frame_id = self.svo1_current_frame +1
                    self.frame_service_call = self.set_svo1_frame_position_client.call_async(self.frame_request_svo1)
                    self.get_logger().debug(f"SVO frame position service call")
                    self.svo1_current_frame = self.svo1_current_frame+1
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
                sleep(0.25)
            

            if self.svo2_current_timestamp < target_time:
                try:
                    self.frame_request_svo2.frame_id = self.svo2_current_frame +1
                    self.frame_service_call = self.set_svo2_frame_position_client.call_async(self.frame_request_svo2)
                    self.get_logger().debug(f"SVO frame position service call")
                    self.svo2_current_frame = self.svo2_current_frame+1
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
                sleep(0.25)

        self.get_logger().info(f"AFTER SEEK ROSBAG current timestamp : {self.rosbag_current_timestamp}")
        self.get_logger().info(f"AFTER SEEK SVO1 current timestamp : {self.svo1_current_timestamp}")
        self.get_logger().info(f"AFTER SEEK SVO2 current timestamp : {self.svo2_current_timestamp}")
        self.get_logger().info(f"Target timestamp : {target_time}")
            
    def synchronized_callback(self, *msgs):
        """Publish synchronized messages"""
        if not msgs:
            return

        self.get_logger().debug(f"Publishing synchronized messages at timestamp {msgs[0].header.stamp.sec}.{msgs[0].header.stamp.nanosec}")

        for topic, msg in zip(self.sync_topic_publishers.keys(), msgs):
            self.sync_topic_publishers[topic].publish(msg)

    def load_wrapper_topics(self, yaml_file):
        """ Load topics from YAML file """
        try:
            with open(yaml_file, "r") as file:
                data = yaml.safe_load(file)
                topic_list = data.get("sync_node", {}).get("zed_wrapper_topics", [])
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file {yaml_file}: {str(e)}")
            return 

        if len(topic_list) == 0:
            self.get_logger().error(f"No ZED Wrapper topics found to synchronize, exiting...")
            sys.exit(0) 
            return

        # Dynamically create subscribers and publishers
        for topic in topic_list:
            try:
                msg_type = get_message(topic["type"])
                subscriber = message_filters.Subscriber(self, msg_type, topic["name"])
                self.sync_subscribers.append(subscriber)

                if self.prefix == "":
                    topic_name = '/sync'+topic["name"]
                else:
                    topic_name = topic["name"].removeprefix(self.prefix)
                publisher = self.create_publisher(msg_type, topic_name, 10)
                self.sync_topic_publishers[topic["name"]] = publisher

                self.get_logger().info(f"Created publisher for {topic['name']} [{topic['type']}]")

            except Exception as e:
                self.get_logger().error(f"Failed to create publisher for {topic['name']}: {str(e)}")
        
        return

    def load_rosbag_topics(self, yaml_file):
        """ Load rosbag topics from YAML file """
        try:
            with open(yaml_file, "r") as file:
                data = yaml.safe_load(file)
                topic_list = data.get("sync_node", {}).get("rosbag_topics", [])
        except Exception as e:
            self.get_logger().error(f"Failed to load YAML file {yaml_file}: {str(e)}")
            return 

        if len(topic_list) == 0:
            self.get_logger().error(f"No ROSbag topics found to synchronize, exiting...")
            sys.exit(0)
            return 

        # Dynamically create subscribers and publishers
        for topic in topic_list:
            try:
                msg_type = get_message(topic["type"])
                subscriber = message_filters.Subscriber(self, msg_type, "/bag"+topic["name"])
                self.sync_subscribers.append(subscriber)
                qos = self.get_qos_profiles_from_bag(self.bag_path, topic["name"])
                publisher = self.create_publisher(msg_type, topic["name"], qos)
                self.sync_topic_publishers[topic["name"]] = publisher
                publisher = self.create_publisher(msg_type, "/bag"+topic["name"], qos)
                self.rosbag_topic_publishers[topic["name"]] = publisher
                self.get_logger().info(f"Created publisher for {topic['name']} [{topic['type']}]")

            except Exception as e:
                self.get_logger().error(f"Failed to create publisher for {topic['name']}: {str(e)}")
        
        return

    def play(self):
        first_msg_time = None
        base_time = time()

        while self.reader.has_next():
            if self.svo1_started is False or self.svo2_started is False:
                continue

            error = max(self.rosbag_current_timestamp - self.svo2_current_timestamp, self.rosbag_current_timestamp - self.svo1_current_timestamp)
            if error > 3:
                self.rosbag_paused = True
            else:
                if self.rosbag_paused:
                    self.reset_time = True
                    self.rosbag_paused = False

            if self.rosbag_paused:
                continue 

            topic, data, t = self.reader.read_next()
            # Time control
            if first_msg_time is None or self.reset_time is True:
                print("reset time")
                first_msg_time = t
                base_time = time()
                self.reset_time = False
            else:
                elapsed_ros_time = (t - first_msg_time) / 1e9
                self.get_logger().debug(f'elapsed ros time: {elapsed_ros_time}')
                target_wall_time = base_time + elapsed_ros_time / self.rosbag_rate
                self.get_logger().debug(f'target_wall time: {target_wall_time}')
                self.get_logger().debug(f'curent time: {time()}')
                sleep(max(0, target_wall_time - time()))
            #Publish data
            if topic in self.topic_types:
                try:
                    msg_type = self.topic_types[topic]
                    msg_class = get_message(msg_type)
                    msg = deserialize_message(data, msg_class)
                    if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
                      stamp = msg.header.stamp
                      self.rosbag_current_timestamp = stamp.sec + stamp.nanosec * 1e-9
                    if topic in self.rosbag_topic_publishers:
                        self.rosbag_topic_publishers[topic].publish(msg)

                except Exception as e:
                    self.get_logger().error(f'Error deserializing message for {topic}: {str(e)}')
            if not self.rosbag_started:
                self.rosbag_started = True

        if not self.reader.has_next():
            self.get_logger().info("Rosbag replay complete. Shutting down node.")
            return

    
def main(args=None):
    rclpy.init(args=args)
    
    try:
        sync_node = DataSynchronizer()
        rclpy.spin(sync_node)
    except KeyboardInterrupt:
        if sync_node is not None:
            sync_node.get_logger().info("Shutting down due to KeyboardInterrupt...")
    finally:
        # Make sure the node is destroyed only if context is still valid
        if rclpy.ok():
            sync_node.get_logger().info("Shutting down cleanly...")
            sync_node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
