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
        super().__init__('sync_data_replay_node')

        ## This sync node aims at synchronizing topics taken from a svo file and a rosbag. Users set up the topics they wish to sync together in a yaml file. 
        ## The rosbag is replayed directly from the node. The SVO is replayed in the ZED ROS wrapper with any camera parameters. Synchronized topics are published by the sync node. 
        ## SVOs are replayed in non realtime, and the replay rate in the wrapper is adjusted 
        ## At any time, users can press the keyboard "space" key to pause the replay of SVO/ ROS BAG and use the right arrow key to advance frame by frame while keeping the rosbag and svo topics synchronized.

        ################################### CLASS PARAMETERS #########################################
        # Get parameters (bag path and launcher param file)
        self.bag_path = self.declare_parameter("bag_path", "").get_parameter_value().string_value
        self.yaml_file = self.declare_parameter("synchronized_topic_config_file", "").get_parameter_value().string_value
        self.namespace = self.declare_parameter("namespace", "").get_parameter_value().string_value
        self.camera_name = self.declare_parameter("camera_name", "").get_parameter_value().string_value
        self.prefix = self.declare_parameter("prefix","").get_parameter_value().string_value

        if self.namespace == "":
            self.namespace = self.camera_name+'/zed_node'
        else:
            self.namespace = self.namespace+'/'+self.camera_name
        self.sync_queue_size = self.declare_parameter("sync_queue_size", 500).get_parameter_value().integer_value
        self.sync_slop = self.declare_parameter("sync_slop", 0.05).get_parameter_value().double_value
        self.seek_time_step = self.declare_parameter("seek_time_step", 0.5).get_parameter_value().double_value

        if not self.bag_path or not self.yaml_file:
            self.get_logger().error("Missing parameters: bag_path or yaml_file")
            sys.exit(1)

        #####################################TOPIC SUBSCRIBERS/PUBLISHERS#############################

        # Initialize publishers and subscribers
        self.sync_subscribers = []
        self.wrapper_subscribers = []
        self.rosbag_topic_publishers = {}
        self.sync_topic_publishers = {}

        ############################################################################################

        ################################### SVO REPLAY #############################################

        # Load topics from ZED wrapper to synchronize (list of topics present in the yaml config file)
        self.load_wrapper_topics(self.yaml_file)

        #Listen to SVO status message 
        self.svo_status_sub = self.create_subscription(SvoStatus,'/'+self.namespace+'/status/svo',self.svo_callback,10)
        self.svo_current_frame = 0
        self.svo_current_timestamp = 0.0
        self.svo_started = False
        
        #Service to modify SVO replay rate
        self.cli = self.create_client(SetParameters, '/'+self.namespace+'/set_parameters')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for'+ '/'+self.namespace+'/set_parameters service...')

        #Service to pause the SVO
        self.svo_pause_client = self.create_client(Trigger, '/'+self.namespace+'/toggle_svo_pause')
        while not self.svo_pause_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service /'+self.namespace+'/toggle_svo_pause...')
        self.pause_request = Trigger.Request()

        #Service to set frame position on SVO
        self.set_svo_frame_position_client = self.create_client(SetSvoFrame, '/'+self.namespace+'/set_svo_frame')
        while not self.set_svo_frame_position_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service'+ '/'+self.namespace+'/set_svo_frame...')
        self.frame_request = SetSvoFrame.Request()

        # SVO Replay Rate parameters
        self.rate = 1.0
        self.kp = 0.002  # Proportional gain

        ############################################################################################


        ################################### ROSBAG REPLAY ##########################################

        # Create rosbag reader
        self.reader = rosbag2_py.SequentialReader()
        storage_options = rosbag2_py.StorageOptions(uri=self.bag_path, storage_id='sqlite3')
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
        self.svo_paused = False 
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

       ################################### KEYBOARD THREAD ########################################

        
        # Start a separate thread to listen for keyboard input
        self.keyboard_thread = threading.Thread(target=self.listen_for_keypress, daemon=True)
        self.keyboard_thread.start()

        ############################################################################################

        ############################################################################################



        # Print initial configuration
        self.get_logger().info("=====================================================")
        self.get_logger().info(f" Node '{self.get_name()}' started")
        self.get_logger().info(f" Namespace: {self.namespace}")
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

    
    def initial_timestamp_match(self):

        ## waiting for SVO to start playing 
        while not self.svo_started:
            self.get_logger().debug("waiting for SVO to start playing...")
        while not self.rosbag_started:
            self.get_logger().debug("waiting for rosbag to start playing...")

        self.get_logger().debug(f"SVO current timestamp : {self.svo_current_timestamp}")
        self.get_logger().debug(f"ROSBAG current timestamp : {self.rosbag_current_timestamp}")

        error = self.rosbag_current_timestamp - self.svo_current_timestamp
        while abs(error)>0.5:
            error = self.rosbag_current_timestamp - self.svo_current_timestamp
            self.get_logger().debug(f"Time difference between SVO and rosbag : {error}")
            if error<0:
                #svo is too advanced, pause it
                if not self.svo_paused:
                    ## Pause SVO 
                    try:
                        self.pause_service_call = self.svo_pause_client.call_async(self.pause_request)
                        response = self.pause_service_call.result()
                        self.get_logger().debug(f"SVO Toggle pause service call")
                    except Exception as e:
                        self.get_logger().error(f'Service call failed: {e}')
                    self.svo_paused = True
            elif error>0:
                #rosbag too advanced, pause it:
                self.rosbag_paused = True
        #When both timestamp matches, unpause SVO if it was paused or rosbag if it was paused
        if self.svo_paused:
            ## Pause SVO 
            try:
                self.pause_service_call = self.svo_pause_client.call_async(self.pause_request)
                response = self.pause_service_call.result()
                self.get_logger().debug(f"SVO Toggle pause service call")
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
            self.svo_paused = False
        if self.rosbag_paused:
            self.rosbag_paused = False
        self.system_initialized = True


    def svo_callback(self, msg):
        """Gives information about the current SVO status. The SVO timestamp is compared with the rosbag timestamp all the time to ensure they are not diverging too much. 
        If the svo is much faster than the rosbag, its replay rate is reduced by adjusting a dynamic ROS wrapper parameter. If the svo is much slower than the rosbag replay rate, its replay rate is increased.""" 
 
        if self.svo_started is False:
            self.svo_started = True
            self.reset_time = True
            self.svo_current_frame = msg.frame_id
            self.svo_current_timestamp = msg.frame_ts* 1e-9
            #Initialize the system with both svo and rosbag having timestamp match
            self.initial_timestamp_match()
            

        
        self.svo_current_frame = msg.frame_id
        self.svo_current_timestamp = msg.frame_ts* 1e-9
        self.get_logger().info(f"SVO current frame : {self.svo_current_frame}")
        self.get_logger().info(f"SVO current timestamp : {self.svo_current_timestamp}")
        self.get_logger().info(f"ROSBAG current timestamp : {self.rosbag_current_timestamp}")
        if self.svo_paused:
            ## if the ROSbag/ SVO replyay is paused, no need to adjust the rate
            return

        ## Calculating difference between both ROSbag / SVO timestamps
        ## The new rate is adjusted based on a PI controller logic whose parameters can be tuned. 
        error = self.rosbag_current_timestamp - self.svo_current_timestamp
        self.get_logger().info(f"Time difference between SVO and rosbag : {error}")
        if abs(error)<0.25:
            self.rate = 1
        adjustment = self.kp * error 
        self.rate += adjustment
        ## Cap rate between [0.1,5]
        self.rate = max(0.1, min(5, self.rate))

        param = Parameter(name='svo.replay_rate', value=self.rate)
        req = SetParameters.Request()
        req.parameters = [param.to_parameter_msg()]

        

        ## As the new rate is calculated, it is sent to the wrapper.

        ##call set SVO replay rate
        try:
          self.rate_service_call = self.cli.call_async(req)
          response = self.rate_service_call.result()
          self.get_logger().debug(f"SVO rate service call")
        except Exception as e:
          self.get_logger().error(f'Service call failed: {e}')
        


    def listen_for_keypress(self):
        """Listen for spacebar press to toggle pause"""
        def on_press(key):
            sleep(0.5)
            if not self.system_initialized:
                return 
            if key == keyboard.Key.space:
                self.keyboard_paused = not self.keyboard_paused
                state = "Paused" if self.keyboard_paused else "Resumed"
                self.get_logger().debug(f"[KEYBOARD] {state}")
                ## Pause/Unpause SVO 
                try:
                  self.pause_service_call = self.svo_pause_client.call_async(self.pause_request)
                  response = self.pause_service_call.result()
                  self.get_logger().debug(f"SVO Toggle pause service call")
                except Exception as e:
                  self.get_logger().error(f'Service call failed: {e}')
                if state == "Resumed":
                     self.reset_time = True
                     self.rosbag_paused = False
                     self.svo_paused = False
                     self.rosbag_has_catched_up = False
                elif state == "Paused":
                     self.rosbag_paused = True
                     self.svo_paused = True
            elif key == keyboard.Key.right:
                ## When the SVO is paused, it is possible to seek by a time step increment when the right arrow key button is pressed.
                if not self.rosbag_has_catched_up:
                    self.seek_rosbag()
                else: 
                    self.seek_if_catchup()
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

    def seek_rosbag(self): 
        """In pause mode, while the svo is still ahead of the rosbag (i.e: the message filter queue contains svos messages ahead of the rosbag), seek on the rosbag by one increment of the seek_time_step parameter until it catches back the svo. Regular synchronization is still possible"""
        if not self.rosbag_paused or not self.svo_paused:
           self.get_logger().info("Rosbag / SVO need to be paused in order for the seek frame feature to work.")
           return
        
        if self.svo_current_timestamp < self.rosbag_current_timestamp:
            ''' the rosbag has catched up with the svo. Seek can now be a simple publisher of data frame by frame.'''
            self.get_logger().info(f"ROSBAG has catched up with the SVO")
            self.rosbag_has_catched_up = True
            return

        target_time = self.rosbag_current_timestamp + self.seek_time_step
        self.get_logger().info(f"ROSBAG current timestamp : {self.rosbag_current_timestamp}")
        self.get_logger().info(f"target timestamp : {target_time}")

        while self.rosbag_current_timestamp < target_time:
            
            if self.svo_current_timestamp < self.rosbag_current_timestamp:
                ''' the rosbag has catched up with the svo. Advance can now be a simple publisher of data frame by frame.'''
                self.get_logger().info(f"ROSBAG has catched up with the SVO")
                self.rosbag_has_catched_up = True
                return

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

        self.get_logger().debug(f"ROSBAG current timestamp : {self.rosbag_current_timestamp}")
        self.get_logger().debug(f"SVO current timestamp : {self.svo_current_timestamp}")

    
    def seek_if_catchup(self): 
        """In pause mode, seek on the rosbag by one increment of the advance_time_step parameter. As the rosbag is now catched up with SVO, advance on the SVO frame by frame to keep both timestamps synchronized. Because the wrapper only sends one message when moving to the next the frame, sync with message filter can be lost. 
        To fix the issue, data is then direclty published in the appropriate topics without message filter.
        """
        if not self.rosbag_paused or not self.svo_paused:
           self.get_logger().info("Rosbag / SVO need to be paused in order for the advance frame feature to work.")
           return

        target_time = self.rosbag_current_timestamp + self.seek_time_step
        self.get_logger().info(f"ROSBAG current timestamp : {self.rosbag_current_timestamp}")
        self.get_logger().info(f"target timestamp : {target_time}")

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
                            self.sync_topic_publishers[topic].publish(msg)
                            #self.get_logger().info(f'publishing rosbag message')
                            
                    except Exception as e:
                        self.get_logger().error(f'Error deserializing message for {topic}: {str(e)}')
            else:
                self.get_logger().info("Rosbag replay complete")

            while self.svo_current_timestamp < self.rosbag_current_timestamp:
                try:
                    self.frame_request.frame_id = self.svo_current_frame +1
                    self.frame_service_call = self.set_svo_frame_position_client.call_async(self.frame_request)
                    self.get_logger().debug(f"SVO frame position service call")
                    self.svo_current_frame = self.svo_current_frame+1
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {e}')
                sleep(0.5)
        self.get_logger().debug(f"ROSBAG current timestamp : {self.rosbag_current_timestamp}")
        self.get_logger().debug(f"SVO current timestamp : {self.svo_current_timestamp}")
            
            
    def synchronized_callback(self, *msgs):
        """Publish synchronized messages"""
        if not msgs:
            return

        self.get_logger().debug(f"Publishing synchronized messages at timestamp {msgs[0].header.stamp.sec}.{msgs[0].header.stamp.nanosec}")

        for topic, msg in zip(self.sync_topic_publishers.keys(), msgs):
            self.sync_topic_publishers[topic].publish(msg)

    def svo_topics_callback(self, msg, topic_name):
        if self.rosbag_has_catched_up:
            self.sync_topic_publishers[topic_name].publish(msg)
            

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

                ##Add to another list of subscribers
                # Create the subscriber
                subscriber_wrapper = self.create_subscription(msg_type, topic["name"], lambda msg, t=topic["name"]: self.svo_topics_callback(msg, t),10)
                self.wrapper_subscribers.append(subscriber_wrapper)

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
            if self.svo_started is False:
                continue

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
