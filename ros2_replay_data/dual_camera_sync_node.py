import rclpy
from rclpy.node import Node
import yaml
import threading
from time import sleep, time
from pynput import keyboard
import os
from sensor_msgs.msg import *
from zed_msgs.msg import *
from zed_msgs.srv import *
from std_srvs.srv import Trigger
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import rosbag2_py

class DataSynchronizer(Node):
    def __init__(self):
        super().__init__('dual_camera_sync_node')

        # --- Parameters ---
        self.bag_path = self.declare_parameter("bag_path", "").value
        self.yaml_file = self.declare_parameter("synchronized_topic_config_file", "").value
        self.namespace = self.declare_parameter("namespace", "").value
        self.camera_name_1 = self.declare_parameter('camera_name_1', 'zed1').value
        self.camera_name_2 = self.declare_parameter('camera_name_2', 'zed1').value
        self.seek_time_step = self.declare_parameter("seek_time_step", 0.5).value
        self.prefix = self.declare_parameter("prefix","").value

        self.camera_1_ns = ""
        self.camera_2_ns = ""

        if self.namespace == "":
            self.camera_1_ns = self.camera_name_1+'/zed_node'
            self.camera_2_ns = self.camera_name_2+'/zed_node'
        else:
            self.camera_1_ns = self.namespace+'/'+self.camera_name_1
            self.camera_2_ns = self.namespace+'/'+self.camera_name_2
            
        
        
        self.sync_slop = self.declare_parameter("sync_slop", 0.05).value # 50ms window
        self.sync_buffer_time = self.declare_parameter("sync_buffer_time", 2.0).value # 2.0s buffer window

        # --- Data Structures ---
        # topic_name -> [(timestamp, message_object), ...]
        self.topic_buffers = {} 
        self.buffer_lock = threading.Lock()
        
        self.svo_status = {
            'cam1': {'ts': 0.0, 'frame': 0, 'started': False},
            'cam2': {'ts': 0.0, 'frame': 0, 'started': False}
        }
        
        self.rosbag_ts = 0.0
        self.is_paused = False
        self.reset_playback_clock = False

        # --- Initialize Components ---
        self.sync_publishers = {}
        self.rosbag_publishers = {}
        
        
        # --- Initialize Rosbag ---
        self.reader = rosbag2_py.SequentialReader()
        info = rosbag2_py.Info()
        self.metadata = info.read_metadata(self.bag_path, "")
        self.reader.open(
            rosbag2_py.StorageOptions(uri=self.bag_path, storage_id=self.metadata.storage_identifier),
            rosbag2_py.ConverterOptions('', '')
        )
        self.bag_types = {t.name: t.type for t in self.reader.get_all_topics_and_types()}

        # --- Initialize Clients ---
        self.init_svo_clients()

        # --- Load Configs ---
        self.load_config(self.yaml_file)

        # --- Background Threads ---
        threading.Thread(target=self.playback_loop, daemon=True).start()
        threading.Thread(target=self.keyboard_controller, daemon=True).start()

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
                reliability = qos_profiles.get('reliability', 'BEST_EFFORT')
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

    def load_config(self, yaml_file):
        with open(yaml_file, 'r') as f:
            config = yaml.safe_load(f)
        
        # 1. Setup SVO Topics (Sub from Wrapper -> Buffer -> Sync Publish)
        for topic in config['sync_node'].get('zed_wrapper_topics', []):
            m_type = get_message(topic['type'])

            if self.prefix == "":
                t_name = '/sync'+topic["name"]
            else:
                t_name = topic["name"].removeprefix(self.prefix)
            
            # Buffer the incoming wrapper data
            self.topic_buffers[t_name] = []
            self.create_subscription(m_type, topic['name'], 
                lambda msg, tn=t_name: self._buffer_cb(tn, msg), 10)
            
            # Output synced topic
            self.sync_publishers[t_name] = self.create_publisher(m_type, f"{t_name}", 10)

        # 2. Setup Rosbag Topics
        for topic_info in self.metadata.topics_with_message_count:
            topic_name = topic_info.topic_metadata.name
            topic_type = topic_info.topic_metadata.type
    
            try:
                # Dynamically import the message type
                m_type = get_message(topic_type)

                #Retrieve Qos Profile
                qos = self.get_qos_profiles_from_bag(self.bag_path, topic_name)
        
                # Create and store the publisher
                self.rosbag_publishers[topic_name] = self.create_publisher(
                    m_type, 
                    topic_name, 
                    qos
                )
                self.get_logger().info(f"Created publisher for {topic_name} [{topic_type}]")
        
            except (ImportError, AttributeError) as e:
                self.get_logger().error(f"Could not import message type {topic_type}: {e}")

    def _buffer_cb(self, topic_name, msg):
        """Standardizes all incoming camera data into a time-sorted buffer."""
        if not hasattr(msg, 'header'): return
        ts = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        with self.buffer_lock:
            self.topic_buffers[topic_name].append((ts, msg))
            
            while len(self.topic_buffers[topic_name]) > 0 and (ts - self.topic_buffers[topic_name][0][0] > self.sync_buffer_time):
                self.topic_buffers[topic_name].pop(0)

    def init_svo_clients(self):
        # Using naming convention from your previous code

        self.create_subscription(SvoStatus, f"{self.camera_1_ns}/status/svo", lambda m: self._svo_status_cb('cam1', m, self.camera_1_ns), 10)
        self.create_subscription(SvoStatus, f"{self.camera_2_ns}/status/svo", lambda m: self._svo_status_cb('cam2', m, self.camera_2_ns), 10)
        
        self.cli_rate = {
            'cam1': self.create_client(SetParameters, f"{self.camera_1_ns}/set_parameters"),
            'cam2': self.create_client(SetParameters, f"{self.camera_2_ns}/set_parameters")
        }

        self.cli_svo1_seek = self.create_client(SetSvoFrame, f"{self.camera_1_ns}/set_svo_frame")
        self.cli_svo2_seek = self.create_client(SetSvoFrame, f"{self.camera_2_ns}/set_svo_frame")

        self.cli_svo1_pause = self.create_client(Trigger, '/'+self.camera_1_ns+'/toggle_svo_pause')
        self.cli_svo2_pause = self.create_client(Trigger, '/'+self.camera_2_ns+'/toggle_svo_pause')

    def _svo_status_cb(self, cam_id, msg, ns):
        self.svo_status[cam_id] = {'ts': msg.frame_ts * 1e-9, 'frame': msg.frame_id, 'started': True}
        
        # Adjust SVO rate to catch up to Rosbag
        if not self.is_paused and self.rosbag_ts > 0:
            error = self.rosbag_ts - self.svo_status[cam_id]['ts']
            # Simple P-control: base rate 1.0 + (error * gain)
            target_rate = max(0.2, min(4.0, 1.0 + (error * 1.5)))
            req = SetParameters.Request()
            req.parameters = [Parameter(name='svo.replay_rate', value=target_rate).to_parameter_msg()]
            self.cli_rate[cam_id].call_async(req)

    def playback_loop(self):
        """Master clock thread with Drift Guard."""
        first_bag_ts = None
        wall_start = time()
        
        # Max allowed lead (in seconds) before the Rosbag pauses to wait for SVOs
        MAX_DRIFT = 0.25 

        while rclpy.ok():
            # 1. Manual Pause or Waiting for Initialization
            if self.is_paused or not all(c['started'] for c in self.svo_status.values()):
                sleep(0.05)
                continue

            # 2. DRIFT GUARD: Check if Rosbag is too far ahead of SVOs
            # We look at the slowest camera to determine if we should wait
            min_svo_ts = min(self.svo_status['cam1']['ts'], self.svo_status['cam2']['ts'])
            
            if self.rosbag_ts - min_svo_ts > MAX_DRIFT:
                self.get_logger().warn(
                    f"Rosbag is {self.rosbag_ts - min_svo_ts:.2f}s ahead. Waiting for SVOs...",
                    throttle_duration_sec=1.0
                )
                self.reset_playback_clock = True # Reset timing baseline after waiting
                sleep(0.1) 
                continue

            if not self.reader.has_next(): 
                self.get_logger().info("End of Rosbag reached.")
                break

            # 3. Read Next Rosbag Message
            topic, data, t_nanos = self.reader.read_next()
            msg_class = get_message(self.bag_types[topic])
            msg = deserialize_message(data, msg_class)
            self.rosbag_ts = t_nanos * 1e-9

            # 4. Real-time Pacing Logic
            if first_bag_ts is None or self.reset_playback_clock:
                first_bag_ts = self.rosbag_ts
                wall_start = time()
                self.reset_playback_clock = False
            
            elapsed_bag_time = self.rosbag_ts - first_bag_ts
            elapsed_wall_time = time() - wall_start
            
            wait_time = elapsed_bag_time - elapsed_wall_time
            if wait_time > 0:
                sleep(wait_time)

            # 5. Publish Rosbag Topic
            if topic in self.rosbag_publishers:
                self.rosbag_publishers[topic].publish(msg)

            # 6. Synchronize and Publish SVO Topics
            # This looks into the buffers for messages closest to 'self.rosbag_ts'
            self.sync_and_publish_all(self.rosbag_ts)

    def sync_and_publish_all(self, target_ts):
        """Searches all SVO buffers for the closest match to the current Bag time."""
        with self.buffer_lock:
            for topic_name, buffer in self.topic_buffers.items():
                if not buffer: continue
                
                # Find index of closest timestamp
                best_idx = min(range(len(buffer)), key=lambda i: abs(buffer[i][0] - target_ts))
                diff = abs(buffer[best_idx][0] - target_ts)
                
                self.get_logger().debug(f"topic name: {topic_name}")
                self.get_logger().debug(f"diff: {diff}")
                if diff <= self.sync_slop:
                    msg_to_pub = buffer[best_idx][1]
                    self.sync_publishers[topic_name].publish(msg_to_pub)



    def step_forward(self):
        """Advances Bag and SVOs to a Target Time (Current + Increment)."""
        if not self.is_paused:
            self.get_logger().warn("Seek only works when paused.")
            return

        # 1. Define Target Time
        # We base it on the smallest current timestamp to ensure no source is left behind
        current_min_ts = min(self.svo_status['cam1']['ts'], 
                             self.svo_status['cam2']['ts'], 
                             self.rosbag_ts)
        
        target_ts = current_min_ts + self.seek_time_step
        self.get_logger().info(f"Seeking to target: {target_ts:.3f} (+{self.seek_time_step}s)")

        # 2. Advance Rosbag to Target
        # We keep reading messages until we reach or slightly exceed the target time
        while self.rosbag_ts < target_ts and self.reader.has_next():
            topic, data, t_nanos = self.reader.read_next()
            self.rosbag_ts = t_nanos * 1e-9
            
            # Publish messages as we go so the state updates (e.g. TF, Odom)
            msg_class = get_message(self.bag_types[topic])
            msg = deserialize_message(data, msg_class)
            if topic in self.rosbag_publishers:
                self.rosbag_publishers[topic].publish(msg)

        # 3. Advance SVOs to Target
        # We calculate how many frames to jump based on a standard 30fps, 
        # or we can simply increment. To be precise, we call the seek service.
        for cam_id in ['cam1', 'cam2']:
            
            while self.svo_status[cam_id]['ts'] < target_ts:
            
                target_frame = self.svo_status[cam_id]['frame'] + 1
                req = SetSvoFrame.Request()
                req.frame_id = target_frame
                
                client = self.cli_svo1_seek if cam_id == 'cam1' else self.cli_svo2_seek
                client.call_async(req)
                self.get_logger().info(f"Jumping {cam_id} to frame {target_frame}")

                # 4. Final Sync Flush
                # Give the SVO wrapper a moment to publish the frames at the new position
                sleep(0.2) 
                self.sync_and_publish_all(self.rosbag_ts)
        self.get_logger().info("Seek complete.")

    def keyboard_controller(self):
        """Keyboard listener handling seek and increment adjustment."""
        def on_press(key):
            try:
                if key == keyboard.Key.space:
                    self.is_paused = not self.is_paused
                    self.cli_svo1_pause.call_async(Trigger.Request())
                    self.cli_svo2_pause.call_async(Trigger.Request())
                    if not self.is_paused: self.reset_playback_clock = True
                    self.get_logger().info(f"System {'PAUSED' if self.is_paused else 'RESUMED'}")
                
                elif key == keyboard.Key.right and self.is_paused:
                    self.step_forward()

                elif key == keyboard.Key.up:
                    self.seek_time_step += 0.1
                    self.get_logger().info(f"Seek step: {self.seek_time_step:.1f}s")

                elif key == keyboard.Key.down:
                    self.seek_time_step = max(0.1, self.seek_time_step - 0.1)
                    self.get_logger().info(f"Seek step: {self.seek_time_step:.1f}s")

            except Exception as e:
                self.get_logger().error(f"Keyboard handling error: {e}")

        with keyboard.Listener(on_press=on_press) as listener:
            listener.join()

def main():
    rclpy.init()
    node = DataSynchronizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()