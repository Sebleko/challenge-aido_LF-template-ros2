import rclpy
import threading
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time

import time
import copy
import logging
import os
import numpy as np
import yaml


from std_msgs.msg import String
from duckietown_msgs.msg import EpisodeStart, WheelEncoderStamped, WheelsCmdStamped
from sensor_msgs.msg import CameraInfo, CompressedImage

from aido_schemas import (
    Context,
    DB20Commands,
    DB20ObservationsWithTimestamp,
    EpisodeStart,
    GetCommands,
    LEDSCommands,
    protocol_agent_DB20_timestamps,
    PWMCommands,
    RGB,
    wrap_direct,
)

class DuckietownBridgeNode(Node): 
    def __init__(self):
        super().__init__('duckietown_bridge')
       
         # Get the vehicle name, which comes in as HOSTNAME
        self.vehicle_name = os.getenv("VEHICLE_NAME")
        topic = f"/{self.vehicle_name}/wheels_driver_node/wheels_cmd"
        self.ik_action_sub = self.create_subscription(
            WheelsCmdStamped, 
            topic, 
            self._ik_action_cb
        )
        # Initialize action. 
        # This will be updated by the _ik_action_cb.
        self.action = np.array([0.0, 0.0])
        self.updated = True
        self.initialized = False

        # TODO: listen to the LED topics

        # Publishes onto the corrected image topic
        # since image out of simulator is currently rectified
        # topic = "/{}/image_topic".format(self.vehicle_name)
        topic = f"/{self.vehicle_name}/camera_node/image/compressed"
        self.cam_pub = self.create_publisher(CompressedImage, topic, 10)

        # Publisher for camera info - needed for the ground_projection
        # topic = "/{}/camera_info_topic".format(self.vehicle_name)
        topic = f"/{self.vehicle_name}/camera_node/camera_info"
        self.cam_info_pub = self.create_publisher(CameraInfo, topic, 1)

        episode_start_topic = f"/{self.vehicle_name}/episode_start"        
        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_publisher(EpisodeStart, episode_start_topic, latching_qos)

        # copied from camera driver:

        left_encoder_topic = "/{}/left_wheel_encoder_node/tick".format(self.vehicle_name)
        self.left_encoder_pub = self.create_publisher(WheelEncoderStamped, left_encoder_topic, 1)
        right_encoder_topic = "/{}/right_wheel_encoder_node/tick".format(self.vehicle_name)
        self.right_encoder_pub = self.create_publisher(WheelEncoderStamped, right_encoder_topic, 1)

        # For intrinsic calibration
        self.cali_file_folder = "/data/config/calibrations/camera_intrinsic/"
        self.frame_id = self.get_namespace().strip("/") + "/camera_optical_frame"
        self.cali_file = self.cali_file_folder + f"{self.vehicle_name}.yaml"

        # Locate calibration yaml file or use the default otherwise
        if not os.path.isfile(self.cali_file):
            self.get_logger().warn(f"Calibration not found: {self.cali_file}.\n Using default instead.")
            self.cali_file = self.cali_file_folder + "default.yaml"

        # Shutdown if no calibration file not found
        if not os.path.isfile(self.cali_file):
            self.get_logger().error("Found no calibration file. Aborting")
            rclpy.shutdown()
            return

        # Load the calibration file
        self.original_camera_info = self.load_camera_info(self.cali_file)
        self.original_camera_info.header.frame_id = self.frame_id
        self.current_camera_info = copy.deepcopy(self.original_camera_info)
        self.get_logger().info(f"Using calibration file: {self.cali_file}")


        log_path = "/challenges/challenge-solution-output"
        if os.path.exists(log_path):
            fh = logging.FileHandler(f"{log_path}/rosagent-after-init_node.log")
            fh.setLevel(logging.DEBUG)
            # create console handler with a higher log level
            ch = logging.StreamHandler()
            ch.setLevel(logging.DEBUG)
            # create formatter and add it to the handlers
            # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            # fh.setFormatter(formatter)
            # ch.setFormatter(formatter)
            # add the handlers to the logger
            root = logging.getLogger()
            root.addHandler(fh)
            root.addHandler(ch)
            #
        self.get_logger().info("Bridge node __init__ finished.")



    def _ik_action_cb(self, msg):
        """
        Callback to listen to last outputted action from inverse_kinematics node
        Stores it and sustains same action until new message published on topic
        """

        self.initialized = True
        vl = msg.vel_left
        vr = msg.vel_right
        self.action = np.array([vl, vr])
        self.updated = True


    def publish_info(self, timestamp: float):
        """
        Publishes a default CameraInfo - TODO: Fix after distortion applied in simulator
        """
        # Publish the CameraInfo message
        stamp = Time(seconds=timestamp)
        self.current_camera_info.header.stamp = stamp
        self.cam_info_pub.publish(self.current_camera_info)

    def publish_episode_start(self, episode_name: str, payload_yaml: str):
        episode_start_message = EpisodeStart()
        episode_start_message.episode_name = episode_name
        episode_start_message.other_payload_yaml = payload_yaml
        self.episode_start_pub.publish(episode_start_message)


    def publish_img(self, obs: bytes, timestamp: float):
        """
        Publishes the image to the compressed_image topic.
        """

        # XXX: make this into a function (there were a few of these conversions around...)
        img_msg = CompressedImage()
       
        img_msg.header.stamp = Time(seconds=timestamp)

        img_msg.format = "jpeg"
        img_msg.data = obs

        self.cam_pub.publish(img_msg)


    def publish_odometry(self, resolution_rad: float, left_rad: float, right_rad: float, timestamp: float):
        """
        :param timestamp:
        :param resolution_rad:
        :param left_rad:
        :param right_rad:
        :return: none
        """
        if resolution_rad == 0:
            self.get_logger().error("Can't interpret encoder data with resolution 0")
        stamp = Time(seconds=timestamp)
        msg = WheelEncoderStamped(
            data=int(np.round(left_rad / resolution_rad)),
            resolution=int(np.round(np.pi * 2 / resolution_rad)),
            type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL,
        )
        msg.header.stamp = stamp

        self.left_encoder_pub.publish(msg)

        msg = WheelEncoderStamped(
            data=int(np.round(right_rad / resolution_rad)),
            resolution=int(np.round(np.pi * 2 / resolution_rad)),
            type=WheelEncoderStamped.ENCODER_TYPE_INCREMENTAL,
        )
        msg.header.stamp = stamp
        self.right_encoder_pub.publish(msg)


    @staticmethod
    def load_camera_info(filename):
        """Loads the camera calibration files.

        Loads the intrinsic and extrinsic camera matrices.

        Args:
            filename (:obj:`str`): filename of calibration files.

        Returns:
            :obj:`CameraInfo`: a CameraInfo message object

        """
        with open(filename, "r") as stream:
            calib_data = yaml.load(stream, Loader=yaml.Loader)
        cam_info = CameraInfo()
        cam_info.width = calib_data["image_width"]
        cam_info.height = calib_data["image_height"]
        cam_info.K = calib_data["camera_matrix"]["data"]
        cam_info.D = calib_data["distortion_coefficients"]["data"]
        cam_info.R = calib_data["rectification_matrix"]["data"]
        cam_info.P = calib_data["projection_matrix"]["data"]
        cam_info.distortion_model = calib_data["distortion_model"]
        return cam_info



# All methods starting with _ live in ros2 land.
# Methods that do not start with _ get called by the duckietown interface. (see wrap_direct() in main())
class DuckietownBridge:
    last_camera_timestamp: float
    last_odometry_timestamp: float
    node: DuckietownBridgeNode
    thread: threading.Thread

    def __init__(self, args):
        self.args = args

        self.last_camera_timestamp = -1
        self.last_odometry_timestamp = -1

    def init(self, context: Context):
        context.info("init()")
        assert(not hasattr(self, "node") and not hasattr(self, "thread"))

        rclpy.init(args=self.args)
        
        self.node = DuckietownBridgeNode()
        self.thread = threading.Thread(tartget=rclpy.sping(), args=(self.node,), daemon=True)
        self.thread.start()
        context.info("inited")

    
    def on_received_seed(self, context: Context, data: int):
        np.random.seed(data)


    def on_received_episode_start(self, context: Context, data: EpisodeStart):
        context.info(f"Starting episode {data.episode_name}.")
        yaml_payload = getattr(data, "yaml_payload", "{}")
        self.node.publish_episode_start(data.episode_name, yaml_payload)


    def on_received_observations(self, data: DB20ObservationsWithTimestamp, context: Context):
        camera = data.camera
        odometry = data.odometry
        # context.info(f'received obs camera {camera.timestamp} odometry {odometry.timestamp}')

        if camera.timestamp != self.last_camera_timestamp or True:
            self.node.publish_img(camera.jpg_data, camera.timestamp)
            self.node.publish_info(camera.timestamp)
            self.last_camera_timestamp = camera.timestamp

        if odometry.timestamp != self.last_odometry_timestamp or True:
            self.node.publish_odometry(
                odometry.resolution_rad, odometry.axis_left_rad, odometry.axis_right_rad, odometry.timestamp
            )
            self.last_odometry_timestamp = odometry.timestamp


    def on_received_get_commands(self, context: Context, data: GetCommands):
        # context.info(f'on_received_get_commands')

        if not self.node.initialized:
            pwm_left, pwm_right = [0, 0]
        else:
            # TODO: let's use a queue here. Performance suffers otherwise.
            # What you should do is: *get the last command*, if available
            # otherwise, wait for one command.
            t0 = time.time()
            while not self.node.updated:
                dt = time.time() - t0
                if dt > 2.0:
                    context.info(f"node not ready since {dt:.1f} s")
                    time.sleep(0.5)
                if dt > 60:
                    msg = "I have been waiting for commands from the ROS part" f" since {int(dt)} s"
                    context.error(msg)
                    raise Exception(msg)
                time.sleep(0.02)
            dt = time.time() - t0
            if dt > 2.0:
                context.info(f"obtained node commands after {dt:.1f} s")
                time.sleep(0.2)

            pwm_left, pwm_right = self.node.action
            self.node.updated = False

        grey = RGB(0.5, 0.5, 0.5)
        led_commands = LEDSCommands(grey, grey, grey, grey, grey)
        pwm_commands = PWMCommands(motor_left=pwm_left, motor_right=pwm_right)
        commands = DB20Commands(pwm_commands, led_commands)

        context.write("commands", commands)

    def finish(self, context):
        context.info("finish()")
        
        rclpy.shutdown()
        self.thread.join()

        context.info("finished")



def main(args=None):
    
    interface = DuckietownBridge(args=args)

    protocol = protocol_agent_DB20_timestamps
    wrap_direct(node=interface, protocol=protocol, args=[])


