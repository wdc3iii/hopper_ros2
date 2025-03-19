import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from rclpy.action import ActionClient

from sensor_msgs.msg import Image
from local_mapper_interfaces.action import SegPrompt
from local_mapper_interfaces.msg import PromptClickData


class SegPromptClient(Node):
    def __init__(self):
        super().__init__('segment_prompt_client')

        # Action client to communicate with Jetson Orin
        self.in_progress = False
        self.seg_prompt_client = ActionClient(self, SegPrompt, 'seg_prompt')

        # Click publisher (sends clicks back to Orin)
        self.click_publisher = self.create_publisher(PromptClickData, 'seg_prompt_clicks', 10)

        # Create subscriptions
        self.image_subscriber = self.create_subscription(           # Image subscriber (receives images from Orin)
            Image,
            'd435_image',
            self.image_callback,
            1
        )
        self.mask_subscriber = self.create_subscription(            # Feedback subscriber (receives segmentation mask updates)
            Image,
            'segmentation_mask',
            self.mask_callback,
            10
        )

        # Timer to visualize image and process keystrokes
        self.timer = self.create_timer(0.1, self.vis_image)

        self.exit_prompt = False
        self.curr_group = 0
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_mask = None
        self.current_goal_handle = None

        # CV2 Window to handle clicks and image display
        self.seg_vis_win_name = "Camera Control"
        cv2.namedWindow(self.seg_vis_win_name, cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow(self.seg_vis_win_name, (960, 540))
        cv2.setMouseCallback(self.seg_vis_win_name, self.mouse_click_callback)

        # Launch the action
        self.send_goal()

    def image_callback(self, msg):
        """Displays received image from Orin."""
        self.get_logger().info("Image callback")
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self.vis_image()
    
    def vis_image(self):
        """Visualize the latest image"""
        # If no image, do nothing
        if self.latest_image is None:
            return

        if self.latest_mask is None:    
            frame = self.latest_image       # If no mask, show the image
        else:
            frame = cv2.addWeighted(        # Overlay the mask on the image
                self.latest_image,
                1,
                cv2.cvtColor((self.latest_mask * 255).astype(np.uint8), cv2.COLOR_GRAY2RGB),
                0.5,
                0
            )
        # Show the image in the cv2 window
        cv2.imshow(self.seg_vis_win_name, cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))

        # Process keystrokes
        key_stroke = cv2.waitKey(1)
        key = self.process_key_stroke(key_stroke)
        if key is not None:
            # Trigger an exit command if key < 0
            if key < 0:
                self.exit_prompt = True
                self.mouse_click_callback(-1, 0, 0, None, None)
            # Change the object group if key is a single digit integer
            elif key >= 0 and key < 10:
                self.get_logger().info(f"Setting group: {key}")
                self.curr_group = key

    @staticmethod
    def process_key_stroke(key_stroke: int):
        """Processes the key stroke read by cv2

        Args:
            key_stroke (int): keystroke from cv2

        Returns:
            int: semantic keystroke: -1 for enter, 0-9 for numerals, else None
        """
        if key_stroke != -1:
            if key_stroke == ord('0'):
                return 0
            elif key_stroke == ord('1'):
                return 1
            elif key_stroke == ord('2'):
                return 2
            elif key_stroke == ord('3'):
                return 3
            elif key_stroke == ord('4'):
                return 4
            elif key_stroke == ord('5'):
                return 5
            elif key_stroke == ord('6'):
                return 6
            elif key_stroke == ord('7'):
                return 7
            elif key_stroke == ord('8'):
                return 8
            elif key_stroke == ord('9'):
                return 9
            elif key_stroke == 13:  # Enter key
                return -1
        return None
    
    def mask_callback(self, msg):
        """Displays segmentation mask received from Orin."""
        self.latest_mask = np.array(msg.data).reshape(msg.height, msg.width)
        self.vis_image()

    def mouse_click_callback(self, event, x, y, flags, param):
        """Handles mouse clicks and sends them to Orin."""
        # Add a point to prompt, or exit the prompting sequence
        if event == cv2.EVENT_LBUTTONDOWN or event == cv2.EVENT_RBUTTONDOWN or self.exit_prompt:
            # Send click message to action server
            click_msg = PromptClickData()
            click_msg.group = self.curr_group
            click_msg.label = True if event == cv2.EVENT_LBUTTONDOWN else False
            click_msg.x = x
            click_msg.y = y
            click_msg.exit_prompt = self.exit_prompt
            self.click_publisher.publish(click_msg)
            
            # Handle exiting properly
            if self.exit_prompt:
                self.exit_prompt = False
                self.get_logger().info(f"Sent exit prompt command.")
            else:
                self.get_logger().info(f"Sent prompt click at ({x}, {y}) to group {self.curr_group} with label {click_msg.label}")
        # Trigger resegmentation
        elif event == cv2.EVENT_MBUTTONDOWN:
            if self.in_progress:
                self.get_logger().info(f"Recieved segmenter prompt command - cannot send, current prompt not complete.")
                return
            self.send_goal()

    def trigger_callback(self, request, response):
        """Handles segmentation trigger requests."""
        # Send a new goal to action server
        self.send_goal()
        response.success = True
        response.message = "Segmentation started."
        return response
    
    def send_goal(self):
        """Requests segmentation from the Jetson Orin."""
        self.get_logger().info("Sending Goal Request to update segmenter prompt...")

        goal_msg = SegPrompt.Goal()
        self.seg_prompt_client.wait_for_server()
        self._send_goal_future = self.seg_prompt_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        # Check if goal has been rejected
        if not goal_handle.accepted:
            self.get_logger().info('Segmentation goal rejected.')
            return
        self.in_progress = True
        self.get_logger().info('Segmentation goal accepted.')

        self.current_goal_handle = goal_handle  # Store handle

        # Add done callback
        self._get_result_future = self.current_goal_handle.get_result_async()
        if self._get_result_future is None:
            self.get_logger().error("Error requesting result from server: get_result_async() returned None.")
        else:
            self.get_logger().info("Successfully requested result from server.")
            self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receives segmentation masks as feedback."""
        self.mask_callback(feedback_msg.feedback.mask)

    def result_callback(self, future):
        """Handles final segmentation result."""
        self.get_logger().info('Segmentation complete.')
        result = future.result().result
        
        self.in_progress = False
        self.mask_callback(result.final_mask)


def main():
    rclpy.init()
    client = SegPromptClient()
    rclpy.spin(client)


if __name__ == '__main__':
    main()
