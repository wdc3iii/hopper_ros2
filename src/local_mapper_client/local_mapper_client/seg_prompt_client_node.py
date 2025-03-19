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

        # Image subscriber (receives images from Orin)
        self.image_subscriber = self.create_subscription(
            Image,
            'd435_image',
            self.image_callback,
            1
        )

        # Feedback subscriber (receives segmentation mask updates)
        self.mask_subscriber = self.create_subscription(
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

        self.seg_vis_win_name = "Camera Control"
        cv2.namedWindow(self.seg_vis_win_name, cv2.WINDOW_GUI_NORMAL)
        cv2.resizeWindow(self.seg_vis_win_name, (960, 540))
        cv2.setMouseCallback(self.seg_vis_win_name, self.mouse_click_callback)

        self.send_goal()

    def image_callback(self, msg):
        """Displays received image from Orin."""
        self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    
    def vis_image(self):
        if self.latest_image is None:
                return
        if self.latest_mask is None:    
            frame = self.latest_image
        else:
            frame = cv2.addWeighted(
                self.latest_image,
                1,
                cv2.cvtColor((self.latest_mask * 255).astype(np.uint8), cv2.COLOR_GRAY2RGB),
                0.5,
                0
            )
        cv2.imshow(self.seg_vis_win_name, cv2.cvtColor(frame,cv2.COLOR_BGR2RGB))
        key_stroke = cv2.waitKey(1)
        if key_stroke == 13:
            self.exit_prompt = True
            self.mouse_click_callback(-1, 0, 0, None, None)
        elif key_stroke >= 0 and key_stroke < 256:
            self.get_logger().info(f"Setting group: {key_stroke}")
            self.curr_group = key_stroke

    def mask_callback(self, msg):
        """Displays segmentation mask received from Orin."""
        self.latest_mask = np.array(msg.data).reshape(msg.height, msg.width)
        self.vis_image()
        cv2.waitKey(1)

    def mouse_click_callback(self, event, x, y, flags, param):
        """Handles mouse clicks and sends them to Orin."""
        if event == cv2.EVENT_LBUTTONDOWN or event == cv2.EVENT_RBUTTONDOWN or self.exit_prompt:
            click_msg = PromptClickData()
            click_msg.group = self.curr_group
            click_msg.label = True if event == cv2.EVENT_LBUTTONDOWN else False
            click_msg.x = x
            click_msg.y = y
            click_msg.exit_prompt = self.exit_prompt
            self.click_publisher.publish(click_msg)
            self.get_logger().info(f"Sent prompt click at ({x}, {y}) to group {self.curr_group} with label {click_msg.label}")
            if self.exit_prompt:
                self.exit_prompt = False
        elif event == cv2.EVENT_MBUTTONDOWN:
            if self.in_progress:
                self.get_logger().info(f"Recieved segmenter prompt command - cannot send, current prompt not complete.")
                return
            self.send_goal()

    def trigger_callback(self, request, response):
        """Handles segmentation trigger requests."""
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
        """Handles the segmentation goal response."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Segmentation goal rejected.')
            return
        self.in_progress = True
        self.get_logger().info('Segmentation goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Receives segmentation masks as feedback."""
        self.mask_callback(feedback_msg.feedback.mask)

    def result_callback(self, future):
        """Handles final segmentation result."""
        result = future.result().result
        self.get_logger().info('Segmentation complete.')
        self.mask_callback(result.final_mask)
        self.in_progress = False
        cv2.waitKey(0)


def main():
    rclpy.init()
    client = SegPromptClient()
    rclpy.spin(client)


if __name__ == '__main__':
    main()
