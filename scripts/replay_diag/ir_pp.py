import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
class PP(Node):
    def __init__(self):
        super().__init__('ir_pp')
        self.br = CvBridge(); self.n = 0
        self.pubs = {}
        for s in ('left', 'right'):
            self.pubs[s] = self.create_publisher(Image, f'/pp/{s}_ir/image_raw', 10)
            self.create_subscription(Image, f'/camera/{s}_ir/image_raw',
                                     lambda m, s=s: self.cb(m, s), 10)
    def cb(self, msg, s):
        img = self.br.imgmsg_to_cv2(msg, 'mono8')
        img = cv2.medianBlur(img, 3)
        img = cv2.equalizeHist(img)
        out = self.br.cv2_to_imgmsg(img, 'mono8'); out.header = msg.header
        self.pubs[s].publish(out)
        self.n += 1
        if self.n % 200 == 0:
            self.get_logger().info(f'republished {self.n}')
rclpy.init(); rclpy.spin(PP())
