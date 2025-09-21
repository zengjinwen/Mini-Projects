import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class ColorDetector(Node):
    def __init__(self):
        super().__init__('color_detector')
        self.bridge = CvBridge() 
        self.sub = self.create_subscription(CompressedImage, '/left/compressed', self.image_compressed, 10)
        #self.pub = self.create_publisher(CompressedImage, '/detected/compressed', 10)
        self.pub = self.create_publisher(Image, '/detected', 10)
        # self.outdir = os.getcwd() + "/src/assignment3/resource/imgs/before"
        # self.count = 0
        # self.stride = 1
        
    def image_compressed(self, msg:CompressedImage):
        frame = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')
        bgr = self.image_processing(frame)
        out = self.bridge.cv2_to_imgmsg(bgr, 'bgr8')
        #out = self.bridge.cv2_to_compressed_imgmsg(bgr, 'jpg')

        out.header = msg.header
        self.pub.publish(out)

    # def image_compressed(self, msg:CompressedImage):
    #     #print(f"{self.count}/100")
    #     self.count += 1

    #     # if self.count % self.stride: 
    #     #     return
        
    #     frame = self.bridge.compressed_imgmsg_to_cv2(msg,'bgr8')

    #     sec, nsec = msg.header.stamp.sec, msg.header.stamp.nanosec
    #     fname = f'{self.count}.png'
    #     cv2.imwrite(f"{self.outdir}/{fname}", frame)
    #     print(f"saved frame {self.count}")
   


    def image_processing(self, frame):
        img_blur = cv2.GaussianBlur(frame, (5,5), 0) # Blurring
        img_hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(img_hsv)
        equ_v = cv2.equalizeHist(v)
        img_hsv_eq = cv2.merge([h, s, equ_v])
        smin, vmin = 50, 25
        smin_r, vmin_r = 255, 255
        smin_b, vmin_b = 80, 80
        red1  = cv2.inRange(img_hsv_eq, (  0, smin, vmin), ( 12, 255, 255))
        red3 =  cv2.inRange(img_hsv_eq, (  12, smin_r, vmin_r), ( 20, 255, 255))
        red2  = cv2.inRange(img_hsv_eq, (170, smin, vmin), (179, 255, 255))
        orange = cv2.inRange(img_hsv_eq, (23, smin, vmin), (24, 255, 255))
        blue   = cv2.inRange(img_hsv, (100, smin_b, vmin_b), (110, 255, 255))
        mask = red1 | red2 | orange | blue
        result = cv2.bitwise_and(img_hsv_eq, img_hsv_eq, mask=mask)
        kernel = np.ones((5,5), np.uint8)
        img_morph = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel, iterations=1)
        bgr = cv2.cvtColor(img_morph, cv2.COLOR_HSV2BGR)
        img_gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(img_gray,0 , 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST,cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(bgr, contours, -1, (0,255,0), 1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        for c in contours:
            area = cv2.contourArea(c)
            rect = cv2.boundingRect(c) # get the bounding box of this contour
            x, y, w, h = rect # the bounding box is described with 4 numbers
            cnt_mask = np.zeros(bgr.shape[:2], dtype=np.uint8)
            cv2.drawContours(cnt_mask, [c], -1, 255, thickness=cv2.FILLED)
            mean_b, mean_g, mean_r, _ = cv2.mean(bgr, mask=cnt_mask)
            if area < 400 or w < 15 or h/w <0.5 or h*w < 1500 or h < 15:
                continue
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 3) # draw the rectangle on your image
            label_color = "blue" if mean_b > 100 else ("orange" if mean_r > 145 else "red")
            label_obj = "flare" if h > w else "drum"
            cv2.putText(frame, f"{label_color} {label_obj}", (x + 10, y - 10), font, 0.7, (0, 255, 0),2, cv2.LINE_AA)  
        return frame




                                  
        

    
def main(args=None):
    rclpy.init(args=args)
    node = ColorDetector()
    rclpy.spin(node)
    # node.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()