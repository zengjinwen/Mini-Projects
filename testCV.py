import numpy as np # make sure to have numpy installed in your environment
import cv2
def display(img):
    cv2.imshow("image", img)
# show image in a window named "image"
    cv2.waitKey(10000)
# wait 10s or until the user presses a key
    cv2.destroyAllWindows()
# Clean up, remember to do this!
if __name__=='__main__':
    print(cv2.__version__)
# print out the opencv version we're using
    img = cv2.imread('sample.png')
# load the image
    display(img)