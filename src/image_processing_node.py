#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import pytesseract
from imutils.object_detection import non_max_suppression
import numpy as np
import time


# Global variables
bridge = CvBridge()
counter = 0
pub = rospy.Publisher("address", String, queue_size=5)

# Load the pre-trained EAST text detector
# MUST BE ABSOLUTE PATH!
net = cv2.dnn.readNet("/home/osboxes/catkin_ws/src/robotika_proekt/models/frozen_east_text_detection.pb")
# ABSOLUTE PATH to the pre-trained EAST text detector -> path_to_project/models/frozen_east_text_detection.pb

# Define the two output layer names for the EAST detector model that we
# are interested in -- the first represents the output probabilities and the
# second can be used to derive the bounding box coordinates of text
layerNames = [
    "feature_fusion/Conv_7/Sigmoid",
    "feature_fusion/concat_3"]


def find_text(image, min_confidence=0.5, width=320, height=320):
    """
    Finds and returns the bounding boxes in an image that represent regions with text.
    """
    global layerNames
    global net

    # Load the input image and grab the image dimensions
    orig = image.copy()
    (H, W) = image.shape[:2]

    # Set the new width and height and then determine the ratio in change
    # for both the width and height
    (newW, newH) = (width, height)
    rW = W / float(newW)
    rH = H / float(newH)

    # Resize the image and grab the new image dimensions
    image = cv2.resize(image, (newW, newH))
    (H, W) = image.shape[:2]

    # Construct a blob from the image and then perform a forward pass of
    # the model to obtain the two output layer sets
    blob = cv2.dnn.blobFromImage(image, 1.0, (W, H), (123.68, 116.78, 103.94), swapRB=True, crop=False)
    start = time.time()
    net.setInput(blob)
    (scores, geometry) = net.forward(layerNames)
    end = time.time()

    # Show timing information on text prediction
    rospy.loginfo("Text detection took {:.6f} seconds".format(end - start))

    # Grab the number of rows and columns from the scores volume, then
    # initialize our set of bounding box rectangles and corresponding
    # confidence scores
    (numRows, numCols) = scores.shape[2:4]
    rects = []
    confidences = []

    # Loop over the number of rows
    for y in range(0, numRows):
        # Extract the scores (probabilities), followed by the geometrical
        # data used to derive potential bounding box coordinates that
        # surround text
        scoresData = scores[0, 0, y]
        xData0 = geometry[0, 0, y]
        xData1 = geometry[0, 1, y]
        xData2 = geometry[0, 2, y]
        xData3 = geometry[0, 3, y]
        anglesData = geometry[0, 4, y]

        # Loop over the number of columns
        for x in range(0, numCols):
            # If our score does not have sufficient probability, ignore it
            if scoresData[x] < min_confidence:
                continue

            # Compute the offset factor as our resulting feature maps will
            # be 4x smaller than the input image
            (offsetX, offsetY) = (x * 4.0, y * 4.0)

            # Extract the rotation angle for the prediction and then
            # compute the sin and cosine
            angle = anglesData[x]
            cos = np.cos(angle)
            sin = np.sin(angle)

            # Use the geometry volume to derive the width and height of
            # the bounding box
            h = xData0[x] + xData2[x]
            w = xData1[x] + xData3[x]

            # Compute both the starting and ending (x, y)-coordinates for
            # the text prediction bounding box
            endX = int(offsetX + (cos * xData1[x]) + (sin * xData2[x]))
            endY = int(offsetY - (sin * xData1[x]) + (cos * xData2[x]))
            startX = int(endX - w)
            startY = int(endY - h)

            # Add the bounding box coordinates and probability score to
            # our respective lists
            rects.append((startX, startY, endX, endY))
            confidences.append(scoresData[x])

    # Apply non-maxima suppression to suppress weak, overlapping bounding boxes
    boxes = non_max_suppression(np.array(rects), probs=confidences)

    boxes_original_size = []
    # Loop over the bounding boxes
    for (startX, startY, endX, endY) in boxes:
        # Scale the bounding box coordinates based on the respective ratios
        startX = int(startX * rW)
        startY = int(startY * rH)
        endX = int(endX * rW)
        endY = int(endY * rH)
        boxes_original_size.append((startX, startY, endX, endY))

        # Draw the bounding box on the image
        cv2.rectangle(orig, (startX, startY), (endX, endY), (0, 255, 0), 2)

    # Show the output image (for testing)
    cv2.imshow("Text Detection", orig)
    cv2.waitKey(20)

    return boxes_original_size


def clean_text(text):
    """
    Removes all non-alphanumerical characters from a string.
    """
    return ("".join(c for c in text if c.isalnum())).lower()


def callback(img_msg):
    global bridge
    global counter
    global pub

    # Take every tenth frame
    if counter % 10 == 0:
        image = None
        try:
            image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Find all the regions in the image that contain text
        boxes = find_text(image)
        for startX, startY, endX, endY in boxes:
            try:
                # Take a region from the image (with a little extra pixels around it)
                region = image[startY - 10:endY + 10, startX - 10:endX + 10]

                # Convert it to grayscale format
                region = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)

                # Subtract the Gaussian blurred region from the original region - get a sharpened image
                region = cv2.addWeighted(region, 3, cv2.GaussianBlur(region, (3, 3), 0), -2, 0)

                # Parse the text from the region into a string
                text = pytesseract.image_to_string(cv2.cvtColor(region, cv2.COLOR_BGR2RGB))

                # Log the parsed text
                rospy.loginfo("Text: " + text)

                # Remove any unwanted characters from the string
                addr_msg = clean_text(text)

                # If the result is an empty string, go to the next region
                if addr_msg == "":
                    continue

                # Publish the (possible) address
                pub.publish(String(addr_msg))
            except Exception as e:
                pass

    # Increment the counter
    counter += 1


def image_processing_node():
    rospy.init_node('image_processing_node')

    rospy.Subscriber("image", Image, callback)

    rospy.spin()


if __name__ == "__main__":
    image_processing_node()
