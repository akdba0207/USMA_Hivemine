import cv2
import numpy as np
import time
import json, os, socket

import rospy
import cv2

centroids_json = {}
os.chdir("pictures")

boundaries = [
    ([0, 0, 245], [30, 100, 255])
]

class VideoPublisher:
    def __init__(self):
        self.camera_index = 0
        self.cap = cv2.VideoCapture(self.camera_index)
        rospy.init_node('video_publisher', anonymous=True)
        self.frames = []
        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    def publish_frames(self):
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if not ret:
                rospy.logerr("Failed to capture image")
                continue

            for (lower, upper) in boundaries:
                sig = 1
                blurredImage = cv2.GaussianBlur(frame, (sig, sig), 0)

                lower = np.array(lower, dtype="uint8")
                upper = np.array(upper, dtype="uint8")
                mask = cv2.inRange(blurredImage, lower, upper)
                binaryImage = cv2.bitwise_and(blurredImage, blurredImage, mask=mask)

                dilatedImage = self.erodeDilate(binaryImage)

                grayscaleImage = cv2.cvtColor(dilatedImage, cv2.COLOR_BGR2GRAY)

                connecttivity = 8
                num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(grayscaleImage, connecttivity, cv2.CV_32S)

                tmp = []
                count = 0
                for centroid in centroids[1:]:
                    area = stats[count, 4]
                    dict = {"cx": int(centroid[0]), "cy": int(centroid[1]), "area": int(area)}
                    tmp.append(dict)
                    cv2.circle(grayscaleImage, (int(centroid[0]), int(centroid[1])), 5, (255, 255, 255), -1)
                    count += 1
                current_time = rospy.Time.now().to_nsec()
                centroids_json[current_time] = tmp

                grayscaleImageBGR = cv2.cvtColor(grayscaleImage, cv2.COLOR_GRAY2BGR)
                combined_frame = np.hstack((frame, grayscaleImageBGR))
                self.frames.append(combined_frame)

                cv2.imshow("images", combined_frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    def erodeDilate(self, image):
        height = 5
        width = 5
        structElem = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (height, width))
        dilatedImage = cv2.dilate(image, structElem, iterations=2)
        return dilatedImage

    def save_video(self, filename, fps=20):
        frame_height, frame_width, _ = self.frames[0].shape
        fourcc = cv2.VideoWriter_fourcc(*'XVID')
        out = cv2.VideoWriter(filename, fourcc, fps, (frame_width, frame_height))
        for frame in self.frames:
            out.write(frame)
        out.release()

    def __del__(self):
        self.cap.release()

def save_centroids_to_json():
    centroid_string = json.dumps(centroids_json, indent=4)
    with open("centroids.json", 'w') as f:
        f.write(centroid_string)
    print(centroid_string)

if __name__ == "__main__":
    try:
        video_publisher = VideoPublisher()
        video_publisher.publish_frames()
    except rospy.ROSInterruptException:
        pass
    finally:
        video_publisher.save_video('output.avi')
        save_centroids_to_json()
