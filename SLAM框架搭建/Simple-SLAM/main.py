###################################################
#             SIMPLE SLAM PROGRAM
# ------------------------------------------------
# Author : Wang Kang
# Date : 2022/2/10
# Email : prince.love@live.cn
###################################################
import cv2
import math
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class SimpleSLAM:

    def __init__(self, width, height, fov):

        # Image size
        self.prev_frame = None
        self.width = width
        self.height = height

        # Create ORB extractor
        self.orb = cv2.ORB_create()
        self.keypoint1 = []     # keypoints in previous frame
        self.descriptor1 = []   # descriptors in previous frame

        # Create the brute force matcher
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Camera parameters
        self.fov = fov
        self.cam_xyz = np.matrix([0, 0, 0]).I   # camera position x,y,z
        self.cam_trajectory = []                # camera position trajectory
        self.landmark_trajectory = []           # landmark trajectory
        self.world_scale = 5

        # Camera intrinsic matrix
        x = width / 2
        y = height / 2
        fov = fov * (math.pi / 180)
        fx = x / math.tan(fov / 2)
        fy = y / math.tan(fov / 2)
        self.K = np.array([[fx, 0, x],
                           [0, fy, y],
                           [0, 0, 1]])


    def run(self, frame):

        # Extract ORB features
        keypoints2, descriptors2 = self.orb.detectAndCompute(frame, None)

        # Store data for association with next frame
        if not self.keypoint1:
            self.prev_frame = frame.copy()
            self.keypoint1 = keypoints2
            self.descriptor1 = descriptors2
            return

        # Matching features
        matches = self.matcher.match(self.descriptor1, descriptors2)
        matches = sorted(matches, key=lambda x: x.distance)
        print("Match points: ", len(matches))

        matchimage = cv2.drawMatches(self.prev_frame, self.keypoint1, frame, keypoints2, matches[:30], self.prev_frame, flags=2)
        cv2.imshow('Data association', matchimage)

        # Points association
        points1 = np.float32([self.keypoint1[m.queryIdx].pt for m in matches])
        points2 = np.float32([keypoints2[m.trainIdx].pt for m in matches])

        # Calculate the fundamental matrix (8-point algorithm)
        E, mask = cv2.findFundamentalMat(points2, points1, cv2.FM_8POINT)

        # Calculate the rotation matrix and translation vecto
        points, R, t, mask = cv2.recoverPose(E, np.float32(points2), np.float32(points1), self.K, 500)
        R = np.asmatrix(R).I

        # Calculate the camera matrix
        C = np.hstack((R, t))
        P = np.asmatrix(self.K) * np.asmatrix(C)

        # Back project points in current frame
        for i in range(len(points2)):
            pts2d = np.asmatrix([points2[i][0], points2[i][1], 1]).T
            pts3d = np.asmatrix(P).I * pts2d

            # Store landmarks
            self.landmark_trajectory.append([pts3d[0][0] * self.world_scale + self.cam_xyz[0],
                                             pts3d[1][0] * self.world_scale + self.cam_xyz[1],
                                             pts3d[2][0] * self.world_scale + self.cam_xyz[2]])

        # Store the new camera position
        self.cam_xyz = self.cam_xyz + t
        self.cam_trajectory.append(self.cam_xyz[:10])

        print("Camera trajectory length: {len(self.cam_trajectory)}")
        print("Landmarks points: {len(self.cam_trajectory)}")

        # Update data of previous frame
        self.prev_frame = frame.copy()
        self.keypoint1 = keypoints2
        self.descriptor1 = descriptors2

    def buildMap(self):
        cam_trajectory = np.array(self.cam_trajectory)
        landmark_trajectory = np.array(self.landmark_trajectory)
        # np.save("output/cam_trajectory.npy", self.cam_trajectory)
        # np.save("output/landmark_trajectory.npy", self.landmark_trajectory)
        self.plotMap(cam_trajectory, landmark_trajectory)

    def loadMap(self):
        cam_trajectory = np.load("output/cam_trajectory.npy")
        landmark_trajectory = np.load("output/landmark_trajectory.npy")
        self.plotMap(cam_trajectory, landmark_trajectory)

    def plotMap(self, cam_trajectory, landmark_trajectory):
        fig = plt.figure()
        ax = Axes3D(fig)

        # Plot camera trajectory
        x, y, z = (np.squeeze(cam_trajectory[:, [0]]), np.squeeze(cam_trajectory[:, [1]]), np.squeeze(cam_trajectory[:, [2]]))
        ax.scatter(x, y, z, c='r')
        ax.plot(x, y, z, 'bo--')

        # Plot landmarks
        x, y, z = (np.squeeze(landmark_trajectory[:, [0]]), np.squeeze(landmark_trajectory[:, [1]]), np.squeeze(landmark_trajectory[:, [2]]))
        ax.scatter(x, y, z, alpha=0.2)

        plt.show()

if __name__ == '__main__':

    cap = cv2.VideoCapture("video/drive.mp4")
    slam = SimpleSLAM(width=800, height=600, fov=60)
    frame_index = 0
    frame_interval = 10

    while (cap.isOpened()):

        # Read frame
        ret, frame = cap.read()
        if ret == False:
            break

        if (frame_index % frame_interval == 0):
            slam.run(frame)

        # Frame index++
        frame_index += 1

        # Display frame
        cv2.imshow("Video", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    slam.buildMap()
    cap.release()
    cv2.destroyAllWindows()
