# Simple-SLAM

### 摘要
之所以称之为Simple-SLAM，就是本文的SLAM算法足够简单，使用OpenCV的ORB做特征提取，以及matplotlib绘制环境建图，麻雀虽小，核心在讲解SLAM的SfM关键技术，将机器人运动的周边环境Point cloud给绘制出来 。

### 第一步：对视频帧提取ORB关键点

ORB关键点是一种非常高效的关键点，用OpenCV即可快速上手使用，如下所示：

```python
import cv2
cap = cv2.VideoCapture('...') # 读取视频
orb = cv2.ORB_create() # 初始化ORB检测器
keypoints, descriptor = orb.detectAndCompute(frame) # 对frame提取关键点
image = cv2.drawKeypoints(frame, keypoints, None, color=(0, 0, 255), flags=0)
cv2.imshow("ORB features", image)
cv2.waitKey(1)
```

![img](https://pic1.zhimg.com/80/v2-c75733bd4cfaca2d7267370996d1141c_720w.jpg)

## 第二步：对 ![[公式]](https://www.zhihu.com/equation?tex=t_1) 和 ![[公式]](https://www.zhihu.com/equation?tex=t_2) 时刻的ORB关键点计算匹配关系

ORB关键点的描述子是二进制串，这里我们可以选择使用NORM_HAMMING距离和Brute force方式进行求解

```python3
keypoints1, descriptor1 = orb.detectAndCompute(frame1) # t1时刻的ORB关键点
keypoints2, descriptor2 = orb.detectAndCompute(frame2) # t2时刻的ORB关键点

# ORB使用的是二进制描述子，使用NORM_HAMMING方式进行相似度比较，并通过sort进行排序
bruteForceMatcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(descriptor1, descriptor2)
matches = sorted(matches, key=lambda x: x.distance)

# 绘制Top10的关键点匹配关系
matchImage = cv2.drawMatches(frame1, keypoints1, frame2, keypoints2, matches[:10], frame1, flags=2)
cv2.imshow('Data association', matchImage)
cv2.waitKey()
```

![img](https://pic1.zhimg.com/80/v2-0f269b69424e17bf0a9f41c4d6187e68_720w.jpg)

## 第三步：通过不同时间采样的关键点进行定位和建图

首先，来科普一下关键知识点：

- focal length：焦距，即镜头到成像点的距离 ![[公式]](https://www.zhihu.com/equation?tex=%28f_x%2Cf_y%29)
- Intrinsic matrix：描述了相机的内参，如焦距、图像像素尺寸 ![[公式]](https://www.zhihu.com/equation?tex=%28x%2Cy%29)

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Bbmatrix%7D+f_x+%26+0+%26+x+%5C%5C+0+%26+f_y+%26+y+%5C%5C+0+%26+0+%26+1+%5Cend%7Bbmatrix%7D)

- Extrinsic matrix：描述了相机的外参，由旋转矩阵与平移向量组成

![[公式]](https://www.zhihu.com/equation?tex=%5Cbegin%7Bbmatrix%7D+R_1+%26+R_2+%26+R_3+%5C%5C+R_4+%26+R_5+%26+R_6+%5C%5C+R_7+%26+R_8+%26+R_9+%5Cend%7Bbmatrix%7D+%5Cbegin%7Bbmatrix%7D+t_1+%5C%5C+t_2+%5C%5C+t_3+%5Cend%7Bbmatrix%7D)

- Camera matrix：相机成像矩阵，即Intrinsic matrix和Extrinsic matrix的组合，将

![[公式]](https://www.zhihu.com/equation?tex=P+%3D+%5Cbegin%7Bbmatrix%7D+f_x+%26+0+%26+x+%5C%5C+0+%26+f_y+%26+y+%5C%5C+0+%26+0+%26+1+%5Cend%7Bbmatrix%7D+%2A+%5Cbegin%7Bbmatrix%7D+R_1+%26+R_2+%26+R_3+%5C%5C+R_4+%26+R_5+%26+R_6+%5C%5C+R_7+%26+R_8+%26+R_9+%5Cend%7Bbmatrix%7D+%5Cbegin%7Bbmatrix%7D+t_1+%5C%5C+t_2+%5C%5C+t_3+%5Cend%7Bbmatrix%7D)

- Back projection：将 ![[公式]](https://www.zhihu.com/equation?tex=i) 时刻图像画面的二维像素点 ![[公式]](https://www.zhihu.com/equation?tex=x_i) 反向投影到对应的三维世界坐标系的位置 ![[公式]](https://www.zhihu.com/equation?tex=X_i)

![[公式]](https://www.zhihu.com/equation?tex=X_i%3DP%5E%7B-1%7Dx_i)

- Structure from Motion (SfM)：通过机器人运动时对场景采集的多张不同视角的照片，实现Back projection
- Fundamental matrix：描述了两张图像的空间几何关系，假设摄像机未矫正（uncalibrated）
- Essential matrix：同Fundamental matrix，但假设摄像机已矫正（calibrated）

由于一张二维图像无法进行back projection，所以我们需要使用SfM，通过机器人/车辆在运动轨迹上对周边环境采集的多张图片进行Struture from motion，假设我们有 ![[公式]](https://www.zhihu.com/equation?tex=t_1) 和 ![[公式]](https://www.zhihu.com/equation?tex=t_2) 时刻采集的关键点 ![[公式]](https://www.zhihu.com/equation?tex=p_1) 与 ![[公式]](https://www.zhihu.com/equation?tex=p_2) （每个时刻关键点超过8个），那么就可以求解：

```python3
# 假设图像中心坐标为x,y，相机焦距已知，分别为fx,fy
K = np.array([[f_x, 0,   x],
              [0,   f_y, y],
              [0,   0,   1]])

# 通过t1和t2时刻检测的ORB关键点，求解fundamental matrix
F, mask = cv2.findFundamentalMat(np.float32(points2), np.float32(points1), cv2.FM_8POINT)

# 从fundamental matrix中提取出R旋转矩阵与t平移向量
points, R, t, mask = cv2.recoverPose(F, np.float32(points2), np.float32(points1), K, 500)
R = np.asmatrix(R).I
```

这里的 ![[公式]](https://www.zhihu.com/equation?tex=t) 即相机位置在世界坐标系的平移向量，假设 ![[公式]](https://www.zhihu.com/equation?tex=t_1) 时刻相机坐标为camera_previous_position

```python3
camera_current_position = camera_previous_position + t
```

将当前时刻的关键点back projection到三维世界坐标系上，记录当前时刻的landmarks（路标）

```python3
# 构建camera matrix
C = np.hstack((R,t))
P = np.asmatrix(K) * np.asmatrix(C)

landmarks = []
for i in range(len(points2)):
    pts2d = np.asmatrix([points2[i][0], points2[i][1], 1]).T
    P = np.asmatrix(K) * np.asmatrix(C)
    pts3d = np.asmatrix(P).I * pts2d
    landmarks.append([pts3d[0][0] * self.scale + self.camPos[0],
                      pts3d[1][0] * self.scale + self.camPos[1],
                      pts3d[2][0] * self.scale + self.camPos[2]])
```

将机器人/车辆运动过程的相机位置（定位）和路标（建图）记录下来，存放在camera_trajectory与landmark_trajectory中

```python3
camera_trajectory.append(camera_current_position)
landmark_trajectory.append(landmarks)
```

## 第四步：使用matplotlib绘制地图

因为是Simple-SLAM，所以一切从简，用最朴素的图表来绘制相机位置轨迹与环境路标

```python3
camera_trajectory_points = np.array(camera_trajectory)
landmark_trajectory_points = np.array(landmark_trajectory)

fig = plt.figure()
ax = Axes3D(fig)
ax.Axes3D(fig)
ax.scatter(landmark_trajectory_points[:, [0]], landmark_trajectory_points[:, [1]], landmark_trajectory_points[:, [2]])
ax.scatter(camera_trajectory_points[:, [0]], camera_trajectory_points[:, [1]], camera_trajectory_points[:, [2]], c='r')
plt.show()
```

![img](https://pic3.zhimg.com/80/v2-e95c8f03d944cb24e061be734f6b6db2_720w.jpg)