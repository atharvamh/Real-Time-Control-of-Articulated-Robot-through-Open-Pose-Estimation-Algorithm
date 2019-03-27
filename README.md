# RAM
Robotic Arm Mimicking Human Arm Movements - 
1. The objective of this project was to control a robotic arm solely by the user's arm movements - the shoulder and elbow movements without
 the use of any physical sensors (such as accelerometer, gyro , etc.) by the user.
2. This was achieved by the OpenPose body pose detection algorithm that detects key points on the user
3. The user stands in front of a camera, and real time video processing involving body pose detection is performed
4. The region of interest,the hand region is then extracted from the complete body pose
5. Mathematical operations are performed to extract the shoulder and the elbow angles with respect to a reference axis
6. These calculated angles are then sent to Arduino through serial communication which in turn controls the servo motors
7. For the claw movement, an aruco marker detection algorithm was used wherein when the marker is detected the claw opens, or if not
 detected the claw is closed.
