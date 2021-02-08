# First import the library
import pyrealsense2 as rs
import math as m
import threading
import zmq
context = zmq.Context()

print("connecting to main thread")
socket = context.socket(zmq.REQ)
socket.connect("tcp://localhost:5557")
latitude = 0
longitude = 0
thetaCoord = 0

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

# Build config object and request pose data
cfg = rs.config()
cfg.enable_stream(rs.stream.pose)

# Start streaming with requested config
pipe.start(cfg)

def infoSend():
    while True:
        socket.send(str(latitude).encode() + b" " + str(longitude).encode() + b" " + str(thetaCoord).encode())
        message = socket.recv()    

def indoorGPS():
    global latitude, longitude, thetaCoord
    latitude = 0
    longitude = 0
    thetaCoord = 0
    try:
        while (True):
            # Wait for the next set of frames from the camera
            frames = pipe.wait_for_frames()

        # Fetch pose frame
            pose = frames.get_pose_frame()
            if pose:
            # Print some of the pose data to the terminal
                data = pose.get_pose_data()

            # Euler angles from pose quaternion
            # See also https://github.com/IntelRealSense/librealsense/issues/5178#issuecomment-549795232
            # and https://github.com/IntelRealSense/librealsense/issues/5178#issuecomment-550217609

                w = data.rotation.w
                x = -data.rotation.z
                y = data.rotation.x
                z = -data.rotation.y

                pitch =  -m.asin(2.0 * (x*z - w*y)) * 180.0 / m.pi;
                roll  =  m.atan2(2.0 * (w*x + y*z), w*w - x*x - y*y + z*z) * 180.0 / m.pi;
                yaw   =  m.atan2(2.0 * (w*z + x*y), w*w + x*x - y*y - z*z) * 180.0 / m.pi;
                latitude = data.translation.x
                longitude = data.translation.y
                thetaCoord = yaw
                print("Position: {}".format(data.translation))
#            print("Frame #{}".format(pose.frame_number))
                print("RPY [deg]: Roll: {0:.7f}, Pitch: {1:.7f}, Yaw: {2:.7f}".format(roll, pitch, yaw))


    finally:
        pipe.stop()

infoSender = threading.Thread(target=infoSend)
infoSender.daemon = True
infoSender.start()
indoorGPS()

