import trollius
from trollius import From

import pygazebo
#import pygazebo.msg.joint_cmd_pb2
import pygazebo.msg.vector3d_pb2

@trollius.coroutine
def publish_loop():
    print("gets to 1")
    manager = yield From(pygazebo.connect(('localhost', 11345)))
#    publisher = yield From(
#        manager.advertise('/gazebo/default/model/joint_cmd',
#                          'gazebo.msgs.JointCmd'))
    publisher = yield From(manager.advertise('~/my_velodyne/vel_cmd','gazebo.msgs.vel_cmd'))
    print("gets to 2")
#    message = pygazebo.msg.joint_cmd_pb2.JointCmd()
    message = pygazebo.msg.vector3d_pb2.Vector3d()
#    message.name = 'robot::left_wheel_hinge'
#    message.axis = 0
#    message.force = 1000.0
    message.z = -100
    message.x = 200
    message.y = 300
    print("gets to 3")
    while True:
        yield From(publisher.publish(message))
        yield From(trollius.sleep(1.0))
    print("gets to 4")
loop = trollius.get_event_loop()
loop.run_until_complete(publish_loop())
