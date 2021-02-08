import trollius
from trollius import From

import pygazebo
import pygazebo.msg.vector3d_pb2

@trollius.coroutine
def publish_loop():
    manager = yield From(pygazebo.connect(('localhost', 11345)))
    publisher = yield From(manager.advertise('~/my_velodyne/vel_cmd','gazebo.msgs.vel_cmd'))
    message = pygazebo.msg.vector3d_pb2.Vector3d()
    message.z = -100
    message.x = 200
    message.y = 300
    while True:
        yield From(publisher.publish(message))
        yield From(trollius.sleep(1.0))
loop = trollius.get_event_loop()
loop.run_until_complete(publish_loop())
