import json
from zenoh import CongestionControl
from stunt.types import Image


class ZenohSensor:
    def __init__(self, session, ke, sensor_type, data_type, configuration={}):
        self.session = session
        self.ke = ke
        self.data_type = data_type
        self.sensor = sensor_type(configuration, self.on_data)
        self.pub = self.session.declare_publisher(
            self.ke, congestion_control=CongestionControl.BLOCK()
        )

    def on_data(self, data):
        if isinstance(data, list):
            self.pub.put(json.dumps(data).encode("utf-8"))
        elif isinstance(data, Image):
            pass
        elif isinstance(data, self.data_type):
            self.pub.put(data.serialize())
        else:
            stunt_data = self.data_type.from_simulator(data)
            self.pub.put(stunt_data.serialize())


class ZenohControl:
    def __init__(self, session, ke, on_data):
        self.session = session
        self.ke = ke
        self.sub = self.session.declare_subscriber(self.ke, on_data)
