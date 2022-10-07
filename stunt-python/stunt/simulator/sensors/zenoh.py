import json
from zenoh import Reliability


class ZenohSensor:
    def __init__(self, session, ke, sensor_type, data_type, configuration={}):
        self.session = session
        self.ke = ke
        self.data_type = data_type
        self.sensor = sensor_type(configuration, self.on_data)

    def on_data(self, data):
        if isinstance(data, list):
            self.session.put(self.ke, json.dumps(data).encode("utf-8"))
        elif isinstance(data, self.data_type):
            self.session.put(self.ke, data.serialize())
        else:
            stunt_data = self.data_type.from_simulator(data)
            self.session.put(self.ke, stunt_data.serialize())


class ZenohControl:
    def __init__(self, session, ke, on_data):
        self.session = session
        self.ke = ke
        self.sub = self.session.declare_subscriber(
            self.ke,
            on_data,
            reliability=Reliability.RELIABLE(),
        )
