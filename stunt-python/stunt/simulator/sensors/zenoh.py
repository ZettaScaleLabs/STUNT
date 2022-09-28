import zenoh
from zenoh import Reliability, SubMode


class ZenohSensor:
    def __init__(self, session, ke, sensor_type, data_type, configuration={}):
        self.session = session
        self.ke = ke
        self.data_type = data_type
        self.sensor = sensor_type(configuration, self.on_data)

    def on_data(self, data):
        stunt_data = self.data_type.from_simulator(data)
        self.session.put(self.ke, stunt_data.serialize())


class ZenohControl:
    def __init__(self, session, ke, on_data):
        self.session = session
        self.ke = ke
        self.sub = self.session.subscribe(
            self.ke, on_data, reliability=Reliability.Reliable, mode=SubMode.Push
        )
