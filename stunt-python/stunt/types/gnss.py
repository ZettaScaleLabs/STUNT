from carla import GnssMeasurement as CarlaGnssMeasurement

from dataclasses import dataclass
from pycdr2 import IdlStruct
from pycdr2.types import float64


@dataclass
class GnssMeasurement(IdlStruct):
    altitude: float64
    latitude: float64
    longitude: float64
    timestamp: float64

    def __init__(
        self,
        altitude=0.0,
        latitude=0.0,
        longitude=0.0,
        timestamp=0.0,
    ):
        self.altitude = altitude
        self.latitude = latitude
        self.longitude = longitude
        self.timestamp = timestamp

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return f"GnssMeasurement(altitude={self.altitude}, "
        + "latitude={self.latitude}, longitude={self.longitude}, "
        + " ts={self.timestamp})"

    @classmethod
    def from_simulator(cls, gnss):
        """Creates a STUNT gnss from a simulator gnss.

        Args:
            gnss: An instance of a simulator gnss.

        Returns:
            :py:class:`.GnssMeasurement`: A STUNT gnss.
        """

        if not isinstance(gnss, CarlaGnssMeasurement):
            raise ValueError(
                "The gnss must be a Location or carla.GnssMeasurement"
            )
        return cls(
            gnss.altitude,
            gnss.latitude,
            gnss.longitude,
            gnss.timestamp,
        )

    def to_simulator(self):
        return CarlaGnssMeasurement(
            self.altitude,
            self.latitude,
            self.longitude,
        )

    def to_dict(self):
        return {
            "altitude": self.altitude,
            "latitude": self.latitude,
            "longitude": self.longitude,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, dictionary):
        return cls(
            dictionary["altitude"],
            dictionary["latitude"],
            dictionary["longitude"],
            dictionary["timestamp"],
        )
