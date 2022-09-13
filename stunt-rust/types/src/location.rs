//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made available under the
// terms of the Eclipse Public License 2.0 which is available at
// http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
// which is available at https://www.apache.org/licenses/LICENSE-2.0.
//
// SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
//
// Contributors:
//   ZettaScale Zenoh Team, <zenoh@zettascale.tech>
//

use crate::vector3d::Vector3D;
use serde::{Deserialize, Serialize};
use std::{
    f64::consts::PI,
    ops::{Add, Deref, Sub},
};

#[derive(Deserialize, Serialize, PartialEq, Debug)]
pub struct Location {
    pub vec: Vector3D,
}

pub const EARTH_RADIUS_EQUATOR: f64 = 6378137.0;
pub const REFERENCE_LATITUDE: f64 = 0.0;
pub const REFERENCE_LONGITUDE: f64 = 0.0;

impl Deref for Location {
    type Target = Vector3D;

    fn deref(&self) -> &Self::Target {
        &self.vec
    }
}

impl<'a, 'b> Add<&'b Location> for &'a Location {
    type Output = Location;

    fn add(self, rhs: &'b Location) -> Self::Output {
        Location {
            vec: &self.vec + &rhs.vec,
        }
    }
}

impl<'a, 'b> Sub<&'b Location> for &'a Location {
    type Output = Location;

    fn sub(self, rhs: &'b Location) -> Self::Output {
        Location {
            vec: &self.vec - &rhs.vec,
        }
    }
}

impl Location {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            vec: Vector3D { x, y, z },
        }
    }

    pub fn from_json(json: impl AsRef<str>) -> Self {
        let res = serde_json::from_str(json.as_ref());
        if let Err(e) = res {
            panic!(
                "Could not deserialize Location:\n\t(json) {}\n\t(error) {:?}",
                json.as_ref(),
                e
            );
        }

        res.unwrap()
    }

    pub fn from_vector(vec: Vector3D) -> Self {
        Self { vec }
    }

    #[allow(non_snake_case)]
    pub fn as_vector_2D(&self) -> [f64; 2] {
        [self.x, self.y]
    }

    /// Creates the Location from GPS (latitude, longitude, altitude).
    ///
    /// This is the inverse of the `_location_to_gps` method found in
    /// https://github.com/carla-simulator/scenario_runner/blob/master/srunner/tools/route_manipulation.py
    pub fn from_gps(latitude: f64, longitude: f64, altitude: f64) -> Self {
        let scale = (REFERENCE_LATITUDE * PI / 180.0).cos();

        let base_x = scale * PI * EARTH_RADIUS_EQUATOR / 180.0 * REFERENCE_LONGITUDE;

        let base_y =
            scale * EARTH_RADIUS_EQUATOR * ((90.0 + REFERENCE_LATITUDE) * PI / 360.0).tan().log2();

        let x = scale * PI * EARTH_RADIUS_EQUATOR / 180.0 * longitude - base_x;
        let mut y =
            scale * EARTH_RADIUS_EQUATOR * ((90.0 + latitude) * PI / 360.0).tan().log2() - base_y;
        // FIXME Understand why this is necessary?
        y *= -1.0;

        Self {
            vec: Vector3D { x, y, z: altitude },
        }
    }

    /// Calculates the Euclidean distance between the given point and the other point.
    pub fn distance(&self, other: &Self) -> f64 {
        (self - other).magnitude()
    }
}

#[cfg(test)]
mod tests {
    use crate::vector3d::Vector3D;

    use super::Location;

    #[test]
    fn test_from_json() {
        assert_eq!(
            Location {
                vec: Vector3D {
                    x: 0.1,
                    y: 0.2,
                    z: 0.3
                }
            },
            Location::from_json(r#"{ "vec" : { "x": 0.1, "z": 0.3, "y": 0.2 } }"#)
        )
    }
}
