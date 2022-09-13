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

use serde::{Deserialize, Serialize};
use std::{
    f64::consts::PI,
    ops::{Add, Sub},
};

#[derive(Deserialize, Serialize, Debug, PartialEq)]
pub struct Vector2D {
    pub x: f64,
    pub y: f64,
}

impl<'a, 'b> Add<&'b Vector2D> for &'a Vector2D {
    type Output = Vector2D;

    fn add(self, rhs: &'b Vector2D) -> Self::Output {
        Vector2D {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
        }
    }
}

impl<'a, 'b> Sub<&'b Vector2D> for &'a Vector2D {
    type Output = Vector2D;

    fn sub(self, rhs: &'b Vector2D) -> Self::Output {
        Vector2D {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
        }
    }
}

impl Vector2D {
    pub fn from_json(json: impl AsRef<str>) -> Self {
        let res = serde_json::from_str(json.as_ref());
        if let Err(e) = res {
            panic!(
                "Could not deserialize Vector2D:\n\t(json) {}\n\t(error) {:?}",
                json.as_ref(),
                e
            );
        }

        res.unwrap()
    }

    /// Computes the angle between the vector and another vector in radians.
    pub fn get_angle(&self, other: &Self) -> f64 {
        let mut angle = self.y.atan2(self.x) - other.y.atan2(other.x);
        if angle > PI {
            angle -= 2.0 * PI;
        } else if angle < -PI {
            angle += 2.0 * PI;
        }

        angle
    }

    /// Calculates the L1 distance between the point and another point.
    pub fn l1_distance(&self, other: &Self) -> f64 {
        (self.x - other.x).abs() + (self.y - other.y).abs()
    }

    /// Calculates the L2 distance between the point and another point.
    pub fn l2_distance(&self, other: &Self) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2)).sqrt()
    }

    ///  Returns the magnitude of the 2D vector.
    pub fn magnitude(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::Vector2D;

    #[test]
    fn test_from_json() {
        assert_eq!(
            Vector2D { x: 0.1, y: 0.2 },
            Vector2D::from_json(r#"{ "x": 0.1, "y": 0.2 }"#)
        )
    }
}
