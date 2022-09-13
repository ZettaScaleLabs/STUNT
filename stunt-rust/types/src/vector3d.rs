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

use nalgebra::{Matrix3, Matrix4, Vector3, Vector4};
use serde::{Deserialize, Serialize};
use std::ops::{Add, Sub};

#[derive(Serialize, Deserialize, PartialEq, Debug)]
pub struct Vector3D {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Vector3D {
    pub fn as_array(&self) -> [f64; 3] {
        [self.x, self.y, self.z]
    }

    pub fn from_json(json: impl AsRef<str>) -> Self {
        let res = serde_json::from_str(json.as_ref());
        if let Err(e) = res {
            panic!(
                "Could not deserialize Vector3D:\n\t(json) {}\n\t(error) {:?}",
                json.as_ref(),
                e
            );
        }

        res.unwrap()
    }

    pub fn l1_distance(&self, other: &Self) -> f64 {
        (self.x - other.x).abs() + (self.y - other.y).abs() + (self.z - other.z).abs()
    }

    pub fn l2_distance(&self, other: &Self) -> f64 {
        ((self.x - other.x).powi(2) + (self.y - other.y).powi(2) + (self.z - other.z).powi(2))
            .sqrt()
    }

    pub fn magnitude(&self) -> f64 {
        (self.x.powi(2) + self.y.powi(2) + self.z.powi(2)).sqrt()
    }

    /// Converts the given 3D vector to the view of the camera using the extrinsic and the intrinsic
    /// matrix.
    ///
    /// TODO We assumed that the dimensions of:
    /// - the extrinsic matrix is 4×4
    /// - the intrinsic matrix is 3×3
    ///
    /// Are these information correct?
    pub fn to_camera_view(
        &self,
        extrinsic_matrix: &Matrix4<f64>,
        intrinsic_matrix: &Matrix3<f64>,
    ) -> Vector3D {
        // Equivalent Python code is:
        // position_vector = np.array([[self.x], [self.y], [self.z], [1.0]])
        let position_vector = Vector4::new(self.x, self.y, self.z, 1.0);

        // # Transform the points to the camera in 3D.
        // transformed_3D_pos = np.dot(np.linalg.inv(extrinsic_matrix), position_vector)
        let inverted_extrinsic_matrix = extrinsic_matrix
            .try_inverse()
            .unwrap_or_else(|| panic!("Could not inverse Matrix: {}", extrinsic_matrix));

        let transformed_3d_pos: Vector4<f64> = inverted_extrinsic_matrix * position_vector;

        // # Transform the points to 2D.
        // position_2D = np.dot(intrinsic_matrix, transformed_3D_pos[:3])
        let transformed_2d_pos: Vector3<f64> = Vector3::new(
            transformed_3d_pos[0],
            transformed_3d_pos[1],
            transformed_3d_pos[2],
        );
        let position_2d = intrinsic_matrix * transformed_2d_pos;

        // # Normalize the 2D points.
        // location_2D = type(self)(
        //     float(position_2D[0] / position_2D[2]),
        //     float(position_2D[1] / position_2D[2]),
        //     float(position_2D[2]),
        // )
        // return location_2D
        Vector3D {
            x: position_2d[0] / position_2d[2],
            y: position_2d[1] / position_2d[2],
            z: position_2d[2],
        }
    }

    /// Rotate the vector by a given `angle`.
    ///
    /// The angle is expected in degree. This method performs the conversion from degrees to
    /// radians.
    pub fn rotate(&self, angle: f64) -> Self {
        let angle_radians = angle.to_radians();
        let x = angle_radians.cos() * self.x - angle_radians.sin() * self.y;
        let y = angle_radians.sin() * self.x - angle_radians.cos() * self.x;

        Self { x, y, z: self.z }
    }
}

impl<'a, 'b> Add<&'b Vector3D> for &'a Vector3D {
    type Output = Vector3D;

    fn add(self, rhs: &'b Vector3D) -> Self::Output {
        Self::Output {
            x: self.x + rhs.x,
            y: self.y + rhs.y,
            z: self.z + rhs.z,
        }
    }
}

impl<'a, 'b> Sub<&'b Vector3D> for &'a Vector3D {
    type Output = Vector3D;

    fn sub(self, rhs: &'b Vector3D) -> Self::Output {
        Self::Output {
            x: self.x - rhs.x,
            y: self.y - rhs.y,
            z: self.z - rhs.z,
        }
    }
}

#[cfg(test)]
mod test {
    use super::Vector3D;

    #[test]
    fn test_l2_distance() {
        let sample_1 = Vector3D {
            x: 6.0,
            y: 51.0,
            z: 3.0,
        };
        let sample_2 = Vector3D {
            x: 1.9,
            y: 99.0,
            z: 2.9,
        };

        // Very trivial test to see if the formula is right…
        assert_eq!(sample_1.l2_distance(&sample_2), 2320.82f64.sqrt());
    }

    #[test]
    fn test_from_json() {
        assert_eq!(
            Vector3D {
                x: 1.0,
                y: 0.2,
                z: 0.3
            },
            Vector3D::from_json(r#"{ "x": 1.0, "y": 0.2, "z": 0.3 }"#)
        )
    }
}
