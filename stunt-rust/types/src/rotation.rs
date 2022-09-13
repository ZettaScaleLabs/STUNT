//
// Copyright (c) 2022 ZettaScale Technology
//
// This program and the accompanying materials are made availab&le under the
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

/// Used to represent the rotation of an actor or obstacle.
///
/// Rotations are applied in the order: Roll (X), Pitch (Y), Yaw (Z).
/// A 90-degree "Roll" maps the positive Z-axis to the positive Y-axis.
/// A 90-degree "Yaw" maps the positive X-axis to the positive Y-axis.
/// A 90-degree "Pitch" maps the positive X-axis to the positive Z-axis.
#[derive(Deserialize, Serialize, PartialEq, Debug)]
pub struct Rotation {
    pub pitch: f64,
    pub yaw: f64,
    pub roll: f64,
}

impl Rotation {
    pub fn from_json(json: impl AsRef<str>) -> Self {
        let res = serde_json::from_str(json.as_ref());
        if let Err(e) = res {
            panic!(
                "Could not deserialize Rotation:\n\t(json) {}\n\t(error) {:?}",
                json.as_ref(),
                e
            );
        }

        res.unwrap()
    }

    pub fn as_array(&self) -> [f64; 3] {
        [self.roll, self.pitch, self.yaw]
    }
}

#[cfg(test)]
mod tests {
    use super::Rotation;

    #[test]
    fn test_from_json() {
        assert_eq!(
            Rotation {
                pitch: 0.1,
                yaw: 0.2,
                roll: 0.3
            },
            Rotation::from_json(r#"{ "yaw": 0.2, "pitch": 0.1, "roll": 0.3 }"#)
        )
    }
}
