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

#[derive(Deserialize, Serialize, Debug, PartialEq)]
pub struct VehicleControl {
    pub throttle: f64,
    pub steer: f64,
    pub brake: f64,
    pub hand_brake: bool,
    pub reverse: bool,
    pub manual_gear_shift: bool,
    pub gear: u8,
}

impl VehicleControl {
    pub fn from_json(json: impl AsRef<str>) -> Self {
        let res = serde_json::from_str(json.as_ref());
        if let Err(e) = res {
            panic!(
                "Could not deserialize VehicleControl:\n\t(json) {}\n\t(error) {:?}",
                json.as_ref(),
                e
            );
        }

        res.unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::VehicleControl;

    #[test]
    fn test_from_json() {
        assert_eq!(
            VehicleControl {
                throttle: 0.1,
                steer: 0.2,
                brake: 0.3,
                hand_brake: false,
                reverse: true,
                manual_gear_shift: true,
                gear: 7
            },
            VehicleControl::from_json(
                r#"{ "throttle": 0.1, "steer": 0.2, "hand_brake": false, "brake": 0.3, "reverse": true, "manual_gear_shift": true, "gear": 7 }"#
            )
        )
    }
}
