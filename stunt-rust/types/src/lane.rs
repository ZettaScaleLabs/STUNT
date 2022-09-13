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
use serde_repr::*;

#[derive(Serialize_repr, Deserialize_repr, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum LaneMarkingColor {
    White = 0,
    Blue = 1,
    Green = 2,
    Red = 3,
    Yellow = 4,
    Other = 5,
}

impl LaneMarkingColor {
    pub fn from_json(json: impl AsRef<str>) -> Self {
        let res = serde_json::from_str(json.as_ref());
        if let Err(e) = res {
            panic!(
                "Could not deserialize LaneMarkingColor:\n\t(json) {}\n\t(error) {:?}",
                json.as_ref(),
                e
            );
        }

        res.unwrap()
    }
}

#[derive(Serialize_repr, Deserialize_repr, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum LaneMarkingType {
    Other = 0,
    Broken = 1,
    Solid = 2,
    SolidSolid = 3,
    SolidBroken = 4,
    BrokenSolid = 5,
    BrokenBroken = 6,
    BottsDots = 7,
    Grass = 8,
    Curb = 9,
    None = 10,
}

impl LaneMarkingType {
    pub fn from_json(json: impl AsRef<str>) -> Self {
        let res = serde_json::from_str(json.as_ref());
        if let Err(e) = res {
            panic!(
                "Could not deserialize LaneMarkingType:\n\t(json) {}\n\t(error) {:?}",
                json.as_ref(),
                e
            );
        }

        res.unwrap()
    }
}

#[derive(Serialize_repr, Deserialize_repr, PartialEq, Eq, Debug)]
#[repr(u32)]
pub enum LaneType {
    None = 1,
    Driving = 2,
    Stop = 4,
    Shoulder = 8,
    Biking = 16,
    Sidewalk = 32,
    Border = 64,
    Restricted = 128,
    Parking = 256,
    Bidirectional = 512,
    Median = 1024,
    Special1 = 2048,
    Special2 = 4096,
    Special3 = 8192,
    Roadworks = 16384,
    Tram = 32768,
    Rail = 65536,
    Entry = 131072,
    Exit = 262144,
    OffRamp = 524288,
    OnRamp = 1048576,
    Any = 4294967294,
}

impl LaneType {
    pub fn from_json(json: impl AsRef<str>) -> Self {
        let res = serde_json::from_str(json.as_ref());
        if let Err(e) = res {
            panic!(
                "Could not deserialize LaneType:\n\t(json) {}\n\t(error) {:?}",
                json.as_ref(),
                e
            );
        }

        res.unwrap()
    }
}

#[derive(Serialize_repr, Deserialize_repr, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum LaneChange {
    None = 0,
    Right = 1,
    Left = 2,
    Both = 3,
}

impl LaneChange {
    pub fn from_json(json: impl AsRef<str>) -> Self {
        let res = serde_json::from_str(json.as_ref());
        if let Err(e) = res {
            panic!(
                "Could not deserialize LaneChange:\n\t(json) {}\n\t(error) {:?}",
                json.as_ref(),
                e
            );
        }

        res.unwrap()
    }
}

#[derive(Serialize_repr, Deserialize_repr, PartialEq, Eq, Debug)]
#[serde(tag = "road_option")]
#[repr(i8)]
pub enum RoadOption {
    Void = -1,
    Left = 1,
    Right = 2,
    Straight = 3,
    LaneFollow = 4,
    ChangeLaneLeft = 5,
    ChangeLaneRight = 6,
}

impl RoadOption {
    pub fn from_json(json: impl AsRef<str>) -> Self {
        let res = serde_json::from_str(json.as_ref());
        if let Err(e) = res {
            panic!(
                "Could not deserialize RoadOption:\n\t(json) {}\n\t(error) {:?}",
                json.as_ref(),
                e
            );
        }

        res.unwrap()
    }
}

#[derive(Serialize, Deserialize, PartialEq, Eq, Debug)]
pub struct LaneMarking {
    pub marking_color: LaneMarkingColor,
    pub marking_type: LaneMarkingType,
    pub lane_change: LaneChange,
}

impl LaneMarking {
    pub fn from_json(json: impl AsRef<str>) -> Self {
        let res = serde_json::from_str(json.as_ref());
        if let Err(e) = res {
            panic!(
                "Could not deserialize LaneMarking:\n\t(json) {}\n\t(error) {:?}",
                json.as_ref(),
                e
            );
        }

        res.unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_from_json() {
        assert_eq!(
            LaneMarking {
                marking_color: LaneMarkingColor::Red,
                marking_type: LaneMarkingType::BottsDots,
                lane_change: LaneChange::Both
            },
            LaneMarking::from_json(
                r#" { "marking_color": 3, "marking_type": 7, "lane_change": 3 } "#
            )
        );

        assert_eq!(RoadOption::Void, RoadOption::from_json("-1"));

        assert_eq!(LaneType::OffRamp, LaneType::from_json("524288"));
    }
}
