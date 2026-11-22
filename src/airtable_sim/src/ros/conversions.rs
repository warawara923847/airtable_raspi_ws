use std::time::{SystemTime, UNIX_EPOCH};

use builtin_interfaces::msg::Time;

const NS_PER_S: i64 = 1_000_000_000;

/// Maps MuJoCo time (always starting at 0) to the ROS time of /clock and every stamp.
///
/// The origin is only added at the ROS boundary and in integer nanoseconds: at a Unix-epoch
/// origin an f64 of seconds resolves about 0.24 us, which would make stamps of one instant
/// differ between topics.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SimClock {
    origin_ns: i64,
}

impl SimClock {
    /// `wall`: the current system time in whole seconds; `zero`: ROS time equals MuJoCo time.
    pub fn from_origin(name: &str) -> Result<Self, String> {
        match name {
            "wall" => {
                let now = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .map_err(|error| format!("system clock is before 1970: {error}"))?;
                let seconds = i32::try_from(now.as_secs())
                    .map_err(|_| "system time does not fit a ROS time stamp".to_string())?;
                Ok(Self::with_origin_s(seconds))
            }
            "zero" => Ok(Self::with_origin_s(0)),
            other => Err(format!("clock_origin must be wall or zero, got {other:?}")),
        }
    }

    fn with_origin_s(seconds: i32) -> Self {
        Self {
            origin_ns: i64::from(seconds) * NS_PER_S,
        }
    }

    pub fn to_ros(self, time_s: f64) -> Time {
        let total_ns = self.origin_ns + (time_s * 1.0e9).round() as i64;

        Time {
            sec: total_ns.div_euclid(NS_PER_S) as i32,
            nanosec: total_ns.rem_euclid(NS_PER_S) as u32,
        }
    }

    /// MuJoCo time of a received ROS stamp; negative if it predates this simulation.
    pub fn to_sim(self, stamp: &Time) -> f64 {
        let total_ns = i64::from(stamp.sec) * NS_PER_S + i64::from(stamp.nanosec);
        (total_ns - self.origin_ns) as f64 * 1.0e-9
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const ORIGIN: i32 = 1_795_000_000;

    #[test]
    fn zero_origin_keeps_simulation_time() {
        let clock = SimClock::from_origin("zero").unwrap();
        let stamp = clock.to_ros(12.25);
        assert_eq!((stamp.sec, stamp.nanosec), (12, 250_000_000));
        assert_eq!(clock.to_sim(&stamp), 12.25);
    }

    #[test]
    fn wall_origin_is_exact_to_the_nanosecond() {
        let clock = SimClock::with_origin_s(ORIGIN);
        // 0.002 s steps accumulate in MuJoCo; the stamp must not pick up epoch rounding.
        let stamp = clock.to_ros(0.002 * 3.0);
        assert_eq!((stamp.sec, stamp.nanosec), (ORIGIN, 6_000_000));
        assert!((clock.to_sim(&stamp) - 0.006).abs() < 1e-12);
    }

    #[test]
    fn stamps_before_the_origin_are_negative_simulation_time() {
        let clock = SimClock::with_origin_s(ORIGIN);
        let earlier = Time {
            sec: ORIGIN - 1,
            nanosec: 500_000_000,
        };
        assert!((clock.to_sim(&earlier) + 0.5).abs() < 1e-12);
    }

    #[test]
    fn wall_origin_is_a_current_date_and_names_are_checked() {
        let clock = SimClock::from_origin("wall").unwrap();
        assert!(clock.to_ros(0.0).sec > 1_600_000_000);
        assert!(SimClock::from_origin("Wall").is_err());
    }
}
