use builtin_interfaces::msg::Time;

pub fn sim_time_to_ros_time(time_s: f64) -> Time {
    let total_ns = (time_s * 1.0e9).round() as i64;

    Time {
        sec: (total_ns / 1_000_000_000) as i32,
        nanosec: (total_ns % 1_000_000_000) as u32,
    }
}
