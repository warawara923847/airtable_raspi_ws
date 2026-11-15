//! External planar wrench at the vehicle COM, independent of actuator commands.

#[derive(Debug, Clone, Copy)]
pub enum WrenchFrame {
    World,
    Table,
    Body,
}

#[derive(Debug, Clone, Copy, Default)]
pub struct AppliedDisturbance {
    /// Beginning of the physics step to which this wrench was applied.
    pub time: f64,
    /// COM wrench expressed in body axes, for monitoring and recording.
    pub body_wrench: [f64; 3],
}

#[derive(Debug, Clone, Copy)]
struct Command {
    frame: WrenchFrame,
    wrench: [f64; 3],
    expires_at: f64,
}

#[derive(Debug, Default)]
pub struct Disturbance {
    command: Option<Command>,
    last_stamp: Option<f64>,
    pub applied: AppliedDisturbance,
}

impl Disturbance {
    pub fn clear(&mut self) {
        self.command = None;
    }

    pub fn set(
        &mut self,
        now: f64,
        stamp: Option<f64>,
        frame: WrenchFrame,
        wrench: [f64; 3],
        timeout: f64,
    ) -> Result<(), String> {
        let source_time = stamp.unwrap_or(now);
        if !wrench.iter().all(|value| value.is_finite())
            || !timeout.is_finite()
            || timeout <= 0.0
            || !source_time.is_finite()
            || source_time < 0.0
            || source_time - now > 0.05
            || now - source_time >= timeout
            || stamp.is_some_and(|value| self.last_stamp.is_some_and(|last| value <= last))
        {
            self.clear();
            return Err("Invalid, duplicate, stale or future disturbance command".into());
        }
        if let Some(value) = stamp {
            self.last_stamp = Some(value);
        }
        self.command = Some(Command {
            frame,
            wrench,
            expires_at: source_time + timeout,
        });
        Ok(())
    }

    pub fn advance(&mut self, time: f64, table_yaw: f64, body_yaw: f64) -> [f64; 6] {
        self.applied = AppliedDisturbance {
            time,
            body_wrench: [0.0; 3],
        };
        let Some(command) = self.command else {
            return [0.0; 6];
        };
        if time + 1e-12 >= command.expires_at {
            self.clear();
            return [0.0; 6];
        }
        let angle = match command.frame {
            WrenchFrame::World => 0.0,
            WrenchFrame::Table => table_yaw,
            WrenchFrame::Body => body_yaw,
        };
        let [fx, fy, mz] = command.wrench;
        let wx = angle.cos() * fx - angle.sin() * fy;
        let wy = angle.sin() * fx + angle.cos() * fy;
        self.applied.body_wrench = [
            body_yaw.cos() * wx + body_yaw.sin() * wy,
            -body_yaw.sin() * wx + body_yaw.cos() * wy,
            mz,
        ];
        // Cartesian world wrench, force first and torque last, ready for apply_ft.
        [wx, wy, 0.0, 0.0, 0.0, mz]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn body_and_table_forces_rotate_without_reordering_force_and_torque() {
        let mut disturbance = Disturbance::default();
        disturbance
            .set(0.0, None, WrenchFrame::Body, [1.0, 0.0, 0.2], 1.0)
            .unwrap();
        let pi_2 = std::f64::consts::FRAC_PI_2;
        let world = disturbance.advance(0.0, 0.0, pi_2);
        assert!(world[0].abs() < 1e-12);
        assert!((world[1] - 1.0).abs() < 1e-12);
        assert_eq!(&world[2..], &[0.0, 0.0, 0.0, 0.2]);
        assert!((disturbance.applied.body_wrench[0] - 1.0).abs() < 1e-12);
        disturbance
            .set(0.0, None, WrenchFrame::Table, [1.0, 0.0, 0.0], 1.0)
            .unwrap();
        assert!((disturbance.advance(0.0, pi_2, 0.0)[1] - 1.0).abs() < 1e-12);
        disturbance
            .set(0.0, None, WrenchFrame::World, [1.0, 0.0, 0.0], 1.0)
            .unwrap();
        assert_eq!(disturbance.advance(0.0, pi_2, pi_2)[0], 1.0);
    }

    #[test]
    fn timeout_invalid_and_duplicate_commands_remove_force() {
        let mut disturbance = Disturbance::default();
        disturbance
            .set(1.0, Some(1.0), WrenchFrame::World, [0.1, 0.0, 0.0], 0.25)
            .unwrap();
        assert_eq!(disturbance.advance(1.24, 0.0, 0.0)[0], 0.1);
        assert_eq!(disturbance.advance(1.25, 0.0, 0.0), [0.0; 6]);
        for (stamp, force) in [
            (Some(1.0), 0.1),
            (Some(0.5), 0.1),
            (Some(2.0), 0.1),
            (None, f64::NAN),
        ] {
            assert!(
                disturbance
                    .set(1.1, stamp, WrenchFrame::World, [force, 0.0, 0.0], 0.25)
                    .is_err()
            );
            assert_eq!(disturbance.advance(1.1, 0.0, 0.0), [0.0; 6]);
        }
    }
}
