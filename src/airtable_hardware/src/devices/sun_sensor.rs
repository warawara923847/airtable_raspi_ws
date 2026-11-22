//! Periodic nanoSSOC-D60 reads on their own thread.
//!
//! The hardware worker times the thruster valves every 1 ms, so it must not wait on the
//! I2C bus. A missing sensor is not fatal: the node keeps running and publishes the error.
use std::sync::mpsc::{self, RecvTimeoutError, Sender};
use std::thread::JoinHandle;
use std::time::{Duration, Instant};

use crate::drivers::nanossoc_d60::{NanoSsocD60, SunObservation, describe_error};

const MIN_LOG_INTERVAL: Duration = Duration::from_secs(5);

/// Logs status changes, at most one per interval so a flickering status cannot flood the log.
#[derive(Default)]
struct StatusLog {
    last: Option<(u8, Instant)>,
}

impl StatusLog {
    fn should_log(&mut self, code: u8, now: Instant) -> bool {
        let due = match self.last {
            None => code != 0,
            Some((last, at)) => code != last && now.duration_since(at) >= MIN_LOG_INTERVAL,
        };
        if due {
            self.last = Some((code, now));
        }
        due
    }
}

pub struct SunSensorThread {
    stop: Option<Sender<()>>,
    thread: Option<JoinHandle<()>>,
}

impl SunSensorThread {
    pub fn spawn(
        mut driver: NanoSsocD60,
        period: Duration,
        mut on_observation: impl FnMut(SunObservation) + Send + 'static,
    ) -> Result<Self, String> {
        if period.is_zero() {
            return Err("sun sensor period must be positive".into());
        }
        let (stop, stopped) = mpsc::channel::<()>();
        let thread = std::thread::Builder::new()
            .name("sun_sensor".into())
            .spawn(move || {
                let mut log = StatusLog::default();
                let mut next = Instant::now();
                loop {
                    let observation = driver.observe();
                    let now = Instant::now();
                    if log.should_log(observation.error_code, now) {
                        eprintln!(
                            "[hardware] sun sensor status {}: {}",
                            observation.error_code,
                            describe_error(observation.error_code)
                        );
                    }
                    on_observation(observation);
                    next += period;
                    let now = Instant::now();
                    if next < now {
                        next = now; // The bus took longer than one period; do not catch up.
                    }
                    match stopped.recv_timeout(next - now) {
                        Err(RecvTimeoutError::Timeout) => {}
                        _ => break,
                    }
                }
            })
            .map_err(|error| format!("sun sensor thread: {error}"))?;
        Ok(Self {
            stop: Some(stop),
            thread: Some(thread),
        })
    }
}

impl Drop for SunSensorThread {
    fn drop(&mut self) {
        drop(self.stop.take()); // Disconnecting wakes the thread at once.
        if let Some(thread) = self.thread.take()
            && thread.join().is_err()
        {
            eprintln!("[hardware] sun sensor thread panicked");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::drivers::nanossoc_d60::DummySunBus;
    use std::sync::{Arc, Mutex};

    #[test]
    fn thread_delivers_observations_and_stops_promptly() {
        let seen = Arc::new(Mutex::new(0_u32));
        let counter = Arc::clone(&seen);
        let thread = SunSensorThread::spawn(
            NanoSsocD60::new(Box::new(DummySunBus::new())),
            Duration::from_millis(5),
            move |observation| {
                assert!(observation.sun_vector.is_some());
                *counter.lock().unwrap() += 1;
            },
        )
        .unwrap();
        std::thread::sleep(Duration::from_millis(60));
        let start = Instant::now();
        drop(thread);
        assert!(start.elapsed() < Duration::from_millis(50));
        assert!(*seen.lock().unwrap() >= 3);
    }

    #[test]
    fn slow_period_does_not_delay_shutdown() {
        let thread = SunSensorThread::spawn(
            NanoSsocD60::new(Box::new(DummySunBus::new())),
            Duration::from_secs(30),
            |_| {},
        )
        .unwrap();
        let start = Instant::now();
        drop(thread);
        assert!(start.elapsed() < Duration::from_millis(100));
    }

    #[test]
    fn status_log_reports_changes_without_flooding() {
        let mut log = StatusLog::default();
        let start = Instant::now();
        assert!(!log.should_log(0, start)); // Healthy from the start: nothing to say.
        assert!(log.should_log(0x14, start));
        assert!(!log.should_log(0x14, start + Duration::from_secs(60)));
        assert!(!log.should_log(0, start + Duration::from_secs(1))); // Throttled flicker.
        assert!(log.should_log(0, start + Duration::from_secs(6))); // Recovery is reported.
    }
}
