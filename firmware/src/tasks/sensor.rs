//! 200 Hz position-sensor acquisition.

use embassy_nrf::saadc::Saadc;
use embassy_time::{Duration, Instant, Ticker};

use crate::shared::{self, inject, Sample, TaskId, LATEST_SAMPLE};

const PERIOD: Duration = Duration::from_millis(5);
/// 12-bit single-ended full scale.
const FULL_SCALE: f32 = 4095.0;

#[embassy_executor::task]
pub async fn sensor_task(mut adc: Saadc<'static, 1>) {
    adc.calibrate().await;
    let mut ticker = Ticker::every(PERIOD);
    let mut buf = [0i16; 1];
    loop {
        ticker.next().await;
        adc.sample(&mut buf).await;
        shared::heartbeat(TaskId::Sensor);

        if inject::get(&inject::FREEZE_SENSOR) {
            // Task alive, data not updating: only the staleness monitor can see this.
            continue;
        }
        // Single-ended readings can be slightly negative around 0 V.
        let raw = buf[0].max(0) as f32 / FULL_SCALE;
        LATEST_SAMPLE.lock(|s| {
            s.set(Some(Sample {
                raw,
                at: Instant::now(),
            }))
        });
    }
}
