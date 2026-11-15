//! Fixed-size fault log designed to live in RAM that survives a reset.
//!
//! The firmware places one `FaultLog` in a `.uninit` linker section, which the
//! startup code does not zero. After any reset the log is integrity-checked
//! (magic + bounds + CRC-32); if the check fails (cold power-up, corruption, layout
//! change) it is reinitialised. This gives post-mortem diagnostics across watchdog
//! and panic resets without wearing flash on every fault.
//!
//! The layout is `#[repr(C)]` and contains only integers, so any bit pattern is a
//! valid value and validation can be done safely after the fact.

use crate::fault::FaultCode;

const MAGIC: u32 = 0x464C_4F47; // "FLOG"

/// One entry in the log.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct FaultRecord {
    timestamp_ms: u32,
    boot: u16,
    code: u8,
    _reserved: u8,
}

impl FaultRecord {
    /// Milliseconds since that boot started.
    pub const fn timestamp_ms(&self) -> u32 {
        self.timestamp_ms
    }

    /// Boot number (lower 16 bits) during which the fault was recorded.
    pub const fn boot(&self) -> u16 {
        self.boot
    }

    /// Decoded fault code, `None` if the stored value is unknown.
    pub const fn code(&self) -> Option<FaultCode> {
        FaultCode::from_u8(self.code)
    }
}

/// Ring buffer of the most recent `N` fault records plus a boot counter.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct FaultLog<const N: usize> {
    magic: u32,
    crc: u32,
    boot_count: u32,
    head: u16,
    len: u16,
    records: [FaultRecord; N],
}

impl<const N: usize> Default for FaultLog<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> FaultLog<N> {
    /// An empty, sealed log.
    pub fn new() -> Self {
        let mut log = Self {
            magic: MAGIC,
            crc: 0,
            boot_count: 0,
            head: 0,
            len: 0,
            records: [FaultRecord::default(); N],
        };
        log.seal();
        log
    }

    /// Structural and CRC check.
    pub fn is_valid(&self) -> bool {
        N > 0
            && N <= u16::MAX as usize
            && self.magic == MAGIC
            && (self.head as usize) < N
            && (self.len as usize) <= N
            && self.crc == self.compute_crc()
    }

    /// Reinitialise the log if it is not valid. Returns `true` if the existing
    /// contents were kept.
    pub fn validate_or_reset(&mut self) -> bool {
        if self.is_valid() {
            true
        } else {
            *self = Self::new();
            false
        }
    }

    /// Increment the boot counter at start-up and return the new boot number.
    pub fn start_boot(&mut self) -> u32 {
        self.boot_count = self.boot_count.wrapping_add(1);
        self.seal();
        self.boot_count
    }

    /// Current boot number.
    pub const fn boot_count(&self) -> u32 {
        self.boot_count
    }

    /// Append a record, overwriting the oldest one when full.
    pub fn push(&mut self, code: FaultCode, timestamp_ms: u32) {
        if N == 0 {
            return;
        }
        let slot = (self.head as usize + self.len as usize) % N;
        self.records[slot] = FaultRecord {
            timestamp_ms,
            boot: self.boot_count as u16,
            code: code as u8,
            _reserved: 0,
        };
        if (self.len as usize) < N {
            self.len += 1;
        } else {
            self.head = ((self.head as usize + 1) % N) as u16;
        }
        self.seal();
    }

    /// Number of stored records.
    pub const fn len(&self) -> usize {
        self.len as usize
    }

    /// `true` when no record is stored.
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Records from oldest to newest.
    pub fn iter(&self) -> impl Iterator<Item = &FaultRecord> {
        (0..self.len as usize).map(move |i| &self.records[(self.head as usize + i) % N])
    }

    /// Whether `code` was recorded during boot number `boot`.
    pub fn recorded_in_boot(&self, code: FaultCode, boot: u32) -> bool {
        self.iter()
            .any(|r| r.code() == Some(code) && r.boot == boot as u16)
    }

    /// Remove all records but keep the boot counter.
    pub fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
        self.seal();
    }

    fn seal(&mut self) {
        self.crc = self.compute_crc();
    }

    fn compute_crc(&self) -> u32 {
        let mut crc = Crc32::new();
        crc.update(&self.magic.to_le_bytes());
        crc.update(&self.boot_count.to_le_bytes());
        crc.update(&self.head.to_le_bytes());
        crc.update(&self.len.to_le_bytes());
        for r in &self.records {
            crc.update(&r.timestamp_ms.to_le_bytes());
            crc.update(&r.boot.to_le_bytes());
            crc.update(&[r.code, r._reserved]);
        }
        crc.finish()
    }

    #[cfg(test)]
    fn corrupt_for_test(&mut self) {
        self.records[0].timestamp_ms ^= 1;
    }
}

/// Bitwise CRC-32 (IEEE 802.3, reflected, poly 0xEDB88320). Small and table-free;
/// the log is only checked at boot and sealed on (rare) fault events.
struct Crc32(u32);

impl Crc32 {
    const fn new() -> Self {
        Self(0xFFFF_FFFF)
    }

    fn update(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.0 ^= b as u32;
            for _ in 0..8 {
                let mask = (self.0 & 1).wrapping_neg();
                self.0 = (self.0 >> 1) ^ (0xEDB8_8320 & mask);
            }
        }
    }

    const fn finish(&self) -> u32 {
        !self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc32_check_value() {
        let mut c = Crc32::new();
        c.update(b"123456789");
        assert_eq!(c.finish(), 0xCBF4_3926);
    }

    #[test]
    fn new_log_is_valid_and_empty() {
        let log: FaultLog<4> = FaultLog::new();
        assert!(log.is_valid());
        assert!(log.is_empty());
    }

    #[test]
    fn ring_overwrites_oldest() {
        let mut log: FaultLog<3> = FaultLog::new();
        log.start_boot();
        for (i, code) in [
            FaultCode::SensorStale,
            FaultCode::TrackingError,
            FaultCode::DeadlineOverrun,
            FaultCode::HeartbeatLost,
        ]
        .into_iter()
        .enumerate()
        {
            log.push(code, i as u32);
        }
        let codes: Vec<_> = log.iter().map(|r| r.code().unwrap()).collect();
        assert_eq!(
            codes,
            [
                FaultCode::TrackingError,
                FaultCode::DeadlineOverrun,
                FaultCode::HeartbeatLost
            ]
        );
        assert!(log.is_valid());
    }

    #[test]
    fn corruption_is_detected_and_reset() {
        let mut log: FaultLog<4> = FaultLog::new();
        log.push(FaultCode::SensorStale, 1);
        log.corrupt_for_test();
        assert!(!log.is_valid());
        assert!(!log.validate_or_reset());
        assert!(log.is_valid() && log.is_empty());
    }

    #[test]
    fn garbage_bounds_are_rejected_before_indexing() {
        let mut log: FaultLog<4> = FaultLog::new();
        log.head = 900;
        log.seal(); // even with a matching CRC, bounds must fail
        assert!(!log.is_valid());
    }

    #[test]
    fn records_are_tagged_with_boot_number() {
        let mut log: FaultLog<8> = FaultLog::new();
        let b1 = log.start_boot();
        log.push(FaultCode::PanicReset, 10);
        let b2 = log.start_boot();
        assert!(log.recorded_in_boot(FaultCode::PanicReset, b1));
        assert!(!log.recorded_in_boot(FaultCode::PanicReset, b2));
    }
}
