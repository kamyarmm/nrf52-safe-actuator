//! Reset-surviving fault log, reset-cause decoding and the panic handler.

use core::mem::MaybeUninit;
use core::ptr::addr_of_mut;

use embassy_nrf::pac;
use safety_core::{FaultCode, FaultLog, ResetCause};

pub const LOG_CAPACITY: usize = 32;
pub type Log = FaultLog<LOG_CAPACITY>;

/// Placed in `.uninit` so the startup code neither zeroes nor initialises it; contents
/// survive watchdog, lockup and software resets (not power loss).
#[link_section = ".uninit.FAULT_LOG"]
static mut FAULT_LOG: MaybeUninit<Log> = MaybeUninit::uninit();

/// Run `f` on the log inside a critical section.
///
/// The log is copied in and out with volatile accesses. Volatile keeps the compiler
/// from reasoning about the "uninitialised" static, and `Log` is `repr(C)` with only
/// integer fields, so whatever bytes survived the reset form a valid value that the
/// CRC check in `safety-core` then accepts or rejects.
pub fn with_log<R>(f: impl FnOnce(&mut Log) -> R) -> R {
    cortex_m::interrupt::free(|_| {
        // SAFETY: exclusive access is guaranteed by the critical section (single core),
        // the pointer is valid and aligned for `Log`, and any bit pattern is a valid
        // `Log` (see above).
        unsafe {
            let p = addr_of_mut!(FAULT_LOG).cast::<Log>();
            let mut log = core::ptr::read_volatile(p);
            let r = f(&mut log);
            core::ptr::write_volatile(p, log);
            r
        }
    })
}

/// Append a fault record stamped with the current uptime.
pub fn record(code: FaultCode) {
    let t = embassy_time::Instant::now().as_millis() as u32;
    with_log(|log| log.push(code, t));
}

/// Read and clear `POWER.RESETREAS`. The register is cumulative, so it must be
/// cleared every boot for the next reading to be meaningful.
pub fn take_reset_cause() -> ResetCause {
    let reg = pac::POWER.resetreas();
    let r = reg.read();
    reg.write_value(pac::power::regs::Resetreas(0xFFFF_FFFF));
    if r.dog() {
        ResetCause::Watchdog
    } else if r.lockup() {
        ResetCause::Lockup
    } else if r.sreq() {
        ResetCause::Software
    } else if r.resetpin() {
        ResetCause::ResetPin
    } else if r.0 == 0 {
        ResetCause::PowerOn
    } else {
        ResetCause::Other
    }
}

/// Outcome of the boot-time log check.
pub struct BootRecord {
    pub boot: u32,
    pub log_was_valid: bool,
    pub panic_in_previous_boot: bool,
}

/// Validate the log, start a new boot and record why the previous one ended.
pub fn start_boot(cause: ResetCause) -> BootRecord {
    with_log(|log| {
        let log_was_valid = log.validate_or_reset();
        let previous = log.boot_count();
        let panic_in_previous_boot =
            log_was_valid && log.recorded_in_boot(FaultCode::PanicReset, previous);
        let boot = log.start_boot();
        match cause {
            ResetCause::Watchdog => log.push(FaultCode::WatchdogReset, 0),
            // RAM content is undefined after power-up, so an invalid log is expected
            // then and only worth recording after a warm reset.
            ResetCause::PowerOn => {}
            _ if !log_was_valid => log.push(FaultCode::LogCorrupted, 0),
            _ => {}
        }
        BootRecord {
            boot,
            log_was_valid,
            panic_in_previous_boot,
        }
    })
}

/// Print the whole log over RTT.
pub fn dump() {
    let log = with_log(|log| *log);
    defmt::info!(
        "fault log: {} record(s), boot #{}",
        log.len(),
        log.boot_count()
    );
    for r in log.iter() {
        match r.code() {
            Some(code) => defmt::info!("  boot {} @ {} ms: {}", r.boot(), r.timestamp_ms(), code),
            None => defmt::info!("  boot {} @ {} ms: <unknown>", r.boot(), r.timestamp_ms()),
        }
    }
}

/// Record the panic, then reset. The next boot sees `PanicReset` in the previous boot's
/// records and starts in the safe state.
#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    cortex_m::interrupt::disable();
    defmt::error!("panic: {}", defmt::Display2Format(info));
    record(FaultCode::PanicReset);
    cortex_m::peripheral::SCB::sys_reset()
}

#[defmt::panic_handler]
fn defmt_panic() -> ! {
    core::panic!("defmt panic")
}
