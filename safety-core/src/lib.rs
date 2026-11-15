//! Hardware-independent safety logic for a fault-tolerant actuator controller.
//!
//! This crate contains everything that decides *whether* the actuator may move and
//! *how much*, with no dependency on a HAL, an executor or a clock source. Keeping it
//! `no_std` and side-effect free means the same code that runs on the nRF52840 is
//! exercised on the host by unit tests and by closed-loop failure-injection scenarios
//! (see `tests/fault_injection.rs`).
//!
//! Data flow for one control cycle:
//!
//! ```text
//!  raw sample ──► monitors ──► persistence filters ──► active faults ──► state machine
//!                                                                          │
//!  setpoint ──► envelope clamp ──► rate limiter ──► PI controller ──► authority limit ──► command
//! ```
//!
//! Time is passed in explicitly as milliseconds so that tests can control it.
#![cfg_attr(not(test), no_std)]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod control;
pub mod fault;
pub mod fault_log;
pub mod heartbeat;
pub mod manager;
pub mod monitor;
pub mod state;

pub use fault::{FaultCode, FaultSet, Severity};
pub use fault_log::{FaultLog, FaultRecord};
pub use heartbeat::HeartbeatMonitor;
pub use manager::{Config, CycleInput, CycleOutput, SafetyManager};
pub use state::{ResetCause, SystemState};
