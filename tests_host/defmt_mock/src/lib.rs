#[macro_export]
macro_rules! info  { ($($t:tt)*) => {} }
#[macro_export]
macro_rules! warn  { ($($t:tt)*) => {} }
#[macro_export]
macro_rules! error { ($($t:tt)*) => {} }
#[macro_export]
macro_rules! trace { ($($t:tt)*) => {} }
#[macro_export]
macro_rules! debug { ($($t:tt)*) => {} }

// Re-export Format derive macro (no-op on host)
pub use defmt_macros::Format;
