// No-op stubs for defmt macros that "consume" their arguments so the host
// build doesn't fire dead_code/unused_variables on values that exist only to
// be logged. Each $arg is bound to `_` via a reference so the compiler sees
// the read without changing semantics.

#[macro_export]
macro_rules! info {
    ($fmt:literal $(, $arg:expr)* $(,)?) => {{ $( let _ = &$arg; )* }};
}
#[macro_export]
macro_rules! warn {
    ($fmt:literal $(, $arg:expr)* $(,)?) => {{ $( let _ = &$arg; )* }};
}
#[macro_export]
macro_rules! error {
    ($fmt:literal $(, $arg:expr)* $(,)?) => {{ $( let _ = &$arg; )* }};
}
#[macro_export]
macro_rules! trace {
    ($fmt:literal $(, $arg:expr)* $(,)?) => {{ $( let _ = &$arg; )* }};
}
#[macro_export]
macro_rules! debug {
    ($fmt:literal $(, $arg:expr)* $(,)?) => {{ $( let _ = &$arg; )* }};
}

// Re-export Format derive macro (no-op on host)
pub use defmt_macros::Format;
