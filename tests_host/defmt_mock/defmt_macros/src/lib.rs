extern crate proc_macro;
use proc_macro::TokenStream;

/// No-op derive macro for `defmt::Format` — allows host compilation of firmware code.
#[proc_macro_derive(Format)]
pub fn format_derive(_input: TokenStream) -> TokenStream {
    TokenStream::new()
}
