defmt Testapp
======

`defmt` is clunky to use without probe-rs and requires special configuration inside the
`.cargo/config.toml` file.

`probe-rs` is currently problematic for usage with the VA108xx , so it is not the default tool
recommended and used for the whole workspace. This project contains an isolated, `defmt` compatible
configuration for testing with `defmt` (and `probe-rs`).
