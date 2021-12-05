[![Crates.io](https://img.shields.io/crates/v/va108xx)](https://crates.io/crates/va108xx)
[![build](https://github.com/us-irs/va108xx-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/us-irs/va108xx-rs/actions/workflows/ci.yml)
[![docs.rs](https://img.shields.io/docsrs/va108xx)](https://docs.rs/va108xx)

# PAC for the Vorago VA108xx microcontroller family

This repository contains the Peripheral Access Crate (PAC) for
Voragos VA108xx series of Cortex-M0 based microcontrollers.

The crate was generated using [`svd2rust`](https://github.com/rust-embedded/svd2rust).

If you are interested in higher-level abstractions, it is recommended you visit
the [`va108xx-hal` HAL crate](https://github.com/robamu-org/va108xx-hal-rs) and
the [`vorago-reb1` BSP crate](https://github.com/robamu-org/vorago-reb1-rs) which build on top of
this PAC and provide application examples as well.

## Usage

To use this crate, add this to your `Cargo.toml`

```toml
[dependencies.va108xx]
version = "<Most Recent Version>"
features = ["rt"]
```

The `rt` feature is optional and recommended. It brings in support for `cortex-m-rt`.

For full details on the autgenerated API, please see the
[svd2rust documentation](https://docs.rs/svd2rust/0.19.0/svd2rust/#peripheral-api).

## Regenerating the PAC

The base file used by `svd2rust` is generated using the `svdtools` package and a
YAML patch file. You can create the patched file by running this command after installing
the Python [`svdtools` package](https://github.com/stm32-rs/svdtools):

```sh
cd svd
svd patch va108xx-patch.yml
```

After that, you can regenerate the PAC by running the `gen-helper.sh` helper script.
