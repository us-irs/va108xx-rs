[![build](https://github.com/us-irs/va108xx-rs/actions/workflows/ci.yml/badge.svg)](https://github.com/us-irs/va108xx-rs/actions/workflows/ci.yml)

Vorago VA108xx Rust Support
=========

This crate collection provides support to write Rust applications for the VA108XX family
of devices.

## List of crates

This workspace contains the following released crates:

- The [`va108xx`](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/va108xx) PAC
  crate containing basic low-level register definition.
- The [`va108xx-hal`](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/va108xx-hal)
  HAL crate containing higher-level abstractions on top of the PAC register crate.
- The [`vorago-reb1`](https://egit.irs.uni-stuttgart.de/rust/va108xx-rs/src/branch/main/vorago-reb1)
  BSP crate containing support for the REB1 development board.

It also contains the following helper crates:

- The `board-tests` contains an application which can be used to test the libraries on the
  board.
- The `examples` crates contains various example applications for the HAL and the PAC.

## Using the `.cargo/config.toml` file

Use the following command to have a starting `config.toml` file

```sh
cp .cargo/def-config.toml .cargo/config.toml
```

You then can adapt the `config.toml` to your needs. For example, you can configure runners
to conveniently flash with `cargo run`.

## Using the sample VS Code files

Use the following command to have a starting configuration for VS Code:

```sh
cp -rT vscode .vscode
```

You can then adapt the files in `.vscode` to your needs.

## Flashing, running and debugging the software

You can use CLI or VS Code for flashing, running and debugging. In any case, take
care of installing the pre-requisites first.

### Pre-Requisites

1. [SEGGER J-Link tools](https://www.segger.com/downloads/jlink/) installed
2. [gdb-multiarch](https://packages.debian.org/sid/gdb-multiarch) or similar
   cross-architecture debugger installed. All commands here assume `gdb-multiarch`.

### Using CLI

You can build the blinky example application with the following command

```sh
cargo build --example blinky
```

Start the GDB server first. The server needs to be started with a certain configuration and with
a JLink script to disable ROM protection.
For example, on Debian based system the following command can be used to do this (this command
is also run when running the `jlink-gdb.sh` script)

```sh
JLinkGDBServer -select USB -device Cortex-M0 -endian little -if JTAG-speed auto \
  -LocalhostOnly
```

After this, you can flash and debug the application with the following command

```sh
gdb-mutliarch -q -x jlink/jlink.gdb target/thumbv6m-none-eabihf/debug/examples/blinky
```

Please note that you can automate all steps except starting the GDB server by using a cargo
runner configuration, for example with the following lines in your `.cargo/config.toml` file:

```toml
[target.'cfg(all(target_arch = "arm", target_os = "none"))']
runner = "gdb-multiarch -q -x jlink/jlink.gdb"
```

After that, you can simply use `cargo run --example blinky` to flash the blinky
example.

### Using VS Code

Assuming a working debug connection to your VA108xx board, you can debug using VS Code with
the [`Cortex-Debug` plugin](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug).

Some sample configuration files for VS code were provided and can be used by running
`cp -rT vscode .vscode` like specified above. After that, you can use `Run and Debug`
to automatically rebuild and flash your application.

If you would like to use a custom GDB application, you can specify the gdb binary in the following
configuration variables in your `settings.json`:

- `"cortex-debug.gdbPath"`
- `"cortex-debug.gdbPath.linux"`
- `"cortex-debug.gdbPath.windows"`
- `"cortex-debug.gdbPath.osx"`
