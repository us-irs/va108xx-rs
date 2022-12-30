Vorago Rust Workspace
========

Workspace for developing Rust code for the Vorago devices

After cloning, run

```sh
git submodule update --init
```

# Preparing the Rust installation


Building an application for the VA108XX family requires the `thumbv6m-none-eabi`
cross-compiler toolchain. If you have not installed it yet, you can do so with

```sh
rustup target add thumbv6m-none-eabi
```

# Debugging with VS Code

The REB1 board features an on-board JTAG, so all that is required to flash the board is a
Micro-USB cable and an 
You can debug applications on the REB1 board with a graphical user interface using VS Code with
the [`Cortex-Debug` plugin](https://marketplace.visualstudio.com/items?itemName=marus25.cortex-debug).

Some sample configuration files for VS code were provided as well. You can simply use `Run and Debug`
to automatically rebuild and flash your application.

The `tasks.json` and the `launch.json` files are generic and you can use them immediately by
opening the folder in VS code or adding it to a workspace.

If you would like to use a custom GDB application, you can specify the gdb binary in the following
configuration variables in your `settings.json`:

- `"cortex-debug.gdbPath"`
- `"cortex-debug.gdbPath.linux"`
- `"cortex-debug.gdbPath.windows"`
- `"cortex-debug.gdbPath.osx"`
