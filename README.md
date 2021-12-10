# Vorago Rust Workspace

Workspace for developing Rust code for the Vorago devices

After cloning, run

```sh
git submodule init
git submodule update
```

## Use VS Code files

Make sure to install the `Cortex-Debug` extension first.

The `tasks.json` and the `launch.json` files are generic and you can use them immediately by
opening the folder in VS code or adding it to a workspace.

If you would like to use a custom GDB application, you can specify the gdb binary in the following
configuration variables in your `settings.json`:

- `"cortex-debug.gdbPath"`
- `"cortex-debug.gdbPath.linux"`
- `"cortex-debug.gdbPath.windows"`
- `"cortex-debug.gdbPath.osx"`
