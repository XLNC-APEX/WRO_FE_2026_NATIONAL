# XLNC APEX robot code

Our code is a Rust project that runs on Raspberry Pi Pico 2 (RP2350).

We use [Embassy](https://embassy.dev/) async framework

## Running

Usually we run it through some debug probe(Any SWD probe should work). Probe needs to connect the Pico 2's debug pins, as well as, your pc. Required: [`probe-rs`](https://probe.rs/docs/tools/cargo-embed/)

Through debug probe:

```sh
cargo embed
```

Specify [log level](https://docs.rs/defmt/latest/defmt/):

```sh
DEFMT_LOG=trace cargo embed
```

Without debug probe:

- Requires usb connection to Pico 2,

- Requires [`picotool`](https://github.com/raspberrypi/picotool.git) installed
- No logging

```sh
cargo run
```
