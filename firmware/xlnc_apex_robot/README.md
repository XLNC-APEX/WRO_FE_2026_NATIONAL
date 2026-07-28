# XLNC APEX robot code

Our code is a Rust project that runs on Raspberry Pi Pico 2 (RP2350).

We use [Embassy](https://embassy.dev/) async framework

## Running

Usually we run it through some debug probe(Any SWD probe should work). Probe needs to connect the Pico 2's debug pins, as well as, your pc. Required: [`probe-rs`](https://probe.rs/docs/tools/cargo-embed/)

<sub>*I prefer using `--release` for smaller size, faster flashes. Note that it disables some debug info like line number when printing.*</sub>

Through debug probe:

```sh
cargo embed --release
```

Specify [log level](https://docs.rs/defmt/latest/defmt/):

```sh
DEFMT_LOG=trace cargo embed --release
```

Without debug probe:

- Requires usb connection to Pico 2,

- Requires [`picotool`](https://github.com/raspberrypi/picotool.git) installed
- No logging

```sh
cargo run --release
```

### Host tests

```sh
cargo test --release --target=host-tuple --lib
```

## Policy

No AI generated code.
