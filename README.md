# iis3dwb-rs

This repo was cloned to be adapted from `lis3dh`, to `iis3dwb`. 
They are similiar chips, therefore shouldn't be a great issue. 

<!--- [![Crate][crate-image]][crate-link]
[![Docs][docs-image]][docs-link] --->
![MIT licensed][license-image]

Platform-agnostic Rust driver for the ST IIS3DWB 3-axis MEMS accelerometer.

## Resources

- [IIS3DWB product page][product-page]
- [IIS3DWB datasheet][datasheet]

### Examples

Run the examples in windows
```powershell
PS $Env:DEFMT_LOG = "info"
PS cargo run --example nrf52dk-fifo 
```

## License

Dual licensed under your choice of either of:

 - Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE) or
   http://www.apache.org/licenses/LICENSE-2.0)
 - MIT license ([LICENSE-MIT](LICENSE-MIT) or
   http://opensource.org/licenses/MIT)

### Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted
for inclusion in the work by you, as defined in the Apache-2.0 license, shall
be dual licensed as above, without any additional terms or conditions.

<!-- [crate-image]:  -->
[crate-link]: https://crates.io/crates/lis3dh
<!-- [docs-image]:  -->
[docs-link]: https://docs.rs/lis3dh/
[license-image]: https://img.shields.io/badge/license-Apache2.0/MIT-blue.svg
[product-page]: https://www.st.com/en/mems-and-sensors/iis3dwb.html
[datasheet]: https://www.st.com/en/mems-and-sensors/iis3dwb.pdf
