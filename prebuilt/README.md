# Prebuilt images

Built from this source tree with Pico SDK 2.2.0 and arm-none-eabi-gcc 13.2.1 (Release). Both images use the default configuration and **boot in simulation mode**. Switch to the ADC with `SOURCE ADC`, then `CONFIG SAVE`.

| file | board | clk_sys |
|---|---|---|
| `edge_pq_pico2.uf2` | Raspberry Pi Pico 2 (RP2350, Arm cores) | 153.6 MHz |
| `edge_pq_pico.uf2` | Raspberry Pi Pico (RP2040) | 128 MHz |

To flash: hold BOOTSEL, plug in USB, and copy the file to the drive that appears. These images have **not** been run on physical hardware by the author of this build — see the main README.
SHA-256:

    f0091d9510b05475f17706d0c831047b2e398a8254f925b9db9be70bc268a972  edge_pq_pico.uf2
    cffb0bea792805d284cdf0a492a4660b271518f987c11b698a9ae1d20649e09f  edge_pq_pico2.uf2
