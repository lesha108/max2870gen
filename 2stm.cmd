cargo build --release
cargo objcopy --bin max2870gen --target thumbv7m-none-eabi --release -- -O binary max2870gen.bin
rem st-flash erase
rem st-flash write max2870gen.bin 0x8000000
rem cargo embed --release --chip STM32F103RE
cargo embed --release --chip STM32F103C8

