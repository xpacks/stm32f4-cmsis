# STM32F4 CMSIS

This project, available from [GitHub](https://github.com/xpacks/stm32f4-cmsis),
includes the STM32F4 CMSIS files.

## Version

* v2.11.0

## Documentation

The latest CMSIS documentation is available from
[keil.com](http://www.keil.com/cmsis).

The list of latest packs is available from [keil.com](https://www.keil.com/dd2/pack/).

## Original files

The original files are available from the `originals` branch.

These files were extracted from `Keil.STM32F4xx_DFP.2.11.0.pack`.

To save space, the following folders/files were removed:

* all non-portable *.exe files
* \_htmresc
* CMSIS/Flash
* CMSIS/Debug
* Documentation
* Drivers/BSP/
* Drivers/CMSIS/CMSIS?END*.*
* Drivers/CMSIS/index.html
* Drivers/CMSIS/README.txt
* Drivers/CMSIS/Documentation
* Drivers/CMSIS/DSP_Lib
* Drivers/CMSIS/Include
* Drivers/CMSIS/Lib
* Drivers/CMSIS/RTOS
* Drivers/CMSIS/SVD
* Drivers/STM32F?xx_HAL_Driver/
* MDK
* Middlewares
* Projects
* Utilities
* package.xml

## Changes

The actual files used by the package are in the `xpack` branch.

Most of the files should be unchanged.

The following files were patched to silence warnings:

* stm32f446xx.h

## Vectors

The `vectors_*.c` files were created using the ARM assembly files.

```
$ bash scripts/generate-vectors.sh
startup_stm32f401xc.s -> vectors_stm32f401xc.c
startup_stm32f401xe.s -> vectors_stm32f401xe.c
startup_stm32f405xx.s -> vectors_stm32f405xx.c
startup_stm32f407xx.s -> vectors_stm32f407xx.c
startup_stm32f410cx.s -> vectors_stm32f410cx.c
startup_stm32f410rx.s -> vectors_stm32f410rx.c
startup_stm32f410tx.s -> vectors_stm32f410tx.c
startup_stm32f411xe.s -> vectors_stm32f411xe.c
startup_stm32f412cx.s -> vectors_stm32f412cx.c
startup_stm32f412rx.s -> vectors_stm32f412rx.c
startup_stm32f412vx.s -> vectors_stm32f412vx.c
startup_stm32f412zx.s -> vectors_stm32f412zx.c
startup_stm32f413xx.s -> vectors_stm32f413xx.c
startup_stm32f415xx.s -> vectors_stm32f415xx.c
startup_stm32f417xx.s -> vectors_stm32f417xx.c
startup_stm32f423xx.s -> vectors_stm32f423xx.c
startup_stm32f427xx.s -> vectors_stm32f427xx.c
startup_stm32f429xx.s -> vectors_stm32f429xx.c
startup_stm32f437xx.s -> vectors_stm32f437xx.c
startup_stm32f439xx.s -> vectors_stm32f439xx.c
startup_stm32f446xx.s -> vectors_stm32f446xx.c
startup_stm32f469xx.s -> vectors_stm32f469xx.c
startup_stm32f479xx.s -> vectors_stm32f479xx.c
```

## Warnings

To silence warnings when compiling the CMSIS drivers, use:

```
-Wno-aggregate-return -Wno-conversion -Wno-missing-prototypes \
-Wno-missing-declarations -Wno-unused-parameter -Wno-padded
```

## Tests

```
export PATH=/usr/local/gcc-arm-none-eabi-5_2-2015q4/bin:$PATH
bash ../../../scripts/run-tests.sh
```

