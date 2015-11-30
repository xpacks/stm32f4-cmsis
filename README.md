# STM32F4 CMSIS

This project, available from [GitHub](https://github.com/xpacks/stm32f4-cmsis),
includes the STM32F4 CMSIS files.

## Version

* v2.4.0

## Documentation

The latest CMSIS documentation is available from
[keil.com](http://www.keil.com/cmsis).

## Original files

The original files are available from the `originals` branch.

These files were extracted from `Keil.STM32F4xx_DFP.2.4.0.pack`.

To save space, the following folders/files were removed:

* all non-portable *.exe files
* _htmresc
* CMSIS/Flash
* Documentation
* Drivers/BSP/
* Drivers/CMSIS/CMSIS?END*.*
* Drivers/CMSIS/index.html
* Drivers/CMSIS/README.txt
* Drivers/CMSIS/Documentation
* Drivers/CMSIS/Include
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

## Vectors

The `vectors_*.c` files were created using the ARM assembly files.

```
$ bash scripts/generate-vectors.sh
startup_stm32f401xc.s -> vectors_stm32f401xc.c
startup_stm32f401xe.s -> vectors_stm32f401xe.c
startup_stm32f405xx.s -> vectors_stm32f405xx.c
startup_stm32f407xx.s -> vectors_stm32f407xx.c
startup_stm32f411xe.s -> vectors_stm32f411xe.c
startup_stm32f415xx.s -> vectors_stm32f415xx.c
startup_stm32f417xx.s -> vectors_stm32f417xx.c
startup_stm32f427xx.s -> vectors_stm32f427xx.c
startup_stm32f429xx.s -> vectors_stm32f429xx.c
startup_stm32f437xx.s -> vectors_stm32f437xx.c
startup_stm32f439xx.s -> vectors_stm32f439xx.c
```

