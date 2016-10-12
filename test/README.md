# Test if the sources compile

Cross compile test files which include the STM32F4 CMSIS Core headers,
with all warnings enabled, with the standard `arm-none-eabi-` GCC compiler.

To run the test, go to the packet folder and start the script:

```
PATH=/usr/local/gcc-arm-none-eabi-4_9-2015q3/bin:$PATH
cd stm32f4-cmsis-pack
bash ../../../scripts/run-tests.sh
```

The script will create `build/testa`, copy the source files and compile them.

