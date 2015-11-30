/*
 * Copyright (c) 2015 Liviu Ionescu
 * This file is part of the xPacks project (https://xpacks.github.io).
 */

/*
 * Compile test for the STM32F4 CMSIS core header.
 */

#include "stm32f4xx.h"

int
main (int argc __attribute__((unused)), char* argv[] __attribute__((unused)))
{
  return 0;
}

void
_exit (int code);

void
_exit (int code __attribute__((unused)))
{
  for (;;)
    {
    }
}
