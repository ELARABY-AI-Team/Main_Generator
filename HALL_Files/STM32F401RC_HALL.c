{
  "rules": {
    "core_includes": {
      "description": "MCAL.h file must begin with all required `#include` statements. This includes the core MCU header file (like stm32f4xx.h or device-specific header), and standard C library includes if any standard functions are used. If unsure which device header to include, insert a placeholder with a comment.",
      "examples": [
        "#include \"stm32f401xc.h\"  // Core device header",
        "#include <stdint.h>",
        "#include <stdbool.h>",
        "#include <stddef.h>",
        "#include <string.h>",
        "#include <stdio.h>",
        "#include <stdlib.h>",
        "#include <math.h>"
      ]
    },
    "WDT_Reset_definition": {
      "description": "Watchdog timer clear implementation",
      "example": "For HOLTEK HT46R24: ClrWdt();"
    },
    "sleep_mode_definition": {
      "description": "Sleep mode stops executing code and peripherals (except OS timer)",
      "example": "For HOLTEK HT46R24: _halt();"
    },
    "file_structure": {
      "description": "Build MCAL Layer contains 2 Files (.h) and (.c)",
      "details": {
        "header_file": {
          "purpose": "Include APIs & type definitions",
          "example": "typedef enum { Vsource_3V = 0, Vsource_5V } t_sys_volt_type; void MCU_Config_Init(t_sys_volt sys_volt);"
        },
        "source_file": {
          "purpose": "Implementation of API bodies"
        }
      }
    }
  }
}