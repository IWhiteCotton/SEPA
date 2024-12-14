#include <neorv32.h>
#include <stdint.h> 

// Registers
#define PERIPHERAL_BASE_1 0x90000000
#define PERIPHERAL_BASE_2 0x90000010

// UART baud rate
#define BAUD_RATE 19200

int main() {

  // Init UART (primary UART = UART0; if no id number is specified the primary UART is used) at default baud rate, no parity bits, ho hw flow control
  neorv32_uart0_setup(BAUD_RATE, PARITY_NONE, FLOW_CONTROL_NONE);

  // Capture all exceptions and give debug info via UART
  neorv32_rte_setup();

  while(1){
    // Read and print IR sensor value
    uint32_t value = neorv32_cpu_load_unsigned_word (PERIPHERAL_BASE_2);
    neorv32_uart0_printf("Read: %x\n",value);
    neorv32_cpu_delay_ms(500);
  }

  return 0;
}
