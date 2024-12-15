-- #################################################################################################
-- # << NEORV32 - Example setup including the bootloader, for the iCEBreaker (c) Board >>          #
-- # ********************************************************************************************* #
-- # BSD 3-Clause License                                                                          #
-- #                                                                                               #
-- # Copyright (c) 2021, Stephan Nolting. All rights reserved.                                     #
-- #                                                                                               #
-- # Redistribution and use in source and binary forms, with or without modification, are          #
-- # permitted provided that the following conditions are met:                                     #
-- #                                                                                               #
-- # 1. Redistributions of source code must retain the above copyright notice, this list of        #
-- #    conditions and the following disclaimer.                                                   #
-- #                                                                                               #
-- # 2. Redistributions in binary form must reproduce the above copyright notice, this list of     #
-- #    conditions and the following disclaimer in the documentation and/or other materials        #
-- #    provided with the distribution.                                                            #
-- #                                                                                               #
-- # 3. Neither the name of the copyright holder nor the names of its contributors may be used to  #
-- #    endorse or promote products derived from this software without specific prior written      #
-- #    permission.                                                                                #
-- #                                                                                               #
-- # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS   #
-- # OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF               #
-- # MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE    #
-- # COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,     #
-- # EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE #
-- # GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED    #
-- # AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING     #
-- # NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED  #
-- # OF THE POSSIBILITY OF SUCH DAMAGE.                                                            #
-- # ********************************************************************************************* #
-- # The NEORV32 Processor - https://github.com/stnolting/neorv32              (c) Stephan Nolting #
-- #################################################################################################

-- Created by Hipolito Guzman-Miranda based on Unai Martinez-Corral's adaptation to the iCESugar board

-- Allow use of std_logic, signed, unsigned
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

-- Library where the processor is located
library neorv32;

entity neorv32_iCEBreaker_BoardTop_MinimalBoot is
  -- Top-level ports. Board pins are defined in setups/osflow/constraints/iCEBreaker.pcf
  port (
    -- 12MHz Clock input
    iCEBreakerv10_CLK                : in std_logic;
    -- UART0
    iCEBreakerv10_RX                 : in  std_logic;
    iCEBreakerv10_TX                 : out std_logic;
    -- Button inputs
    iCEBreakerv10_BTN_N              : in std_logic;
    iCEBreakerv10_PMOD2_9_Button_1   : in std_logic;
    iCEBreakerv10_PMOD2_10_Button_3  : in std_logic;
    -- LED outputs
    iCEBreakerv10_LED_R_N            : out std_logic;
    iCEBreakerv10_LED_G_N            : out std_logic;
    -- PMOD1B
    iCEBreakerv10_PMOD1B_1           : out std_logic;
    iCEBreakerv10_PMOD1B_2           : out std_logic;
    --iCEBreakerv10_PMOD1B_3           : out std_logic;
    -- PMOD1A
    iCEBreakerv10_PMOD1A_1           : in std_logic
  );
end entity;

architecture neorv32_iCEBreaker_BoardTop_MinimalBoot_rtl of neorv32_iCEBreaker_BoardTop_MinimalBoot is

  -- -------------------------------------------------------------------------------------------
  -- Constants for microprocessor configuration
  -- -------------------------------------------------------------------------------------------

  -- General config --
  constant CLOCK_FREQUENCY              : natural := 12_000_000;  -- Microprocessor clock frequency (Hz)
  constant INT_BOOTLOADER_EN            : boolean := true;        -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
  constant HW_THREAD_ID                 : natural := 0;           -- hardware thread id (32-bit)

  -- RISC-V CPU Extensions --
  constant CPU_EXTENSION_RISCV_A        : boolean := true;        -- implement atomic extension?
  constant CPU_EXTENSION_RISCV_C        : boolean := true;        -- implement compressed extension?
  constant CPU_EXTENSION_RISCV_E        : boolean := false;       -- implement embedded RF extension?
  constant CPU_EXTENSION_RISCV_M        : boolean := true;        -- implement mul/div extension?
  constant CPU_EXTENSION_RISCV_U        : boolean := false;       -- implement user mode extension?
  constant CPU_EXTENSION_RISCV_Zfinx    : boolean := false;       -- implement 32-bit floating-point extension (using INT regs!)
  constant CPU_EXTENSION_RISCV_Zicsr    : boolean := true;        -- implement CSR system?
  constant CPU_EXTENSION_RISCV_Zifencei : boolean := false;       -- implement instruction stream sync.?

  -- Extension Options --
  constant FAST_MUL_EN                  : boolean := false;       -- use DSPs for M extension's multiplier
  constant FAST_SHIFT_EN                : boolean := false;       -- use barrel shifter for shift operations
  constant CPU_CNT_WIDTH                : natural := 34;          -- total width of CPU cycle and instret counters (0..64)

  -- Physical Memory Protection (PMP) --
  constant PMP_NUM_REGIONS              : natural := 0;            -- number of regions (0..64)
  constant PMP_MIN_GRANULARITY          : natural := 64*1024;      -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes

  -- Hardware Performance Monitors (HPM) --
  constant HPM_NUM_CNTS                 : natural := 0;            -- number of implemented HPM counters (0..29)
  constant HPM_CNT_WIDTH                : natural := 40;           -- total size of HPM counters (0..64)

  -- Internal Instruction memory --
  constant MEM_INT_IMEM_EN              : boolean := true;         -- implement processor-internal instruction memory
  constant MEM_INT_IMEM_SIZE            : natural := 64*1024;      -- size of processor-internal instruction memory in bytes

  -- Internal Data memory --
  constant MEM_INT_DMEM_EN              : boolean := true;         -- implement processor-internal data memory
  constant MEM_INT_DMEM_SIZE            : natural := 8*1024;      -- size of processor-internal data memory in bytes

  -- Internal Cache memory --
  constant ICACHE_EN                    : boolean := false;       -- implement instruction cache
  constant ICACHE_NUM_BLOCKS            : natural := 4;           -- i-cache: number of blocks (min 1), has to be a power of 2
  constant ICACHE_BLOCK_SIZE            : natural := 64;          -- i-cache: block size in bytes (min 4), has to be a power of 2
  constant ICACHE_ASSOCIATIVITY         : natural := 1;           -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2

  -- Processor peripherals --
  constant IO_GPIO_EN                   : boolean := true;        -- implement general purpose input/output port unit (GPIO)?
  constant IO_MTIME_EN                  : boolean := true;        -- implement machine system timer (MTIME)?
  constant IO_UART0_EN                  : boolean := true;        -- implement primary universal asynchronous receiver/transmitter (UART0)?
  constant IO_PWM_NUM_CH                : natural := 3;           -- number of PWM channels to implement (0..60); 0 = disabled
  constant IO_WDT_EN                    : boolean := true;        -- implement watch dog timer (WDT)?

  -- -------------------------------------------------------------------------------------------
  -- Signals for internal IO connections
  -- -------------------------------------------------------------------------------------------
  signal gpio_o : std_ulogic_vector(63 downto 0);
  signal gpio_i : std_ulogic_vector(63 downto 0);

  -- -------------------------------------------------------------------------------------------
  -- Signals for internal IO connections
  -- -------------------------------------------------------------------------------------------
  signal wb_cyc_m2s : std_ulogic;
  signal wb_stb_m2s : std_ulogic_vector(1 downto 0);        -- One per slave
  signal wb_adr_m2s : std_ulogic_vector(31 downto 0);
  signal wb_we_m2s  : std_ulogic;                           
  signal wb_dat_m2s : std_ulogic_vector(31 downto 0);       -- Data from the master to the slaves
  signal wb_dat_s2m : std_ulogic_vector(63 downto 0);       -- Data from the slaves (2 slaves) to the master
  signal wb_sel_m2s : std_ulogic_vector(3 downto 0);        -- Byte enable (array de vectores)
  signal wb_ack_s2m : std_ulogic_vector(1 downto 0);        -- One ack per slave
  signal wb_err_s2m : std_ulogic_vector(1 downto 0);        -- One err per slave
  signal wb_stall_s : std_ulogic_vector(1 downto 0) := (others => '0');
  signal wb_rty_s   : std_logic_vector(1 downto 0) := (others => '0');

  -- pwm signal
  signal pwm_o : std_ulogic;

  -- opb data signal
  signal data_i : std_ulogic;

  -- -------------------------------------------------------------------------------------------
  -- Signals for the Master Interface in the interconnect
  -- -------------------------------------------------------------------------------------------
  signal wb_cyc_m    : std_ulogic_vector(1 downto 0)  := (others => '0');
  signal wb_lock_m   : std_ulogic_vector(1 downto 0)  := (others => '0');
  signal wb_stb_m    : std_ulogic_vector(1 downto 0)  := (others => '0');
  signal wb_adr_m    : std_ulogic_vector(63 downto 0) := (others => '0'); 
  signal wb_we_m     : std_ulogic_vector(1 downto 0)  := (others => '0');
  signal wb_dat_i_m  : std_ulogic_vector(63 downto 0) := (others => '0'); 


  signal wb_dat_o_m  : std_ulogic_vector(31 downto 0) := (others => '0');
  
  
  signal wb_sel_m    : std_ulogic_vector(07 downto 0) := (others => '0'); 
  signal wb_ack_m    : std_ulogic;
  signal wb_err_m    : std_ulogic; 


  -- DEBUG!!-------------------
  signal DEBUG_LED_OUT_SIGNAL: std_logic;
  -- DEBUG!!-------------------

  -- DEBUG!!------------------
  constant DEBUG_ADDR : std_logic_vector(31 downto 0) := x"90000010";
  -- DEBUG!!------------------

begin

  -- -------------------------------------------------------------------------------------------
  -- Instance the PWM
  -- -------------------------------------------------------------------------------------------
  pwm_inst: entity neorv32.wb_pwm
  generic map(
    WB_ADDR_BASE => x"90000000",
    WB_ADDR_SIZE => 8)
  port map(

    -- wishbone host interface --
    wb_clk_i  => std_ulogic(iCEBreakerv10_CLK),
    wb_rstn_i => std_ulogic(iCEBreakerv10_BTN_N),
    wb_adr_i => wb_adr_m2s,
    wb_dat_i => wb_dat_m2s,
    wb_dat_o => wb_dat_s2m(31 downto 0),
    wb_we_i => wb_we_m2s,
    wb_sel_i => wb_sel_m2s,
    wb_stb_i => wb_stb_m2s(0),
    wb_cyc_i => wb_cyc_m2s,
    wb_ack_o => wb_ack_s2m(0),
    wb_err_o => wb_err_s2m(0),

    -- PWM sensor interface --
    pwm_o => pwm_o
  );
  -- -------------------------------------------------------------------------------------------
  -- Instance the OPB704
  -- -------------------------------------------------------------------------------------------
  opb_inst: entity neorv32.wb_IR_sensor
  generic map(
    WB_ADDR_BASE => x"90000010",    -- Every 16, another peripheral 
    WB_ADDR_SIZE => 4)
  port map(

    -- wishbone host interface --
    wb_clk_i  => std_ulogic(iCEBreakerv10_CLK),
    wb_rstn_i => std_ulogic(iCEBreakerv10_BTN_N),
    wb_adr_i => wb_adr_m2s,
    wb_dat_i => wb_dat_m2s,
    wb_dat_o => wb_dat_s2m(63 downto 32),
    wb_we_i => wb_we_m2s,
    wb_sel_i => wb_sel_m2s,
    wb_stb_i => wb_stb_m2s(1),
    wb_cyc_i => wb_cyc_m2s,
    wb_ack_o => wb_ack_s2m(1),
    wb_err_o => wb_err_s2m(1),

    -- OPB704 sensor interface --
    data_i => data_i
  );
  -- -------------------------------------------------------------------------------------------
  -- Instance the interconnect
  -- -------------------------------------------------------------------------------------------
  interconnect_inst: entity neorv32.wb_switch
    generic map(
         
         dat_sz => 32,    -- Width of data bus
         nib_sz => 4,     -- Nibble size
         addr_sz => 32,   -- Width of address bus   
         mstr_bits => 1,  -- Number of Wishbone Masters (in powers of 2)
         slv_bits => 1    -- Number of Wishbone Slaves (in powers of 2)
    )
    port map(

         clk_i  => std_ulogic(iCEBreakerv10_CLK),
         rst_i => std_ulogic(iCEBreakerv10_BTN_N),

         -- Whishbone Master Interface
         wb_mstr_cyc_i    => wb_cyc_m,
         wb_mstr_lock_i   => wb_lock_m,
         wb_mstr_stb_i    => wb_stb_m,
         wb_mstr_adr_i    => wb_adr_m,
         wb_mstr_we_i     => wb_we_m,
         wb_mstr_dat_i    => wb_dat_i_m,
         wb_mstr_dat_o    => wb_dat_o_m,
         wb_mstr_sel_i    => wb_sel_m,
         wb_mstr_ack_o    => wb_ack_m,
         wb_mstr_err_o    => wb_err_m,
         wb_mstr_stall_o  => open,
         wb_mstr_rty_o    => open,

        -- DEBUG!!-------------------
        DEBUG_LED_OUT => DEBUG_LED_OUT_SIGNAL,
        -- DEBUG!!-------------------

         -- Slaves Interface
         wb_slv_cyc_o => wb_cyc_m2s,
         wb_slv_stb_o => wb_stb_m2s,
         wb_slv_adr_o => wb_adr_m2s,
         wb_slv_we_o  => wb_we_m2s,
         wb_slv_dat_o => wb_dat_m2s,
         wb_slv_sel_o => wb_sel_m2s,
         wb_slv_dat_i => wb_dat_s2m,
         wb_slv_ack_i => wb_ack_s2m,
         wb_slv_err_i => wb_err_s2m,
         wb_slv_lock_o => open,
         wb_slv_stall_i => wb_stall_s,
         wb_slv_rty_i => wb_rty_s
    );

  -- -------------------------------------------------------------------------------------------
  -- Instance the microprocessor
  -- -------------------------------------------------------------------------------------------

  neorv32_inst: entity neorv32.neorv32_top
  generic map (
    -- General --
    CLOCK_FREQUENCY              => CLOCK_FREQUENCY,               -- clock frequency of clk_i in Hz
    INT_BOOTLOADER_EN            => INT_BOOTLOADER_EN,             -- boot configuration: true = boot explicit bootloader; false = boot from int/ext (I)MEM
    HW_THREAD_ID                 => HW_THREAD_ID,                  -- hardware thread id (32-bit)

    -- On-Chip Debugger (OCD) --
    ON_CHIP_DEBUGGER_EN          => false,                         -- implement on-chip debugger?

    -- RISC-V CPU Extensions --
    CPU_EXTENSION_RISCV_A        => CPU_EXTENSION_RISCV_A,         -- implement atomic extension?
    CPU_EXTENSION_RISCV_C        => CPU_EXTENSION_RISCV_C,         -- implement compressed extension?
    CPU_EXTENSION_RISCV_E        => CPU_EXTENSION_RISCV_E,         -- implement embedded RF extension?
    CPU_EXTENSION_RISCV_M        => CPU_EXTENSION_RISCV_M,         -- implement mul/div extension?
    CPU_EXTENSION_RISCV_U        => CPU_EXTENSION_RISCV_U,         -- implement user mode extension?
    CPU_EXTENSION_RISCV_Zfinx    => CPU_EXTENSION_RISCV_Zfinx,     -- implement 32-bit floating-point extension (using INT regs!)
    CPU_EXTENSION_RISCV_Zicsr    => CPU_EXTENSION_RISCV_Zicsr,     -- implement CSR system?
    CPU_EXTENSION_RISCV_Zicntr   => true,                          -- implement base counters?
    CPU_EXTENSION_RISCV_Zifencei => CPU_EXTENSION_RISCV_Zifencei,  -- implement instruction stream sync.?

    -- Extension Options --
    FAST_MUL_EN                  => FAST_MUL_EN,                   -- use DSPs for M extension's multiplier
    FAST_SHIFT_EN                => FAST_SHIFT_EN,                 -- use barrel shifter for shift operations
    CPU_CNT_WIDTH                => CPU_CNT_WIDTH,                 -- total width of CPU cycle and instret counters (0..64)

    -- Physakerv10_CLK),ical Memory Protection (PMP) --
    PMP_NUM_REGIONS              => PMP_NUM_REGIONS,       -- number of regions (0..64)
    PMP_MIN_GRANULARITY          => PMP_MIN_GRANULARITY,   -- minimal region granularity in bytes, has to be a power of 2, min 8 bytes
 
    -- Hardware Performance Monitors (HPM) --
    HPM_NUM_CNTS                 => HPM_NUM_CNTS,          -- number of implemented HPM counters (0..29)
    HPM_CNT_WIDTH                => HPM_CNT_WIDTH,         -- total size of HPM counters (1..64)

    -- Internal Instruction memory --
    MEM_INT_IMEM_EN              => MEM_INT_IMEM_EN,       -- implement processor-internal instruction memory
    MEM_INT_IMEM_SIZE            => MEM_INT_IMEM_SIZE,     -- size of processor-internal instruction memory in bytes

    -- Internal Data memory --
    MEM_INT_DMEM_EN              => MEM_INT_DMEM_EN,       -- implement processor-internal data memory
    MEM_INT_DMEM_SIZE            => MEM_INT_DMEM_SIZE,     -- size of processor-internal data memory in bytes

    -- Internal Cache memory --
    ICACHE_EN                    => ICACHE_EN,             -- implement instruction cache
    ICACHE_NUM_BLOCKS            => ICACHE_NUM_BLOCKS,     -- i-cache: number of blocks (min 1), has to be a power of 2
    ICACHE_BLOCK_SIZE            => ICACHE_BLOCK_SIZE,     -- i-cache: block size in bytes (min 4), has to be a power of 2
    ICACHE_ASSOCIATIVITY         => ICACHE_ASSOCIATIVITY,  -- i-cache: associativity / number of sets (1=direct_mapped), has to be a power of 2

    -- External memory interface --
    MEM_EXT_EN                   => true,                  -- implement external memory bus interface?
    MEM_EXT_TIMEOUT              => 255,                   -- cycles after a pending bus access auto-terminates (0 = disabled)

    -- Processor peripherals --
    IO_GPIO_EN                   => IO_GPIO_EN,    -- implement general purpose input/output port unit (GPIO)?
    IO_MTIME_EN                  => IO_MTIME_EN,   -- implement machine system timer (MTIME)?
    IO_UART0_EN                  => IO_UART0_EN,   -- implement primary universal asynchronous receiver/transmitter (UART0)?
    IO_UART1_EN                  => false,         -- implement secondary universal asynchronous receiver/transmitter (UART1)?
    IO_SPI_EN                    => false,         -- implement serial peripheral interface (SPI)?
    IO_TWI_EN                    => false,         -- implement two-wire interface (TWI)?
    IO_PWM_NUM_CH                => IO_PWM_NUM_CH, -- number of PWM channels to implement (0..60); 0 = disabled
    IO_WDT_EN                    => IO_WDT_EN,     -- implement watch dog timer (WDT)?
    IO_TRNG_EN                   => false,         -- implement true random number generator (TRNG)?
    IO_CFS_EN                    => false,         -- implement custom functions subsystem (CFS)?
    IO_CFS_CONFIG                => x"00000000",   -- custom CFS configuration generic
    IO_CFS_IN_SIZE               => 32,            -- size of CFS input conduit in bits
    IO_CFS_OUT_SIZE              => 32,            -- size of CFS output conduit in bits
    IO_NEOLED_EN                 => false          -- implement NeoPixel-compatible smart LED interface (NEOLED)?
  )
  port map(
    -- Global control --
    clk_i       => std_ulogic(iCEBreakerv10_CLK),   -- global clock, rising edge
    rstn_i      => std_ulogic(iCEBreakerv10_BTN_N), -- global reset, low-active, async

    -- JTAG on-chip debugger interface (available if ON_CHIP_DEBUGGER_EN = true) --
    jtag_trst_i => '0',                             -- low-active TAP reset (optional)
    jtag_tck_i  => '0',                             -- serial clock
    jtag_tdi_i  => '0',                             -- serial data input
    jtag_tdo_o  => open,                            -- serial data output
    jtag_tms_i  => '0',                             -- mode select

    -- Wishbone bus interface (available if MEM_EXT_EN = true) --
    wb_tag_o    => open,                            -- request tag    (SHOULDN'T BE NEEDED, OPTIONAL)

    wb_adr_o    => wb_adr_m(31 downto 0),           -- address

    wb_dat_i    => wb_dat_i_m(31 downto 0),         -- read data
    wb_dat_o    => wb_dat_o_m(31 downto 0),         -- write data
    wb_we_o     => wb_we_m(0),                      -- read/write
    wb_sel_o    => wb_sel_m(3 downto 0),            -- byte enable
    wb_stb_o    => wb_stb_m(0),                     -- strobe
    wb_cyc_o    => wb_cyc_m(0),                     -- valid cycle
    wb_lock_o   => wb_lock_m(0),                    -- exclusive access request
    wb_ack_i    => wb_ack_m,                        -- transfer acknowledge
    wb_err_i    => wb_err_m,                        -- transfer error

    -- Advanced memory control signals (available if MEM_EXT_EN = true) --
    fence_o     => open,                            -- indicates an executed FENCE operation
    fencei_o    => open,                            -- indicates an executed FENCEI operation

    -- GPIO (available if IO_GPIO_EN = true) --
    gpio_o      => gpio_o,                          -- parallel output
    gpio_i      => gpio_i,                          -- parallel input

    -- primary UART0 (available if IO_UART0_EN = true) --
    uart0_txd_o => iCEBreakerv10_TX,                -- UART0 send data
    uart0_rxd_i => iCEBreakerv10_RX,                -- UART0 receive data
    uart0_rts_o => open,                            -- hw flow control: UART0.RX ready to receive ("RTR"), low-active, optional
    uart0_cts_i => '0',                             -- hw flow control: UART0.TX allowed to transmit, low-active, optional

    -- secondary UART1 (available if IO_UART1_EN = true) --
    uart1_txd_o => open,                         -- UART1 send data
    uart1_rxd_i => '0',                          -- UART1 receive data
    uart1_rts_o => open,                         -- hw flow control: UART1.RX ready to receive ("RTR"), low-active, optional
    uart1_cts_i => '0',                          -- hw flow control: UART1.TX allowed to transmit, low-active, optional

    -- SPI (available if IO_SPI_EN = true) --
    spi_sck_o   => open,                         -- SPI serial clock
    spi_sdo_o   => open,                         -- controller data out, peripheral data in
    spi_sdi_i   => '0',                          -- controller data in, peripheral data out
    spi_csn_o   => open,                         -- SPI CS

    -- TWI (available if IO_TWI_EN = true) --
    twi_sda_io  => open,                         -- twi serial data line
    twi_scl_io  => open,                         -- twi serial clock line

    -- PWM (available if IO_PWM_NUM_CH > 0) --
    pwm_o       => open,                         -- pwm channels

    -- Custom Functions Subsystem IO --
    cfs_in_i    => (others => '0'),              -- custom CFS inputs conduit
    cfs_out_o   => open,                         -- custom CFS outputs conduit

    -- NeoPixel-compatiblepwm_servo => '0'),     -- current system time from ext. MTIME (if IO_MTIME_EN = false)
    mtime_o     => open,                         -- current system time from int. MTIME (if IO_MTIME_EN = true)

    -- Interrupts --
    mtime_irq_i => '0',                          -- machine timer interrupt, available if IO_MTIME_EN = false
    msw_irq_i   => '0',                          -- machine software interrupt
    mext_irq_i  => '0'                           -- machine external interrupt
  );

  --DEBUG : process
  --begin
      --wait until rising_edge(iCEBreakerv10_CLK);

      -- DEBUG!!------------------
      --if (wb_adr_m(31 downto 0) = DEBUG_ADDR) then
      --    iCEBreakerv10_PMOD1B_2 <= '1';
      --else
      --    iCEBreakerv10_PMOD1B_2 <= '0';
      --end if;
      -- DEBUG!!-------------------

  --end process;

  -- -------------------------------------------------------------------------------------------
  -- IO Connections
  -- -------------------------------------------------------------------------------------------

  iCEBreakerv10_PMOD1B_1 <= pwm_o;
  iCEBreakerv10_PMOD1B_2 <= DEBUG_LED_OUT_SIGNAL;
  --iCEBreakerv10_PMOD1B_3 <= wb_cyc_m(0);
  data_i <= iCEBreakerv10_PMOD1A_1;
  
end architecture;
