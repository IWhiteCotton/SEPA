library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library neorv32;
use neorv32.neorv32_package.all;

entity wb_IR_sensor is
  generic (
    WB_ADDR_BASE : std_ulogic_vector(31 downto 0); -- module base address, size-aligned
    WB_ADDR_SIZE : positive                        -- module address space in bytes, has to be a power of two, min 4
  );
  port (
    
    -- wishbone host interface --
    wb_clk_i  : in  std_ulogic;                     -- clock
    wb_rstn_i : in  std_ulogic;                     -- reset, async, low-active
    wb_adr_i  : in  std_ulogic_vector(31 downto 0); -- address
    wb_dat_i  : in  std_ulogic_vector(31 downto 0); -- read data
    wb_dat_o  : out std_ulogic_vector(31 downto 0); -- write data
    wb_we_i   : in  std_ulogic;                     -- read/write
    wb_sel_i  : in  std_ulogic_vector(03 downto 0); -- byte enable
    wb_stb_i  : in  std_ulogic;                     -- strobe
    wb_cyc_i  : in  std_ulogic;                     -- valid cycle
    wb_ack_o  : out std_ulogic;                     -- transfer acknowledge
    wb_err_o  : out std_ulogic;                     -- transfer error
    
    -- OPB704 sensor interface --
    data_i : in  std_ulogic                         -- Sensor data 
  );
end wb_IR_sensor;

architecture wb_IR_sensor_rtl of wb_IR_sensor is

  -- internal constants --
  constant addr_mask_c : std_ulogic_vector(31 downto 0) := std_ulogic_vector(to_unsigned(WB_ADDR_SIZE-1, 32));
  constant all_zero_c  : std_ulogic_vector(31 downto 0) := (others => '0');

  -- address match --
  signal access_req : std_ulogic;

   -- The reading of the sensor will be stored here --
  signal sensor_data   : std_ulogic_vector(31 downto 0) := (others => '0');   

begin

  -- SANITY CHECKS --
  assert not (WB_ADDR_SIZE < 4) report "wb_regs config ERROR: Address space <WB_ADDR_SIZE> has to be at least 4 bytes." severity error;
  assert not (is_power_of_two_f(WB_ADDR_SIZE) = false) report "wb_regs config ERROR: Address space <WB_ADDR_SIZE> has to be a power of two." severity error;
  assert not ((WB_ADDR_BASE and addr_mask_c) /= all_zero_c) report "wb_regs config ERROR: Module base address <WB_ADDR_BASE> has to be aligned to its address space <WB_ADDR_SIZE>." severity error;

  -- CHECK DEVICE ACCESS --
  access_req <= '1' when ((wb_adr_i and (not addr_mask_c)) = (WB_ADDR_BASE and (not addr_mask_c))) else '0';

  -- COMBINATIONAL PROCESS --
  comb: process(wb_cyc_i, wb_stb_i, wb_sel_i, access_req, wb_we_i, sensor_data)
  begin

    wb_dat_o <= sensor_data;      -- Output connected to sensor_data register by default
    wb_ack_o <= '0';              -- Default ack is inactive

    -- Is the peripheral selected?
    if (wb_cyc_i = '1') and (wb_stb_i = '1') and (access_req = '1') then

      -- Write access, in this design I only acknowledge it
      if (wb_we_i = '1') and (wb_sel_i = "1111") then 
        wb_ack_o <= '1';
      else
      -- Read access
        case to_integer(unsigned(wb_adr_i(index_size_f(WB_ADDR_SIZE)-1 downto 2))) is
          when 0 =>
            wb_dat_o <= sensor_data;
          when others =>
            null;
        end case;
        wb_ack_o <= '1';
      end if;

    end if;

  end process;

  -- SYNCHRONOUS PROCESS --
  sync: process(wb_rstn_i, wb_clk_i)
  begin
    if wb_rstn_i = '0' then
      sensor_data <= (others => '0');                          
    elsif rising_edge(wb_clk_i) then                                      
      sensor_data(0) <= data_i;      -- Get sensor data
    end if;
  end process;

  -- ERRORS --
  wb_err_o <= '0';

end wb_IR_sensor_rtl;