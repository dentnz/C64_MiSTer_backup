library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.std_logic_unsigned.ALL;
use IEEE.numeric_std.all;

entity reu is
port (
    clock               : in  std_logic;
    reset               : in  std_logic;

    g_ram_tag           : std_logic_vector(7 downto 0) := X"10";
    g_extended          : boolean := false;
    g_ram_base          : unsigned(27 downto 0) := X"0000000"; -- second (=upper 16M)

    -- register interface
    -- Signals are expanded out, as we are integrating with c64.sv
    slot_req_data       : in  std_logic_vector(7 downto 0);
    slot_req_io_address : in  unsigned(15 downto 0);
    slot_req_bus_address: in  unsigned(15 downto 0);
    slot_req_io_read    : in  std_logic;
    slot_req_io_write   : in  std_logic;

    --slot_resp       : out t_slot_resp;
    slot_resp_nmi       : out std_logic;
    slot_resp_irq       : out std_logic;
    slot_resp_data      : out std_logic_vector(7 downto 0);
    slot_resp_reg_output: out std_logic;

    -- Active low
    write_ff00          : in  std_logic;

    -- system interface
    phi2_tick           : in  std_logic := '0';
    reu_dma_n           : out std_logic := '1';
    size_ctrl           : in  std_logic_vector(2 downto 0) := "111";
    enable              : in  std_logic;
    inhibit             : in  std_logic := '0'; -- to pause a transfer, because OSD menu is active
    ba                  : in  std_logic;
	cpu_we              : in  std_logic;

    -- memory interface
    -- Memory request details going out
    mem_req_request     : out std_logic := '0';
    mem_req_address     : out unsigned(25 downto 0);
    mem_req_read_writen : out std_logic := '1';
    mem_req_data        : out std_logic_vector(7 downto 0);
    mem_req_tag         : out std_logic_vector(7 downto 0);

    -- Memory response to request
    mem_resp_rack_tag   : in  std_logic_vector(7 downto 0);
    mem_resp_dack_tag   : in  std_logic_vector(7 downto 0);
    mem_resp_data       : in  std_logic_vector(7 downto 0);

    -- DMA request (requests to c64-based sdram)
    dma_req_request     : out std_logic := '0';
    dma_req_address     : out unsigned(15 downto 0);
    dma_req_read_writen : out std_logic := '1';
    dma_req_data        : out std_logic_vector(7 downto 0);

    -- DMA response from the c64
    dma_resp_rack       : in  std_logic;
    dma_resp_dack       : in  std_logic;
    dma_resp_data       : in  std_logic_vector(7 downto 0)
);

end reu;

-- The REU is actually really simple.
-- There are 4 modes of operation, and 4 active states:
-- r = reu read, w = reu write, R = c64 read, W = c64 write
-- Copy from c64 to reu (00): Rw
-- Copy from reu to c64 (01): rW
-- Swap                 (10): rRwW (RrWw or rRWw or RrwW or rRwW)
-- Verify               (11): rR   (Rr or rR)
-- The smallest implementation is very likely when only one bit is needed to select between
-- transition options, and reducing the total number of transitions makes sense too.

architecture gideon of reu is

    type t_state is (idle, do_read_c64, use_data, do_write_c64, do_read_reu, do_write_reu, reu_write_done, check_end, delay, phi2GoingHigh, waitForCPUR);
    signal state     : t_state;

    attribute fsm_encoding : string;
    attribute fsm_encoding of state : signal is "one-hot";

    signal io_wdatau  : unsigned(7 downto 0);
    signal io_rdata   : std_logic_vector(7 downto 0);
    signal io_write   : std_logic;
    signal io_read    : std_logic;

    signal c64_base   : unsigned(15 downto 0) := (others => '0');
    signal reu_base   : unsigned(23 downto 0) := (others => '0');
    signal length_reg : unsigned(23 downto 0) := (others => '1');

    signal c64_addr   : unsigned(15 downto 0) := (others => '0');
    signal reu_addr   : unsigned(23 downto 0) := (others => '0');
    signal count      : unsigned(23 downto 0) := (others => '1');
    
    signal c64_req    : std_logic := '0';
    signal c64_rack   : std_logic := '0';
    signal c64_dack   : std_logic := '0';
    signal reu_req    : std_logic := '0';
    signal reu_rack   : std_logic := '0';
    signal reu_dack   : std_logic := '0';

    signal glob_rwn   : std_logic := '1';

    signal masked_reu_addr : unsigned(23 downto 0);
    signal mask            : unsigned(7 downto 0);

    signal c64_read_reg : std_logic_vector(7 downto 0) := (others => '0');
    signal reu_read_reg : std_logic_vector(7 downto 0) := (others => '0');

    signal verify_error : std_logic;
    signal trans_done   : std_logic;
    signal irq_pend     : std_logic;
    signal reserved     : std_logic_vector(2 downto 0);

    type t_control is record
        irq_en      : std_logic;
        irq_done    : std_logic;
        irq_error   : std_logic;
        fix_reu     : std_logic;
        fix_c64     : std_logic;
    end record;

    type t_command is record
        execute     : std_logic;
        autoload    : std_logic;
        ff00        : std_logic;
        mode        : std_logic_vector(1 downto 0);
    end record;

    constant c_control_def : t_control := (others => '0');
    constant c_command_def : t_command := (
        mode     => "00",
        execute  => '0',
        ff00     => '1',
        autoload => '0' );

    signal control     : t_control;
    signal command     : t_command;

    -- signals for extended mode
    signal rate_div     : unsigned(7 downto 0) := (others => '0');
    signal start_delay  : unsigned(7 downto 0) := (others => '0');
    signal ext_count    : unsigned(7 downto 0) := (others => '0');

    -- These are from reu_pkg
    constant c_status     : unsigned(7 downto 0) := X"00";
    constant c_command    : unsigned(7 downto 0) := X"01";
    constant c_c64base_l  : unsigned(7 downto 0) := X"02";
    constant c_c64base_h  : unsigned(7 downto 0) := X"03";
    constant c_reubase_l  : unsigned(7 downto 0) := X"04";
    constant c_reubase_m  : unsigned(7 downto 0) := X"05";
    constant c_reubase_h  : unsigned(7 downto 0) := X"06";
    constant c_translen_l : unsigned(7 downto 0) := X"07";
    constant c_translen_h : unsigned(7 downto 0) := X"08";
    constant c_irqmask    : unsigned(7 downto 0) := X"09";
    constant c_control    : unsigned(7 downto 0) := X"0A";
    -- extended registers
    constant c_size_read  : unsigned(7 downto 0) := X"0C";
    constant c_start_delay: unsigned(7 downto 0) := X"0D";
    constant c_rate_div   : unsigned(7 downto 0) := X"0E";
    constant c_translen_x : unsigned(7 downto 0) := X"0F";

    constant c_mode_toreu   : std_logic_vector(1 downto 0) := "00";
    constant c_mode_toc64   : std_logic_vector(1 downto 0) := "01";
    constant c_mode_swap    : std_logic_vector(1 downto 0) := "10";
    constant c_mode_verify  : std_logic_vector(1 downto 0) := "11";

	signal cpu_we_s         : std_logic;
	signal cpu_read_delay   : std_logic_vector(1 downto 0) := "00";
	signal ba_s				: std_logic;
	signal phi2_tick_s      : std_logic;
begin
    with size_ctrl select mask <=
        "00000001" when "000",
        "00000011" when "001",
        "00000111" when "010",
        "00001111" when "011",
        "00011111" when "100",
        "00111111" when "101",
        "01111111" when "110",
        "11111111" when others;

    --masked_reu_addr(23 downto 19) <= (reu_base(23 downto 19) and mask(7 downto 3)) when not g_extended else
    --                                 (reu_addr(23 downto 19) and mask(7 downto 3));
									 
	masked_reu_addr(23 downto 19) <= (reu_base(23 downto 19) and mask(7 downto 3));
									 
    masked_reu_addr(18 downto 16) <= (reu_addr(18 downto 16) and mask(2 downto 0));
    masked_reu_addr(15 downto  0) <= reu_addr(15 downto 0);

    reu_rack <= '1' when mem_resp_rack_tag = g_ram_tag else '0';
    reu_dack <= '1' when mem_resp_dack_tag = g_ram_tag else '0';

    io_wdatau <= unsigned(slot_req_data);

    -- fill mem request structure
    -- Expanded out as we need to easily pass signals directly out to system verilog code of c64.sv
    mem_req_tag         <= g_ram_tag;
    mem_req_request     <= reu_req;
    mem_req_address     <= g_ram_base(25 downto 24) & masked_reu_addr(23 downto 0);
    mem_req_read_writen <= glob_rwn;
    mem_req_data        <= c64_read_reg;

    -- fill dma request structure
    dma_req_request     <= c64_req;
    dma_req_address     <= c64_addr;
    dma_req_read_writen <= glob_rwn;
    dma_req_data        <= reu_read_reg;
    c64_rack            <= dma_resp_rack;
    c64_dack            <= dma_resp_dack;

    p_main: process(clock)
        procedure next_address is
        begin
            if control.fix_c64='0' then
                c64_addr <= c64_addr + 1;
            end if;
            if control.fix_reu='0' then
                reu_addr <= reu_addr + 1;
            end if;
            if count = 1 then
                trans_done <= '1';
            else
                count <= count - 1;
            end if;
        end procedure;

        procedure transfer_end is
        begin
            if command.autoload='1' then
                c64_addr <= c64_base;
                reu_addr <= reu_base(reu_addr'range);
				--reu_addr <= reu_base(23 downto 0);
                --if g_extended then
                --count <= length_reg(15 downto 0);
                --else
                count(15 downto 0) <= length_reg(15 downto 0);
                --end if;
            end if;
            command.ff00 <= '1'; -- reset to default state
			-- We need to wait for phi2 before closing off
			state <= phi2GoingHigh;
        end procedure;

        procedure dispatch is
        begin
            glob_rwn <= '1'; -- we're going to read.
            case command.mode is
                when c_mode_toreu =>  -- C64 to REU
                    c64_req <= '1';
                    state <= do_read_c64;
                when others => -- in all other cases, read reu first
                    reu_req <= '1';
                    state <= do_read_reu;
            end case;
        end procedure;

        variable r: unsigned(7 downto 0);
    begin
        if rising_edge(clock) then
			cpu_we_s <= cpu_we;
			ba_s <= ba;
			phi2_tick_s <= phi2_tick;
            r := "000" & slot_req_io_address(4 downto 0);
			
            if io_write='1' then  --$DF00-$DF1F, decoded below in a concurrent statement
                case r is
                when c_c64base_l  => c64_base(7 downto 0) <= io_wdatau;
                                     c64_addr <= c64_base(15 downto 8) & io_wdatau;  -- half autoload bug
                when c_c64base_h  => c64_base(15 downto 8) <= io_wdatau;
                                     c64_addr <= io_wdatau & c64_base(7 downto 0);   -- half autoload bug
                when c_reubase_l  => reu_base(7 downto 0) <= io_wdatau;
                                     reu_addr(15 downto 0) <= reu_base(15 downto 8) & io_wdatau;  -- half autoload bug
                when c_reubase_m  => reu_base(15 downto 8) <= io_wdatau;
                                     reu_addr(15 downto 0) <= io_wdatau & reu_base(7 downto 0);   -- half autoload bug
                when c_reubase_h  => reu_base(23 downto 16) <= io_wdatau;
                                     reu_addr(23 downto 16) <= io_wdatau;
                when c_translen_l => length_reg(7 downto 0) <= io_wdatau;
                                     count(15 downto 0) <= length_reg(15 downto 8) & io_wdatau;   -- half autoload bug
                when c_translen_h => length_reg(15 downto 8) <= io_wdatau;
                                     count(15 downto 0) <= io_wdatau & length_reg(7 downto 0);    -- half autoload bug

                when c_irqmask    =>
                    control.irq_en    <= io_wdatau(7);
                    control.irq_done  <= io_wdatau(6);
                    control.irq_error <= io_wdatau(5);

                when c_control    =>
                    control.fix_reu  <= io_wdatau(6);
                    control.fix_c64  <= io_wdatau(7);

                when c_command    =>
                    command.execute  <= io_wdatau(7);
                    reserved(2)      <= io_wdatau(6);
                    command.autoload <= io_wdatau(5);
                    command.ff00     <= io_wdatau(4);
                    reserved(1)      <= io_wdatau(3);
                    reserved(0)      <= io_wdatau(2);
                    command.mode     <= slot_req_data(1 downto 0);

                when others =>
                    null;
                end case;
            end if;

            -- extended registers
            --if io_write='1' then  --$DF00-$DF1F, decoded below in a concurrent statement
            --    case r is
            --    when c_start_delay =>
            --        start_delay <= io_wdatau;
            --    when c_rate_div    =>
            --        rate_div    <= io_wdatau;
            --    when c_translen_x =>
            --        --length_reg(23 downto 16) <= io_wdatau;
                    --count(23 downto 16) <= io_wdatau;
            --    when others =>
            --        null;
            --    end case;
            --end if;

            -- clear on read flags
            if io_read='1' then
                if r = c_status then
                    verify_error <= '0';
                    trans_done   <= '0';
                end if;
            end if;
			
            case state is
            when idle =>
                reu_dma_n <= '1';
                glob_rwn  <= '1';
                if command.execute='1' then
                    if (command.ff00='0' and write_ff00='1') or (command.ff00='1') then
                        verify_error <= '0';
                        trans_done   <= '0';
                        command.execute <= '0';
                        state <= delay;
                    end if;
                end if;

            when delay =>
				-- Wait for a phi2 tick to go by while the bus is available
                if phi2_tick_s='0' and phi2_tick='1' and ba='1' and ba_s='1' and cpu_we='0' then
					--state <= phi2GoingHigh;
					reu_dma_n <= '0';
					dispatch;
				end if;

            when phi2GoingHigh =>
				if phi2_tick_s='0' and phi2_tick='1' and ba='1' then
					state <= idle;
                end if;
				
			when waitForCPUR =>
				cpu_read_delay <= cpu_read_delay + 1;
				-- Wait until the CPU has finished it's write, 3 cycles after the phi tick
				if cpu_read_delay="01" then
					-- DMA signal is allowed now, engage
					reu_dma_n <= '0';
					cpu_read_delay <= "00";
					dispatch;
				end if;

            when do_read_reu =>
                if reu_rack='1' then
                    reu_req <= '0';
                end if;
                if reu_dack='1' then
                    reu_read_reg <= mem_resp_data;
                    state <= use_data;
                end if;

            when use_data =>
                if inhibit = '0' then
                    case command.mode is
                        when c_mode_swap | c_mode_verify =>
                            c64_req <= '1';
                            glob_rwn <= '1';
                            state <= do_read_c64;
                        when others =>
                            c64_req <= '1';
                            glob_rwn <= '0';
                            state <= do_write_c64;
                    end case;
                end if;

            when do_write_c64 =>
                if c64_rack='1' then
					c64_req <= '0';
					next_address;
                    state <= check_end;
                end if;

            when do_read_c64 =>
                if c64_rack='1' then
                    c64_req <= '0';
                end if;
                if c64_dack='1' then
                    c64_read_reg <= dma_resp_data;

                    case command.mode is
                        when c_mode_verify =>
                            if dma_resp_data /= reu_read_reg then
                                verify_error <= '1';
                                state <= idle;
                            else
                                next_address;
                                state <= check_end;
                            end if;
                        when others =>
                            reu_req <= '1';
                            glob_rwn <= '0';
                            state <= do_write_reu;
                    end case;
                end if;

            when do_write_reu =>
                if reu_rack='1' then
                	reu_req <= '0';
                    state <= reu_write_done;
                end if;

            when reu_write_done =>
				case command.mode is
                    when c_mode_swap =>
                        if inhibit = '0' then
                            c64_req <= '1';
                            glob_rwn <= '0';
                            state <= do_write_c64;
                        end if;
                    when others =>
                        next_address;
                        state <= check_end;
                end case;

            when check_end =>
				--ext_count <= rate_div;
                if trans_done='1' then
                    transfer_end;
                --elsif g_extended then
                --    state <= delay;
                elsif inhibit = '0' then
                    state <= delay;
                end if;

            when others =>
                null;
            end case;

            if reset='1' then
                reu_req      <= '0';
                c64_req      <= '0';
                glob_rwn     <= '1';
                control      <= c_control_def;
                command      <= c_command_def;
                state        <= idle;
                reserved     <= (others => '0');
                verify_error <= '0';
                trans_done   <= '0';
                reu_dma_n    <= '1';

                c64_base     <= (others => '0');
                reu_base     <= (others => '0');
                length_reg   <= X"00FFFF";
                c64_addr     <= (others => '0');
                reu_addr     <= (others => '0');
                count        <= (others => '1');
                start_delay  <= (others => '0');
                rate_div     <= (others => '0');
                ext_count    <= (others => '0');
				cpu_we_s     <= '0';
				cpu_read_delay <= "00";
            end if;
        end if;
    end process;

    p_read: process(slot_req_bus_address, slot_req_io_address, slot_req_io_read, slot_req_io_write, control, command, count, c64_addr, reu_addr, verify_error, trans_done, irq_pend,
                c64_base, reu_base, length_reg, reserved, mask, start_delay, rate_div)

        variable r: unsigned(7 downto 0);
    begin
        io_rdata <= X"FF";
        r := "000" & slot_req_bus_address(4 downto 0);
        case r is
        when c_status     =>
            io_rdata(7) <= irq_pend;
            io_rdata(6) <= trans_done;
            io_rdata(5) <= verify_error;
            io_rdata(4) <= mask(1); -- for 256k and larger, this is set to '1'.
            io_rdata(3 downto 0) <= X"0"; -- version

        when c_command    =>
            io_rdata(7) <= command.execute;
            io_rdata(6) <= reserved(2);
            io_rdata(5) <= command.autoload;
            io_rdata(4) <= command.ff00;
            io_rdata(3 downto 2) <= reserved(1 downto 0);
            io_rdata(1 downto 0) <= command.mode;

        when c_irqmask    =>
            io_rdata(7) <= control.irq_en;
            io_rdata(6) <= control.irq_done;
            io_rdata(5) <= control.irq_error;

        when c_control    =>
            io_rdata(7) <= control.fix_c64;
            io_rdata(6) <= control.fix_reu;

        when c_c64base_l  => io_rdata <= std_logic_vector(c64_addr(7 downto 0));
        when c_c64base_h  => io_rdata <= std_logic_vector(c64_addr(15 downto 8));
        when c_reubase_l  => io_rdata <= std_logic_vector(reu_addr(7 downto 0));
        when c_reubase_m  => io_rdata <= std_logic_vector(reu_addr(15 downto 8));
        when c_reubase_h  =>
--            if g_extended then
--                io_rdata <= std_logic_vector(reu_addr(23 downto 16));
--            else
                io_rdata <= "11111" & std_logic_vector(reu_addr(18 downto 16)); -- maximum 19 bits
--            end if;
        when c_translen_l => io_rdata <= std_logic_vector(count(7 downto 0));
        when c_translen_h => io_rdata <= std_logic_vector(count(15 downto 8));

        when c_size_read    => io_rdata <= std_logic_vector(mask);
        when c_start_delay  => io_rdata <= std_logic_vector(start_delay);
        when c_rate_div     => io_rdata <= std_logic_vector(rate_div);
        when others =>
            null;

        end case;
    end process;

    irq_pend <= control.irq_en and ((control.irq_done and trans_done) or (control.irq_error and verify_error));

    slot_resp_nmi        <= '0';
    slot_resp_irq        <= irq_pend;
    slot_resp_data       <= io_rdata(7 downto 0);
    slot_resp_reg_output <= enable when slot_req_bus_address(8 downto 5)=X"8" and
                                        slot_req_bus_address(4 downto 2)/="111" and
                                        (state = idle)
                                   else '0';
    io_write             <= (enable and slot_req_io_write) when slot_req_io_address(8 downto 5)=X"8" else '0';
    io_read              <= (enable and slot_req_io_read) when slot_req_io_address(8 downto 5)=X"8" else '0';
end gideon;