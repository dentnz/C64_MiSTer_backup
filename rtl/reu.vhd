-- CSG REU/REC compatible RAM expansion controller
-- initial version: 2.9.2001 by Rainer Buchty (rainer@buchty.net)
-- syntactically correct, but completely untested -- so use at your own risk
--
-- 16MB DRAM to 64kB C64 memory DMA unit

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;

package p_rec is
component rec is
	port(
		phi2, dotclk:	in std_logic;
		rst:		    in std_logic;

		dma:		out std_logic;
		ba:		    in std_logic;

		cpu_addr:	  in std_logic_vector(15 downto 0);
		cpu_addr_out: out std_logic_vector(15 downto 0);
		cpu_data:	  in std_logic_vector(7 downto 0);
		cpu_data_out: out std_logic_vector(7 downto 0);
		cpu_rw:       in std_logic;
		cpu_we:       in std_logic;
		cpu_rw_out:   out std_logic;
		IOF:          in std_logic;

		reu_addr:	out std_logic_vector(11 downto 0);
		reu_data:	inout std_logic_vector(7 downto 0);
		reu_rw:		out std_logic;
		reu_ras,
		reu_cas:	out std_logic;

		size:		in std_logic;

		aec:		inout std_logic;
        reu_debug:  out std_logic_vector(7 downto 0)
	);
end component;
end package;

-- --------------------------------------------------------------------------

library IEEE;
use IEEE.std_logic_1164.all;
use IEEE.std_logic_arith.all;
use IEEE.std_logic_unsigned.all;

entity rec is
	port(
		phi2, dotclk:	in std_logic;
		rst:		    in std_logic;

		dma:		    out std_logic;
		ba:		        in std_logic;

		cpu_addr:	    in std_logic_vector(15 downto 0);
		cpu_addr_out:   out std_logic_vector(15 downto 0);
		cpu_data:	    in std_logic_vector(7 downto 0);
		cpu_data_out:   out std_logic_vector(7 downto 0);
		cpu_rw:		    inout std_logic;
		cpu_rw_out:     out std_logic;
		cpu_we:         in std_logic;
        IOF:            in std_logic;

		reu_addr:	    out std_logic_vector(11 downto 0);
		reu_data:	    inout std_logic_vector(7 downto 0);
		reu_rw:		    out std_logic;
		reu_ras,
		reu_cas:	    out std_logic;

		size:		    in std_logic;
		aec:		    inout std_logic;
		reu_debug:      out std_logic_vector(7 downto 0)
	);
end entity;

-- --------------------------------------------------------------------------

architecture arch_rec of rec is

-- ACR and IMR
signal acr: std_logic_vector(1 downto 0);
signal imr: std_logic_vector(2 downto 0);

-- CR
signal execute, load, ff00:	 std_logic;
signal tt:                   std_logic_vector(1 downto 0);

-- SR
signal ip, eob, fault:       std_logic;
constant version:            std_logic_vector(3 downto 0):="0000";

signal state:              std_logic_vector(2 downto 0);
--signal state_2:              std_logic_vector(2 downto 0);

-- transfer control
signal base_reu, sadr_reu:   std_logic_vector(23 downto 0);
signal base_c64, sadr_c64:   std_logic_vector(15 downto 0);
signal xfer_cnt, xfer_len:   std_logic_vector(15 downto 0);

-- SWAP storage
signal reu_store, cpu_store: std_logic_vector(7 downto 0);

-- misc
signal ref_cnt:	std_logic_vector(11 downto 0);	-- refresh counter
signal exec_s:	std_logic;			-- exec sampled
signal ba_s:	std_logic;			-- ba sampled
signal scnt:	std_logic;			-- SWAP state

signal dma_cpu_data_out: std_logic_vector(7 downto 0);

begin

    -- --------------------------------------------------------------------------
    -- DMA transfer (rising edge)
    -- --------------------------------------------------------------------------
    process(dotclk,rst)
    begin
        if rst='0' then
            base_reu<="000000000000000000000000";
            base_c64<="0000000000000000";
            xfer_cnt<="0000000000000000";

            ref_cnt<="000000000000";

            exec_s<='0';
            ba_s<=ba;

            state<=(others=>'0');
            --state_2<=(others=>'0');
            scnt<='0';

        -- rising edge: RAM addressing & data transfer
        elsif dotclk'event and dotclk='1' then
            ba_s<=ba;

            -- mute c64 lines by default
            cpu_addr_out<="ZZZZZZZZZZZZZZZZ";
            dma_cpu_data_out<="ZZZZZZZZ";

            -- transfer control registers while idle
            if exec_s='0' then
                base_reu<=sadr_reu;
                base_c64<=sadr_c64;
                xfer_cnt<=xfer_len;

                exec_s<=execute;
                scnt<='0';

            -- memory access - Bus Available
            elsif ba='1' then
                case state is
                when "000" =>	-- apply row addr
                    reu_addr<=base_reu(11 downto 0);
                    cpu_addr_out<=base_c64;

                when "001" =>	-- apply col addr
                    reu_addr<=base_reu(23 downto 12);
                    cpu_addr_out<=base_c64;

                when "010" =>	-- transfer data
                    cpu_addr_out<=base_c64;

                    fault<='0';

                    -- Transfer type
                    case tt is
                    when "00" =>	-- C64->REU
                        reu_data<=cpu_data;

                    when "01" =>	-- REU->C64
                        dma_cpu_data_out<=reu_data;

                    when "10" =>	-- SWAP
                        if scnt='0' then
                            reu_store<=cpu_data;
                            cpu_store<=reu_data;
                        else
                            reu_data<=reu_store;
                            dma_cpu_data_out<=cpu_store;
                        end if;

                        scnt<=not(scnt);

                    when "11" =>	-- VERIFY
                        if cpu_data/=reu_data then
                            fault<='1';
                        end if;

                    when others =>
                        null;

                    end case;

                when "011" =>	-- inc addresses, dec counter
                    xfer_cnt<=xfer_cnt-1;

                    if acr(0)='1' then
                        base_reu<=base_reu+1;
                    end if;

                    if acr(0)='1' then
                        base_c64<=base_c64+1;
                    end if;


                when "100" =>	-- CBR #1
                    reu_addr<=ref_cnt;

                when "101" =>	-- CBR #2
                    reu_addr<="000000000000";

                when "111" =>	-- finish/reload
                    ref_cnt<=ref_cnt+1;

                    if xfer_cnt=0 then
                        if load='1' then
                            base_reu<=sadr_reu;
                            base_c64<=sadr_c64;
                            xfer_cnt<=xfer_len;
                        end if;

                        eob<='1';
                        exec_s<='0';
                    end if;

                when others =>
                    null;

                end case;
            end if;
        -- falling edge: RAM control
        elsif dotclk'event and dotclk='0' then
            -- RAM control defaults
            -- Active low
            reu_rw<='1';
            reu_ras<='1';
            reu_cas<='1';

            if exec_s='1' and ba_s<='1' then
                -- todo is this the dma pin?
                cpu_rw_out<='1';
                state<=state+1;

                -- RAM control state machine
                case state is
                when "000" =>	-- sample row addr
                    reu_ras<='0';
                    reu_cas<='1';

                when "001" =>	-- sample col addr
                    reu_ras<='0';
                    reu_cas<='0';

                when "010" =>	-- data write
                    reu_ras<='0';
                    reu_cas<='0';

                    case tt is
                    when "00" =>	-- C64->REU
                        reu_rw<='0';
                    when "01" =>	-- REU->C64
                        cpu_rw_out<='0';
                    when "10" =>	-- SWAP
                        if scnt='1' then
                            cpu_rw_out<='0';
                            reu_rw<='0';
                        end if;
                    when others =>
                        null;

                    end case;

                when "011" =>	-- end of transfer
                    reu_ras<='1';
                    reu_cas<='0';

                when "100" => 	-- CBR #1
                    reu_ras<='1';
                    reu_cas<='0';

                when "101" =>	-- CBR #2
                    reu_ras<='0';
                    reu_cas<='0';

                when others =>
                    reu_ras<='1';
                    reu_cas<='1';

                end case;
            end if;
        end if;
    end process;

    -- --------------------------------------------------------------------------
    -- external access to REU registers
    -- --------------------------------------------------------------------------
    reg_access:
    process(phi2,rst)
    variable reg_addr: std_logic_vector(3 downto 0);
    begin
        if rst='0' then
            execute<='0';
            load<='0';
            ff00<='0';

            sadr_c64<="0000000000000000";
            sadr_reu<="000000000000000000000000";
            xfer_len<="0000000000000000";

            imr<="000";
            acr<="00";

        elsif phi2'event and phi2='1' then
            reg_addr:=cpu_addr(3 downto 0);

            -- clear execute when finished
            if exec_s='1' and execute='1' then
                execute<='0';

            -- auto-execute on ff00 access
            elsif load='1' and cpu_addr="1111111100000000" then
                execute<='1';

            -- normal access
            elsif IOF='1' and cpu_we='1' then
                reu_debug<="00000001";

                -- DFXX writes
                case reg_addr is

                when "0001" =>	-- CR
                    execute<=cpu_data(7);
                    load<=cpu_data(5);
                    ff00<=cpu_data(4);
                    tt<=cpu_data(1 downto 0);

                when "0010" =>	-- c64 start address
                    sadr_c64(15 downto  8)<=cpu_data;
                when "0011" =>
                    sadr_c64( 7 downto  0)<=cpu_data;

                when "0100" =>	-- reu start address
                    sadr_reu(23 downto 16)<=cpu_data;
                when "0101" =>
                    sadr_reu(15 downto  8)<=cpu_data;
                when "0110" =>
                    sadr_reu( 7 downto  0)<=cpu_data;

                when "0111" =>	-- transfer length
                    xfer_len(15 downto  8)<=cpu_data;
                when "1000" =>
                    xfer_len( 7 downto  0)<=cpu_data;

                when "1001" =>	-- IMR
                    imr<=cpu_data(7 downto 5);

                when "1010" =>	-- ACR
                    acr<=cpu_data(7 downto 6);

                when others =>
                    null;

                end case;

            elsif IOF='1' and cpu_rw='1' then
                reu_debug<="00000011";

                -- DFXX reads
                case reg_addr is
                when "0000" =>	-- SR
                    cpu_data_out<=ip & eob & fault & size & version;

                when "0001" =>	-- CR
                    cpu_data_out<=execute & '1' & load & ff00 & "11" & tt;

                when "0010" =>	-- c64 start address
                    cpu_data_out<=sadr_c64(15 downto  8);
                when "0011" =>
                    cpu_data_out<=sadr_c64( 7 downto  0);

                when "0100" =>	-- reu start address
                    cpu_data_out<=sadr_reu(23 downto 16);
                when "0101" =>
                    cpu_data_out<=sadr_reu(15 downto  8);
                when "0110" =>
                    cpu_data_out<=sadr_reu( 7 downto  0);

                when "0111" =>	-- transfer length
                    cpu_data_out<=xfer_len(15 downto  8);
                when "1000" =>
                    cpu_data_out<=xfer_len( 7 downto  0);

                -- Interrupt mask register is not used apparently
                when "1001" =>	-- IMR
                    cpu_data_out<=imr & "11111";

                when "1010" =>	-- ACR
                    cpu_data_out<=acr & "111111";

                when others =>
                    null;

                end case;
            end if;
        end if;
    end process;
end architecture;
