-- -----------------------------------------------------------------------
--
--                                 FPGA 64
--
--     A fully functional commodore 64 implementation in a single FPGA
--
-- -----------------------------------------------------------------------
-- Peter Wendrich (pwsoft@syntiac.com)
-- http://www.syntiac.com/fpga64.html
-- -----------------------------------------------------------------------
--
-- System runs on 32 Mhz (derived from a 50MHz clock). 
-- The VIC-II runs in the first 4 cycles of 32 Mhz clock.
-- The CPU runs in the last 16 cycles. Effective cpu speed is 1 Mhz.
-- 4 additional cycles are used to interface with the C-One IEC port.
-- 
-- -----------------------------------------------------------------------
-- Dar 08/03/2014 
--
-- Based on fpga64_cone
-- add external selection for 15KHz(TV)/31KHz(VGA)
-- add external selection for power on NTSC(60Hz)/PAL(50Hz)
-- add external conection in/out for IEC signal
-- add sid entity 
-- -----------------------------------------------------------------------

library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.std_logic_unsigned.ALL;
use IEEE.numeric_std.all;

-- -----------------------------------------------------------------------

entity fpga64_sid_iec is
generic (
	resetCycles : integer := 4095
);
port(
	clk32       : in  std_logic;
	reset_n     : in  std_logic;
	bios        : in  std_logic_vector(1 downto 0);

	-- keyboard interface (use any ordinairy PS2 keyboard)
	ps2_key     : in  std_logic_vector(10 downto 0);

	-- external memory
	ramAddr     : out unsigned(15 downto 0);
	ramDataIn   : in  unsigned(7 downto 0);
	ramDataOut  : out unsigned(7 downto 0);
	ramCE       : out std_logic;
	ramWe       : out std_logic;

	io_cycle    : out std_logic;
	idle        : out std_logic;

	-- VGA/SCART interface
	ntscMode    : in  std_logic;
	hsync       : out std_logic;
	vsync       : out std_logic;
	r           : out unsigned(7 downto 0);
	g           : out unsigned(7 downto 0);
	b           : out unsigned(7 downto 0);
	
	-- cartridge port
	game        : in  std_logic;
	exrom       : in  std_logic;
	ioE_rom     : in  std_logic;
	ioF_rom     : in  std_logic;
	max_ram     : in  std_logic;
	irq_n       : in  std_logic;
	nmi_n       : in  std_logic;
	nmi_ack     : out std_logic;
	dma_n       : in  std_logic;
	ba          : out std_logic;
	romL        : out std_logic;	-- cart signals LCA
	romH        : out std_logic;	-- cart signals LCA
	UMAXromH 	: out std_logic;	-- cart signals LCA
	IOE			: out std_logic;	-- cart signals LCA
	IOF			: out std_logic;	-- cart signals LCA
	CPU_hasbus  : out std_logic;	-- CPU has the bus STROBE
	freeze_key  : out std_logic;
	mod_key     : out std_logic;

	ioF_ext     : in  std_logic;
	ioE_ext     : in  std_logic;
	io_data     : in  unsigned(7 downto 0);
	reu_attached: in  std_logic;

	-- joystick interface
	joyA        : in  std_logic_vector(6 downto 0);
	joyB        : in  std_logic_vector(6 downto 0);
	joyC        : in  std_logic_vector(6 downto 0);
	joyD        : in  std_logic_vector(6 downto 0);
	pot1        : in  std_logic_vector(7 downto 0);
	pot2        : in  std_logic_vector(7 downto 0);
	pot3        : in  std_logic_vector(7 downto 0);
	pot4        : in  std_logic_vector(7 downto 0);

	--Connector to the SID
	audio_data  : out std_logic_vector(17 downto 0);
	extfilter_en: in  std_logic;
	sid_ver     : in  std_logic;
	sid_we_ext  : out std_logic;
	sid_mode    : in  std_logic_vector(1 downto 0);

	-- IEC
	iec_data_o	: out std_logic;
	iec_data_i	: in  std_logic;
	iec_clk_o	: out std_logic;
	iec_clk_i	: in  std_logic;
	iec_atn_o	: out std_logic;
	
	c64rom_addr : in  std_logic_vector(13 downto 0);
	c64rom_data : in  std_logic_vector(7 downto 0);
	c64rom_wr   : in  std_logic;

	cass_motor  : out std_logic;
	cass_write  : out std_logic;
	cass_sense  : in  std_logic;
	cass_in     : in  std_logic;

    phi2_p      : out std_logic;
    phi2_n      : out std_logic;
    phi2        : out std_logic;
    aec_out     : out std_logic;
    cpu_we      : out std_logic;
    -- Allows us to get CPU data out and into the REU if attached
    cpu_do      : out std_logic_vector(7 downto 0);

	uart_enable : in  std_logic;

	uart_txd    : out std_logic; -- CIA2, PortA(2) 
	uart_rts    : out std_logic; -- CIA2, PortB(1)
	uart_dtr    : out std_logic; -- CIA2, PortB(2)
	uart_ri_out	: out std_logic; -- CIA2, PortB(3)
	uart_dcd_out: out std_logic; -- CIA2, PortB(4)

	uart_rxd    : in std_logic; -- CIA2, PortB(0)
	uart_ri_in  : in std_logic; -- CIA2, PortB(3)
	uart_dcd_in : in std_logic; -- CIA2, PortB(4)
	uart_cts    : in std_logic; -- CIA2, PortB(6)
	uart_dsr    : in std_logic  -- CIA2, PortB(7)
);
end fpga64_sid_iec;

-- -----------------------------------------------------------------------

architecture rtl of fpga64_sid_iec is
-- System state machine
type sysCycleDef is (
	CYCLE_IDLE0, CYCLE_IDLE1, CYCLE_IDLE2, CYCLE_IDLE3,
	CYCLE_IDLE4, CYCLE_IDLE5, CYCLE_IDLE6, CYCLE_IDLE7,
	CYCLE_IEC0,  CYCLE_IEC1,  CYCLE_IEC2,  CYCLE_IEC3,
	CYCLE_VIC0,  CYCLE_VIC1,  CYCLE_VIC2,  CYCLE_VIC3,
	CYCLE_CPU0,  CYCLE_CPU1,  CYCLE_CPU2,  CYCLE_CPU3,
	CYCLE_CPU4,  CYCLE_CPU5,  CYCLE_CPU6,  CYCLE_CPU7,
	CYCLE_CPU8,  CYCLE_CPU9,  CYCLE_CPUA,  CYCLE_CPUB,
	CYCLE_CPUC,  CYCLE_CPUD,  CYCLE_CPUE,  CYCLE_CPUF
);

signal sysCycle     : sysCycleDef := sysCycleDef'low;
signal phi0_cpu     : std_logic;
signal cpuHasBus    : std_logic;

signal baLoc        : std_logic;
signal irqLoc       : std_logic;
signal nmiLoc       : std_logic;
signal aec          : std_logic;

signal enableCpu    : std_logic;
signal enableVic    : std_logic;
signal enablePixel  : std_logic;

signal irq_cia1     : std_logic;
signal irq_cia2     : std_logic;
signal irq_vic      : std_logic;

signal systemWe     : std_logic;
signal pulseWrRam   : std_logic;
signal colorWe      : std_logic;
signal systemAddr   : unsigned(15 downto 0);
signal ramDataReg   : unsigned(7 downto 0);

signal cs_vic       : std_logic;
signal cs_sid       : std_logic;
signal cs_color     : std_logic;
signal cs_cia1      : std_logic;
signal cs_cia2      : std_logic;
signal cs_ram       : std_logic;
signal cs_ioE       : std_logic;
signal cs_ioF       : std_logic;
signal cs_romL      : std_logic;
signal cs_romH      : std_logic;
signal cs_UMAXromH  : std_logic;							-- romH VIC II read flag
signal cpuWe        : std_logic;
signal cpuAddr      : unsigned(15 downto 0);
signal cpuDi        : unsigned(7 downto 0);
signal cpuDo        : unsigned(7 downto 0);

signal cpuIO        : unsigned(7 downto 0);

signal reset        : std_logic := '1';
signal reset_cnt    : integer range 0 to resetCycles := 0;

-- CIA signals
signal enableCia_p  : std_logic;
signal enableCia_n  : std_logic;
signal cia1Do       : unsigned(7 downto 0);
signal cia2Do       : unsigned(7 downto 0);
signal cia1_pai     : unsigned(7 downto 0);
signal cia1_pao     : unsigned(7 downto 0);
signal cia1_pbi     : unsigned(7 downto 0);
signal cia1_pbo     : unsigned(7 downto 0);
signal cia2_pai     : unsigned(7 downto 0);
signal cia2_pao     : unsigned(7 downto 0);
signal cia2_pbi     : unsigned(7 downto 0);
signal cia2_pbo     : unsigned(7 downto 0);

signal todclk       : std_logic;
signal toddiv       : std_logic_vector(19 downto 0);
signal toddiv3      : std_logic_vector(1 downto 0);

-- video
signal vicColorIndex: unsigned(3 downto 0);
signal vicHSync     : std_logic;
signal vicVSync     : std_logic;
signal vicBus       : unsigned(7 downto 0);
signal vicDi        : unsigned(7 downto 0);
signal vicDiAec     : unsigned(7 downto 0);
signal vicAddr      : unsigned(15 downto 0);
signal vicData      : unsigned(7 downto 0);
signal lastVicDi    : unsigned(7 downto 0);
signal vicAddr1514  : unsigned(1 downto 0);
signal colorQ       : unsigned(3 downto 0);
signal colorData    : unsigned(3 downto 0);
signal colorDataAec : unsigned(3 downto 0);

-- SID signals
signal sid_do       : std_logic_vector(7 downto 0);
signal sid_do6581   : std_logic_vector(7 downto 0);
signal sid_do8580   : std_logic_vector(7 downto 0);
signal sid_we       : std_logic;
signal sid_sel_int  : std_logic;
signal audio_6581   : signed(17 downto 0);
signal pot_x1       : std_logic_vector(7 downto 0);
signal pot_y1       : std_logic_vector(7 downto 0);
signal pot_x2       : std_logic_vector(7 downto 0);
signal pot_y2       : std_logic_vector(7 downto 0);
signal audio_8580   : std_logic_vector(17 downto 0);

signal clk_1MHz     : std_logic_vector(31 downto 0);

component sid8580
	port (
		reset         : in std_logic;
		clk           : in std_logic;
		ce_1m         : in std_logic;
		we            : in std_logic;
		addr          : in std_logic_vector(4 downto 0);
		data_in       : in std_logic_vector(7 downto 0);
		data_out      : out std_logic_vector(7 downto 0);
		pot_x         : in std_logic_vector(7 downto 0);
		pot_y         : in std_logic_vector(7 downto 0);
		audio_data    : out std_logic_vector(17 downto 0);
		extfilter_en  : in std_logic
  );
end component sid8580;

component mos6526
	PORT (
		clk           : in  std_logic;
		mode          : in  std_logic := '0'; -- 0 - 6526 "old", 1 - 8521 "new"
		phi2_p        : in  std_logic;
		phi2_n        : in  std_logic;
		res_n         : in  std_logic;
		cs_n          : in  std_logic;
		rw            : in  std_logic; -- '1' - read, '0' - write
		rs            : in  unsigned(3 downto 0);
		db_in         : in  unsigned(7 downto 0);
		db_out        : out unsigned(7 downto 0);
		pa_in         : in  unsigned(7 downto 0);
		pa_out        : out unsigned(7 downto 0);
		pb_in         : in  unsigned(7 downto 0);
		pb_out        : out unsigned(7 downto 0);
		flag_n        : in  std_logic;
		pc_n          : out std_logic;
		tod           : in  std_logic;
		sp_in         : in  std_logic;
		sp_out        : out std_logic;
		cnt_in        : in  std_logic;
		cnt_out       : out std_logic;
		irq_n         : out std_logic
	);
end component; 

begin

-- -----------------------------------------------------------------------
-- Local signal to outside world
-- -----------------------------------------------------------------------
ba <= baLoc;

io_cycle <= '1' when
	(sysCycle = CYCLE_IDLE0) or (sysCycle = CYCLE_IDLE1) or
	(sysCycle = CYCLE_IDLE2) or (sysCycle = CYCLE_IDLE3) or
	(sysCycle = CYCLE_IEC0)  or (sysCycle = CYCLE_IEC1)  or 
	(sysCycle = CYCLE_IEC2)  or (sysCycle = CYCLE_IEC3)  else '0';

idle <= '1' when
	(sysCycle = CYCLE_IDLE4) or (sysCycle = CYCLE_IDLE5) or
	(sysCycle = CYCLE_IDLE6) or (sysCycle = CYCLE_IDLE7) else '0';

phi2_p <= enableCia_p;
phi2_n <= enableCia_n;

cpu_we <= cpuWe;

-- -----------------------------------------------------------------------
-- System state machine, controls bus accesses
-- and triggers enables of other components
-- -----------------------------------------------------------------------
process(clk32)
begin
	if rising_edge(clk32) then
		if sysCycle = sysCycleDef'high then
			sysCycle <= sysCycleDef'low;
		else
			sysCycle <= sysCycleDef'succ(sysCycle);
		end if;
	end if;
end process;

-- PHI0/2-clock emulation
process(clk32)
begin
	if rising_edge(clk32) then
		if sysCycle = sysCycleDef'pred(CYCLE_CPU0) then
			phi0_cpu <= '1';
			if baLoc = '1' or cpuWe = '1' then
				cpuHasBus <= '1';
			end if;
		end if;
		if sysCycle = sysCycleDef'high then
			phi0_cpu <= '0';
			cpuHasBus <= '0';
		end if;
	end if;
end process;

process(clk32)
begin
	if rising_edge(clk32) then
		enableVic <= '0';
		enableCia_n <= '0';
		enableCia_p <= '0';
		enableCpu <= '0';
        phi2 <= '0';
		case sysCycle is
		when CYCLE_VIC2 =>
			enableVic <= '1';
		when CYCLE_CPUC =>
		    phi2 <= '1';
			enableCia_n <= '1';
        when CYCLE_CPUD =>
            phi2 <= '1';
        when CYCLE_CPUE =>
            enableVic <= '1';
            enableCpu <= '1';
            phi2 <= '1';
		when CYCLE_CPUF =>
		    phi2 <= '1';
			enableCia_p <= '1';
		when others =>
			null;
		end case;
	end if;
end process;

hSync <= vicHSync;
vSync <= vicVSync;

c64colors: entity work.fpga64_rgbcolor
port map (
	index => vicColorIndex,
	r => r,
	g => g,
	b => b
);

-- -----------------------------------------------------------------------
-- Color RAM
-- -----------------------------------------------------------------------
colorram: entity work.spram
generic map (
	DATA_WIDTH => 4,
	ADDR_WIDTH => 10
)
port map (
	clk => clk32,
	we => colorWe,
	addr => systemAddr(9 downto 0),
	data => cpuDo(3 downto 0),
	q => colorQ
);

process(clk32)
begin
	if rising_edge(clk32) then
		colorWe <= (cs_color and pulseWrRam);
		colorData <= colorQ;
	end if;
end process;

aec_out <= aec;

-- -----------------------------------------------------------------------
-- PLA and bus-switches
-- -----------------------------------------------------------------------
buslogic: entity work.fpga64_buslogic
port map (
	clk => clk32,
	reset => reset,
	bios => bios,

	cpuHasBus => cpuHasBus,
	aec => aec,

	bankSwitch => cpuIO(2 downto 0),

	game => game,
	exrom => exrom,
	ioE_rom => ioE_rom,
	ioF_rom => ioF_rom,
	max_ram => max_ram,

	ramData => ramDataReg,

	ioF_ext => ioF_ext,
	ioE_ext => ioE_ext,
	io_data => io_data,

	cpuWe => cpuWe,
	cpuAddr => cpuAddr,
	cpuData => cpuDo,
	vicAddr => vicAddr,
	vicData => vicData,
	sidData => unsigned(sid_do),
	colorData => colorData,
	cia1Data => cia1Do,
	cia2Data => cia2Do,
	lastVicData => lastVicDi,

	systemWe => systemWe,
	systemAddr => systemAddr,
	dataToCpu => cpuDi,
	dataToVic => vicDi,

	reu_attached => reu_attached,
	dma_n => dma_n,

	cs_vic => cs_vic,
	cs_sid => cs_sid,
	cs_color => cs_color,
	cs_cia1 => cs_cia1,
	cs_cia2 => cs_cia2,
	cs_ram => cs_ram,
	cs_ioE => cs_ioE,
	cs_ioF => cs_ioF,
	cs_romL => cs_romL,
	cs_romH => cs_romH,
	cs_UMAXromH => cs_UMAXromH,

	c64rom_addr => c64rom_addr,
	c64rom_data => c64rom_data,
	c64rom_wr => c64rom_wr
);

process(clk32)
begin
	if rising_edge(clk32) then
		pulseWrRam <= '0';
		if cpuWe = '1' then
			if sysCycle = CYCLE_CPUC then
				pulseWrRam <= '1';
			end if;
		end if;
	end if;
end process;

-- -----------------------------------------------------------------------
-- VIC-II video interface chip
-- -----------------------------------------------------------------------
process(clk32)
begin
	if rising_edge(clk32) then
		if phi0_cpu = '1' then
			if cpuWe = '1' and cs_vic = '1' then
				vicBus <= cpuDo;
			else
				vicBus <= x"FF";
			end if;
		end if;
	end if;
end process;

cpu_do <= std_logic_vector(cpuDo);

-- In the first three cycles after BA went low, the VIC reads
-- $ff as character pointers and
-- as color information the lower 4 bits of the opcode after the access to $d011.
vicDiAec <= vicBus when aec = '0' else vicDi;
colorDataAec <= cpuDi(3 downto 0) when aec = '0' else colorData;

vic: entity work.video_vicii_656x
generic map (
	registeredAddress => false,
	emulateRefresh => true,
	emulateLightpen => true,
	emulateGraphics => true
)			
port map (
	clk => clk32,
	reset => reset,
	enaPixel => enablePixel,
	enaData => enableVic,
	-- CPU clock from the VIC
	phi => phi0_cpu,
	
	baSync => '0',
	ba => baLoc,

	mode6569 => (not ntscMode),
	mode6567old => '0',
	mode6567R8 => ntscMode,
	mode6572 => '0',
	
	cs => cs_vic,
	we => cpuWe,
	lp_n => cia1_pbi(4),

	aRegisters => cpuAddr(5 downto 0),
	diRegisters => cpuDo,
	di => vicDiAec,
	diColor => colorDataAec,
	do => vicData,

	vicAddr => vicAddr(13 downto 0),
	addrValid => aec,

	hsync => vicHSync,
	vsync => vicVSync,
	colorIndex => vicColorIndex,

	irq_n => irq_vic
);

-- Pixel timing
process(clk32)
begin
	if rising_edge(clk32) then
		enablePixel <= '0';
		if sysCycle = CYCLE_VIC2
		or sysCycle = CYCLE_IDLE2
		or sysCycle = CYCLE_IDLE6
		or sysCycle = CYCLE_IEC2
		or sysCycle = CYCLE_CPU2
		or sysCycle = CYCLE_CPU6
		or sysCycle = CYCLE_CPUA
		or sysCycle = CYCLE_CPUE then
			enablePixel <= '1';
		end if;
	end if;
end process;

-- -----------------------------------------------------------------------
-- SID
-- -----------------------------------------------------------------------
div1m: process(clk32)				-- this process devides 32 MHz to 1MHz (for the SID)
begin									
	if (rising_edge(clk32)) then			    			
		if (reset = '1') then				
			clk_1MHz 	<= "00000000000000000000000000000001";
		else
			clk_1MHz(31 downto 1) <= clk_1MHz(30 downto 0);
			clk_1MHz(0)           <= clk_1MHz(31);
		end if;
	end if;
end process;

audio_data  <= std_logic_vector(audio_6581) when sid_ver='0' else audio_8580;

sid_we      <= pulseWrRam and phi0_cpu and cs_sid;
sid_sel_int <= not sid_mode(1) or (not sid_mode(0) and not cpuAddr(5)) or (sid_mode(0) and not cpuAddr(8));
sid_we_ext  <= sid_we and (not sid_mode(1) or not sid_sel_int);
sid_do      <= std_logic_vector(io_data) when sid_sel_int = '0' else sid_do6581 when sid_ver='0' else sid_do8580;

pot_x1 <= (others => '1' ) when cia1_pao(6) = '0' else not pot1;
pot_y1 <= (others => '1' ) when cia1_pao(6) = '0' else not pot2;
pot_x2 <= (others => '1' ) when cia1_pao(7) = '0' else not pot3;
pot_y2 <= (others => '1' ) when cia1_pao(7) = '0' else not pot4;

sid_6581: entity work.sid_top
port map (
	clock => clk32,
	reset => reset,

	addr => "000" & cpuAddr(4 downto 0),
	wren => sid_we and sid_sel_int,
	wdata => std_logic_vector(cpuDo),
	rdata => sid_do6581,

	potx => pot_x1 and pot_x2,
	poty => pot_y1 and pot_y2,

	comb_wave_l => '0',
	comb_wave_r => '0',

	extfilter_en => extfilter_en,

	start_iter => clk_1MHz(31),
	sample_left => audio_6581,
	sample_right => open
);

sid_8580 : sid8580
port map (
	reset => reset,
	clk => clk32,
	ce_1m => clk_1MHz(31),
	we => sid_we and sid_sel_int,
	addr => std_logic_vector(cpuAddr(4 downto 0)),
	data_in => std_logic_vector(cpuDo),
	data_out => sid_do8580,
	pot_x => pot_x1 and pot_x2,
	pot_y => pot_y1 and pot_y2,
	audio_data => audio_8580,
	extfilter_en => extfilter_en
);

-- -----------------------------------------------------------------------
-- CIAs
-- -----------------------------------------------------------------------
cia1: mos6526
port map (
	clk => clk32,
	phi2_p => enableCia_p,
	phi2_n => enableCia_n,
	res_n => not reset,
	cs_n => not cs_cia1,
	rw => not cpuWe,

	rs => cpuAddr(3 downto 0),
	db_in => cpuDo,
	db_out => cia1Do,

	pa_in => cia1_pai,
	pa_out => cia1_pao,
	pb_in => cia1_pbi,
	pb_out => cia1_pbo,

	flag_n => cass_in,
	sp_in => '1',
	cnt_in => '1',

	tod => todclk, --vicVSync,

	irq_n => irq_cia1
);

cia2: mos6526
port map (
	clk => clk32,
	phi2_p => enableCia_p,
	phi2_n => enableCia_n,
	res_n => not reset,
	cs_n => not cs_cia2,
	rw => not cpuWe,

	rs => cpuAddr(3 downto 0),
	db_in => cpuDo,
	db_out => cia2Do,

	pa_in => cia2_pai,
	pa_out => cia2_pao,
	pb_in => cia2_pbi,
	pb_out => cia2_pbo,

	-- Looks like most of the old terminal programs use the FLAG_N input (and to PB0) on CIA2 to
	-- trigger an interrupt on the falling edge of the RXD input.
	-- (and they don't use the "SP" pin for some reason?) ElectronAsh.
	flag_n => uart_rxd,
	
	sp_in => uart_rxd,	-- Hooking up to the SP pin anyway, ready for the "UP9600" style serial.
	cnt_in => '1',

	tod => todclk, --vicVSync,

	irq_n => irq_cia2
);
	
-- generate TOD clock from stable 32 MHz
-- Can we simply use vicVSync?
process(clk32, reset)
begin
	if reset = '1' then
		todclk <= '0';
		toddiv <= (others => '0');
	elsif rising_edge(clk32) then
		toddiv <= toddiv + 1;
		if (ntscMode = '1' and toddiv = 266665 and toddiv3 = "00") or
			(ntscMode = '1' and toddiv = 266666 and toddiv3 /= "00") or
			toddiv = 319999 then
			toddiv <= (others => '0');
			todclk <= not todclk;
			toddiv3 <= toddiv3 + 1;
			if toddiv3 = "10" then toddiv3 <= "00"; end if;
		end if;
	end if;
end process;
	

-- -----------------------------------------------------------------------
-- 6510 CPU
-- -----------------------------------------------------------------------
cpu: entity work.cpu_6510
port map (
	clk => clk32,
	reset => reset,
	enable => enableCpu,
	nmi_n => nmiLoc,
	nmi_ack => nmi_ack,
	irq_n => irqLoc,
	-- dma can pause the cpu now
	rdy => dma_n and baLoc,

	di => cpuDi,
	addr => cpuAddr,
	do => cpuDo,
	we => cpuWe,

	diIO => cpuIO(7) & cpuIO(6) & cpuIO(5) & cass_sense & cpuIO(3) & "111",
	doIO => cpuIO
);

cass_motor <= cpuIO(5);
cass_write <= cpuIO(3);

-- -----------------------------------------------------------------------
-- Keyboard
-- -----------------------------------------------------------------------
Keyboard: entity work.fpga64_keyboard
port map (
	clk => clk32,
	ps2_key => ps2_key,

	joyA => not unsigned(joyA(4 downto 0)),
	joyB => not unsigned(joyB(4 downto 0)),
	pai => cia1_pao,
	pbi => cia1_pbo,
	pao => cia1_pai,
	pbo => cia1_pbi,

	restore_key => freeze_key,
	mod_key => mod_key,
	backwardsReadingEnabled => '1'
);

-- -----------------------------------------------------------------------
-- Reset button
-- -----------------------------------------------------------------------
calcReset: process(clk32)
begin
	if rising_edge(clk32) then
		if sysCycle = sysCycleDef'high then
			if reset_cnt = resetCycles then
				reset <= '0';
			else
				reset <= '1';
				reset_cnt <= reset_cnt + 1;
			end if;
		end if;
		-- if reset_n = '0' or dma_n = '0' then -- temp reset fix
		if reset_n = '0' then -- temp reset fix
			reset_cnt <= 0;
		end if;
	end if;
end process;

iec_data_o <= not cia2_pao(5);
iec_clk_o <= not cia2_pao(4);
iec_atn_o <= not cia2_pao(3);
ramDataOut <= "00" & cia2_pao(5 downto 3) & "000" when sysCycle >= CYCLE_IEC0 and sysCycle <= CYCLE_IEC3 else cpuDo;
ramAddr <= systemAddr;
ramWe <= '0' when sysCycle = CYCLE_IEC2 or sysCycle = CYCLE_IEC3 else not systemWe;
ramCE <= '0' when sysCycle /= CYCLE_IDLE0 and sysCycle /= CYCLE_IDLE1 and sysCycle /= CYCLE_IDLE2 and sysCycle /= CYCLE_IDLE3 and
						sysCycle /= CYCLE_IDLE4 and sysCycle /= CYCLE_IDLE5 and sysCycle /= CYCLE_IDLE6 and sysCycle /= CYCLE_IDLE7 and
						sysCycle /= CYCLE_IEC0  and sysCycle /= CYCLE_IEC1  and sysCycle /= CYCLE_IEC2  and sysCycle /= CYCLE_IEC3  and
						sysCycle /= CYCLE_CPU0  and sysCycle /= CYCLE_CPU1  and sysCycle /= CYCLE_CPUF  and
						cs_ram = '1' else '1';

process(clk32)
begin
	if rising_edge(clk32) then
		if sysCycle = CYCLE_CPUD or sysCycle = CYCLE_VIC2 then
			ramDataReg <= unsigned(ramDataIn);
		end if;
		if sysCycle = CYCLE_VIC3 then
			lastVicDi <= vicDi;
		end if;
	end if;
end process;

--serialBus
serialBus: process(clk32)
begin
	if rising_edge(clk32) then
		if sysCycle = CYCLE_IEC1 then
			cia2_pai(7) <= iec_data_i and not cia2_pao(5);
			cia2_pai(6) <= iec_clk_i and not cia2_pao(4);
		end if;
	end if;
end process;

cia2_pai(5 downto 0) <= cia2_pao(5 downto 0);

process(joyC, joyD, cia2_pbo, uart_rxd, uart_ri_in, uart_dcd_in, uart_cts, uart_dsr, uart_enable)
begin
	if uart_enable = '1' then
		cia2_pbi(0) <= uart_rxd;
		cia2_pbi(1) <= '1';
		cia2_pbi(2) <= '1';
		cia2_pbi(3) <= uart_ri_in;
		cia2_pbi(4) <= uart_dcd_in;
		cia2_pbi(5) <= '1';
		cia2_pbi(6) <= uart_cts;
		cia2_pbi(7) <= uart_dsr;
	else
		if cia2_pbo(7) = '1' then
			cia2_pbi(3 downto 0) <= not unsigned(joyC(3 downto 0));
		else
			cia2_pbi(3 downto 0) <= not unsigned(joyD(3 downto 0));
		end if;
		if joyC(6 downto 4) /= "000" then
			cia2_pbi(4) <= '0';
		else
			cia2_pbi(4) <= '1';
		end if;
		if joyD(6 downto 4) /= "000" then
			cia2_pbi(5) <= '0';
		else
			cia2_pbi(5) <= '1';
		end if;
		cia2_pbi(7 downto 6) <= cia2_pbo(7 downto 6);
	end if;
end process;

-- UART outputs...
uart_txd <= cia2_pao(2);
uart_rts <= cia2_pbo(1);
uart_dtr <= cia2_pbo(2);
uart_ri_out <= cia2_pbo(3);
uart_dcd_out <= cia2_pbo(4);

-- -----------------------------------------------------------------------
-- VIC bank to address lines
-- -----------------------------------------------------------------------
-- The glue logic on a C64C will generate a glitch during 10 <-> 01
-- generating 00 (in other words, bank 3) for one cycle.
--
-- When using the data direction register to change a single bit 0->1
-- (in other words, decreasing the video bank number by 1 or 2),
-- the bank change is delayed by one cycle. This effect is unstable.
process(clk32)
begin
	if rising_edge(clk32) then
		if phi0_cpu = '0' and enableVic = '1' then
			vicAddr1514 <= not cia2_pao(1 downto 0);
		end if;
	end if;
end process;

-- emulate only the first glitch (enough for Undead from Emulamer)
vicAddr(15 downto 14) <= "11" when ((vicAddr1514 xor not cia2_pao(1 downto 0)) = "11") and (cia2_pao(0) /= cia2_pao(1)) else not unsigned(cia2_pao(1 downto 0));

-- -----------------------------------------------------------------------
-- Interrupt lines
-- -----------------------------------------------------------------------
irqLoc <= irq_cia1 and irq_vic and irq_n; 
nmiLoc <= irq_cia2 and nmi_n;

-- -----------------------------------------------------------------------
-- Cartridge port lines LCA
-- -----------------------------------------------------------------------
romL <= cs_romL;
romH <= cs_romH;
IOE <= cs_ioE;
IOF <= cs_ioF;
UMAXromH <= cs_UMAXromH;
CPU_hasbus <= cpuHasBus;

end architecture;
