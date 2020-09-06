
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;
library components_lib;


entity i2c_controller is
    Port (
        clk_master   : in  STD_LOGIC;  -- we aspect a 32MHz clock
        nreset       : in  STD_LOGIC;
        scl_in       : in  STD_LOGIC;
        sda_in       : in  STD_LOGIC;
        scl_out      : out STD_LOGIC;
        sda_out      : out STD_LOGIC;
		  
		  
		  -- for probing
		  address : out STD_LOGIC_VECTOR(7 downto 0);
		  laddr : out STD_LOGIC;
		  
		  byte1 : out STD_LOGIC_VECTOR(7 downto 0);
		  lbyte1 : out STD_LOGIC;
		  
		  byte2 : out STD_LOGIC_VECTOR(7 downto 0);
		  lbyte2 : out STD_LOGIC;
		  
		  pro1 : out STD_LOGIC;
		  pro2 : out STD_LOGIC;
		  pro3 : out STD_LOGIC;
		  pro4 : out STD_LOGIC
    );
end i2c_controller;


architecture Behavioral of i2c_controller is

	---------------- COMPONENTS ----------------
	
    component clk_2N_divider 
        Port ( clk_in  : in  STD_LOGIC;
               clk_out : out STD_LOGIC;
               nreset  : in  STD_LOGIC;
               n_scale : in  STD_LOGIC_VECTOR (15 downto 0)
        );
    end component;
    
    
    component frequency_counter is
        Port ( 
            clk_ref  : in  STD_LOGIC;
            clk      : in  STD_LOGIC;
            en       : in  STD_LOGIC;
            
            cout     : out STD_LOGIC_VECTOR(15 downto 0);
            eval     : out STD_LOGIC
        );
    end component;
    
    
    component glitch_filter_array
        Generic ( 
            PORT_WIDTH   : INTEGER;
            OUT_ON_RESET : STD_LOGIC
        );
        Port ( 
            clk       : in  STD_LOGIC;
            nreset    : in  STD_LOGIC;   -- asynchronous reset
            sync_rst  : in  STD_LOGIC;   -- synchronous reset
            signal_i  : in  STD_LOGIC_VECTOR( 1 downto 0 );
            signal_o  : out STD_LOGIC_VECTOR( 1 downto 0 )
        );
    end component;
    
    
    component start_stop_detect is
        Port ( 
            clk      : in  STD_LOGIC;
            nreset   : in  STD_LOGIC;
            sync_rst : in  STD_LOGIC;
            scl      : in  STD_LOGIC;
            sda      : in  STD_LOGIC;
            start    : out STD_LOGIC;
            stop     : out STD_LOGIC
        );
    end component;
    
    
    component serial2parallel
        Generic (
            DATA_WIDTH : INTEGER range 1 to (INTEGER'high)
        );
        Port ( 
            nreset    : in  STD_LOGIC;                                       -- if active, it disables the acquisition (active low) 
            da_en     : in  STD_LOGIC;                                       -- data bit acquire eneable
            data_in   : in  STD_LOGIC;
            data_out  : out STD_LOGIC_VECTOR( (DATA_WIDTH - 1 ) downto 0 );
            dstartrec : out STD_LOGIC;                                       -- active at first recorded bit (data start recording)
            dready    : out STD_LOGIC;                                       -- data ready (active high)
            old_data  : out STD_LOGIC
        );
    end component;
    
        
    component register_sr_load is
        Generic ( 
            DATA_WIDTH  : INTEGER range 1 to (INTEGER'high) := 8
        );
        Port (
            clk             : in  STD_LOGIC;
            reset           : in  STD_LOGIC;    -- sync reset
            set             : in  STD_LOGIC;
            load            : in  STD_LOGIC;       
            def_set_value   : in  STD_LOGIC_VECTOR( (DATA_WIDTH - 1) downto 0 );
            D               : in  STD_LOGIC_VECTOR( (DATA_WIDTH - 1) downto 0 );
            Q               : out STD_LOGIC_VECTOR( (DATA_WIDTH - 1) downto 0 )
         );
    end component;


    component parallel2serial is
        Generic (
            DATA_WIDTH : INTEGER range 1 to (INTEGER'high) := 8
        );
        Port ( 
            enable      : in  STD_LOGIC;                                       -- if active, it disables the acquisition (active low) 
            shift       : in  STD_LOGIC;                                       -- data bit acquire eneable
            data_in     : in  STD_LOGIC_VECTOR( (DATA_WIDTH - 1 ) downto 0 );
            data_out    : out STD_LOGIC;
            dstartser   : out STD_LOGIC;                                       -- active at first recorded bit (data start recording)
            dfinishser  : out STD_LOGIC                                        -- data ready (active high)
        );
    end component;
    
    ------------------------------------------------
    
    
    signal scl_i, sda_i : STD_LOGIC;
    signal scl_o, sda_o : STD_LOGIC;
    
    -- STATUS SIGNALS
    signal ss_start_cond_detect, ss_stop_cond_detect     : STD_LOGIC;
    signal ss_data_in_detect, ss_data_in_ready           : STD_LOGIC;
    signal ss_ack_sent, ss_ack_in_ready                  : STD_LOGIC;
    signal ss_start_send_data_out, ss_done_send_data_out : STD_LOGIC;
       
    -- CONTROL SIGNALS
    subtype sda_out_mux_sel is STD_LOGIC_VECTOR(1 downto 0);
    
    signal cs_data_in_acq_en  : STD_LOGIC;
    signal cs_receive_ack_en  : STD_LOGIC;
    signal cs_send_ack_en     : STD_LOGIC;
    signal cs_send_data_out   : STD_LOGIC;
    signal cs_sda_out_sel     : sda_out_mux_sel;
    
    constant t_sda_out_idle : sda_out_mux_sel := "00";
    constant t_sda_out_ack  : sda_out_mux_sel := "01";
    constant t_sda_out_data : sda_out_mux_sel := "11";

    -- SINCHRONIZATION SIGNALS
    signal scl_ticks_cnt   : STD_LOGIC_VECTOR(15 downto 0);
    signal scl_h_ticks_cnt : STD_LOGIC_VECTOR(15 downto 0);
    signal scl_q_ticks_cnt : STD_LOGIC_VECTOR(15 downto 0);
    signal scl_clk_q_tick  : STD_LOGIC;
    
	-- I2C SIGNALS
	signal i2c_addr : STD_LOGIC_VECTOR(7 downto 0);
	signal i2c_cmd  : STD_LOGIC;
	signal i2c_data : STD_LOGIC_VECTOR(7 downto 0);
	
    -- DATA SIGNALS
    signal data_in  : STD_LOGIC_VECTOR(7 downto 0);
    signal data_out : STD_LOGIC;
    signal ack_out  : STD_LOGIC;
    signal ack_in   : STD_LOGIC;

    constant I2C_ACK : STD_LOGIC := '0';         
    
    signal slave_address : STD_LOGIC_VECTOR(7 downto 0) := x"2a"; 
    
    signal data_acquire: STD_LOGIC; 
    signal start_byte_rec, byte_read : STD_LOGIC; 
    
    signal en_ack : STD_LOGIC;

begin



    -- Signal Interface:
    -- Input
    -- -- (control) clk_master, nreset
    -- -- (data) scl_in, sda_in
    -- Output
     -- -- (data) scl_i, sda_i
    signal_interface: block
        signal iSignals, fSignals  : STD_LOGIC_VECTOR(1 downto 0);
        -- signal v_sda_i, v_ack      : STD_LOGIC_VECTOR(1 downto 0);
        signal clk_signal          : STD_LOGIC;   
    begin
    
        iSignals <= scl_in & sda_in;
    
        clk_singal_interface: clk_2N_divider
            Port Map (
                clk_in  =>  clk_master,
                clk_out =>  clk_signal,
                nreset  =>  nreset,
                n_scale =>  x"0002"
            );
            
        
        signal_filter: glitch_filter_array
            generic map (
                PORT_WIDTH   =>  2,
                OUT_ON_RESET =>  '1'
            )
            port map (
                clk      =>  clk_signal,
                nreset   =>  nreset,
                sync_rst =>  '0',
		        signal_i =>  iSignals,
                signal_o =>  fSignals
	        );  
	        
        scl_i <= fsignals(1);
        sda_i <= fsignals(0);     
                
    end block signal_interface;
    
	 
	-- Signal Interface:
    -- Input
    -- -- (control) clk_master, nreset, start_cond_detect, stop_cond_detect
    -- -- (data) sda_i, scl_i
    -- Output
     -- -- (data) scl_ticks_cnt, scl_h_ticks_cnt, scl_q_ticks_cn
     -- -- (clk) scl_clk_q_tick
    slave_frequency_counter: block
        signal en_fcount : STD_LOGIC;
        signal cout      : STD_LOGIC_VECTOR(15 downto 0);
        signal tcount    : STD_LOGIC_VECTOR(15 downto 0);
        signal eval      : STD_LOGIC;
        signal reset     : STD_LOGIC;
        
        signal count_en  : STD_LOGIC;
        signal cnt       : STD_LOGIC_VECTOR(15 downto 0);
    begin
    
        ff_sr_fcount_en: process( clk_master, nreset )
        begin
            if ( nreset = '0' ) then
                en_fcount <= '0';
            elsif ( rising_edge( clk_master ) ) then
                if ( ( ss_start_cond_detect = '1' ) and ( eval = '1' ) ) then
                    en_fcount <= 'X';
                elsif ( ss_start_cond_detect = '1' ) then
                    en_fcount <= '1';
                elsif ( eval = '1' ) then
                    en_fcount <= '0';
                end if;
            end if;  
        end process ff_sr_fcount_en;
        
        
        fcounter: frequency_counter
            port map (
                clk_ref => clk_master,
                clk     => scl_i,
                en      => en_fcount,
                cout    => cout,
                eval    => eval
            );
            
            
        reset <= not nreset;    
            
        reg_fcount: register_sr_load
            Generic Map (
                DATA_WIDTH    =>  16
            )
            Port Map (
                clk           =>  clk_master,
                reset         =>  reset,
                set           =>  ss_stop_cond_detect,
                load          =>  eval,
                def_set_value =>  x"0000",
                D             =>  cout,
                Q             =>  tcount
            );
            
        scl_ticks_cnt   <= '0' & tcount(15 downto 1);
        scl_h_ticks_cnt <= "00" & tcount(15 downto 2);
        scl_q_ticks_cnt <= "000" & tcount(15 downto 3);      
        
		  
        scl_sda_sync_signal_gen: process( scl_i, clk_master )
            variable out_scl_q : STD_LOGIC := '0';
        begin
				if ( scl_i = '1' ) then
					cnt <= scl_q_ticks_cnt;
                    out_scl_q := '0';
            elsif ( rising_edge( clk_master ) ) then
					if ( cnt > x"0000" ) then
						cnt <= std_logic_vector( unsigned( cnt ) - 1 );
						out_scl_q := '1';
               else
               out_scl_q := '0';
					end if;
            end if; 
            scl_clk_q_tick <= out_scl_q;
        end process scl_sda_sync_signal_gen;
    
	    pro2 <= scl_clk_q_tick;
	 
    end block slave_frequency_counter;
	
	
	-- Signal Interface:
    -- Input
    -- -- (control) clk_master, nreset, scl_clk_q_tick. send_ack_en
    -- -- (data) sda_i, scl_i
    -- Output
    -- -- (status) start_cond_detect, stop_cond_detect, ack_sent
    bus_signal_control: block
    begin
    
        sta_sto_detect: start_stop_detect
            port map ( 
                clk      =>  clk_master,
                nreset   =>  nreset,
                sync_rst =>  '0',
                scl      =>  scl_i,
                sda      =>  sda_i,
                start    =>  ss_start_cond_detect,
                stop     =>  ss_stop_cond_detect
            );
    
            
        ack_gen: process( scl_clk_q_tick, cs_send_ack_en )
            variable done: STD_LOGIC := '0';  
        begin
            if ( cs_send_ack_en = '0' ) then
                done := '0';
            elsif ( falling_edge( scl_clk_q_tick ) ) then          
				done := '1';
            end if;
            
            ss_ack_sent <= done;
        end process ack_gen;
    
		ack_out <= cs_send_ack_en AND NOT ss_ack_sent;
	
    end block bus_signal_control;
	 
	 
	 
	-- Signal Interface:
    -- Input
    -- -- (control) clk_master, nreset, cs_data_in_acq_en
    -- -- (data) sda_i, scl_i
    -- Output
    -- -- (status) data_in_detect, data_in_ready
    -- -- (data) data_in
    data_in_generate: block
        signal data_acquired    : STD_LOGIC_VECTOR(7 downto 0);
        signal ack_acquired     : STD_LOGIC_VECTOR(0 downto 0);
        signal old_data         : STD_LOGIC;
        signal old_data1         : STD_LOGIC;
        signal reset            : STD_LOGIC;
        signal dready, ack_ready  : STD_LOGIC;
        signal en_ack_acq       : STD_LOGIC;
        signal v_sda_i, v_ack   : STD_LOGIC_VECTOR(1 downto 0); 
        signal ack_detect : STD_LOGIC;
    begin
    
		data_in_gen: serial2parallel
            generic map (
                DATA_WIDTH =>  8
            )
            port map ( 
                nreset    =>  cs_data_in_acq_en,
                da_en     =>  scl_i,
                data_in   =>  sda_i,
                data_out  =>  data_acquired,
                dstartrec =>  ss_data_in_detect,
                dready    =>  dready,
                old_data  =>  old_data
            );
            
        reset <= '0';
            
        reg_data_in: register_sr_load
            Generic Map (
                DATA_WIDTH    =>  8
            )
            Port Map (
                clk           =>  clk_master,
                reset         =>  reset,
                set           =>  ss_data_in_detect,
                load          =>  dready,
                def_set_value =>  x"00",
                D             =>  data_acquired,
                Q             =>  data_in
            );
            
        en_ack_acq <= cs_receive_ack_en AND scl_i;
        v_sda_i(0) <= sda_i;


        ack_in_gen: serial2parallel
            generic map (
				DATA_WIDTH =>  1
            )
            port map ( 
                nreset    =>  cs_receive_ack_en,
                da_en     =>  scl_i,
                data_in   =>  sda_i,
                data_out  =>  ack_acquired,
                dstartrec =>  ack_detect,
                dready    =>  ack_ready,
                old_data  =>  old_data1
            );

        ack_in <= ack_acquired(0);

		-- latch data ready with scl_clk_q_tick's falling edge
		process( scl_clk_q_tick )
		begin
			if ( falling_edge( scl_clk_q_tick ) ) then
                ss_data_in_ready <= dready;
                ss_ack_in_ready <= ack_ready;
			end if;
		end process; 
		             
		
	end block data_in_generate;
	 
	 
	 
	data_out_generate: block
		signal n_scl_clk_q_tick : STD_LOGIC;
	begin
        -- ack_out cs_sda_out_sel
    
        -- multiplexer
        with cs_sda_out_sel select
            sda_o <= '1'           when t_sda_out_idle,
                     ss_ack_sent   when t_sda_out_ack,
                     data_out      when t_sda_out_data,
                     '1'           when others;
        
        
		with cs_sda_out_sel select
            pro3 <= '1'           when t_sda_out_idle,
                     ss_ack_sent   when t_sda_out_ack,
                     '1'           when others;
					 
        sda_out <= sda_o;
        scl_out <= '1';

        
		
	n_scl_clk_q_tick <= not scl_clk_q_tick;	
	
    data_out_serializer: parallel2serial
    Generic Map (
        DATA_WIDTH   => 8
    )
    Port Map (
        enable     =>  cs_send_data_out,
        shift      =>  n_scl_clk_q_tick,
        data_in    =>  x"33",
        data_out   =>  data_out,
        dstartser  =>  ss_start_send_data_out,
        dfinishser =>  ss_done_send_data_out
    );

    end block data_out_generate;
  
    
	 
    fsm: block
        type states is ( 
			s_idle, 
            s_start,
            s_addr_cmd,
			s_data_receive, 
            s_send_ack,
            s_data_send,
            s_receive_ack,
            s_stop
		);
        signal i2c_state : states;     
        
        signal ctrl_enable_data_receive  : STD_LOGIC; -- enable the acquisition of a input data
        signal ctrl_enable_receive_ack   : STD_LOGIC; -- enable the acquisition of the ack/nac from the master
        signal ctrl_enable_send_ack      : STD_LOGIC; -- enable the block acts to send a ack/nack bit
        signal ctrl_enable_send_data     : STD_LOGIC; -- enable the sending of data out
        signal ctrl_select_sda_out       : sda_out_mux_sel := t_sda_out_idle;
        
    begin
    
        
        state_machine: process( clk_master, nreset )
--			variable data : STD_LOGIC_VECTOR(7 downto 0);
			variable add : STD_LOGIC_VECTOR(7 downto 0);
			variable cmd : STD_LOGIC;
            variable cnt_data : INTEGER := 0;
        begin
                  
        if ( nreset = '0' ) then
         
            i2c_state <= s_idle;
            
            ctrl_enable_data_receive <= '0';
            ctrl_enable_send_ack <= '0';
            ctrl_enable_send_data <= '0';
            ctrl_enable_receive_ack <= '0';
            ctrl_select_sda_out <= t_sda_out_idle;
            cnt_data := 0;
                     
        elsif ( rising_edge( clk_master ) ) then
        
            ctrl_enable_send_ack <= '0';
            ctrl_enable_data_receive <= '0';
            ctrl_enable_send_data <= '0';
        
            case i2c_state is
            
                when s_idle =>
                    -- manage next state
                    if ( ss_start_cond_detect = '1' ) then
                        i2c_state <= s_start;
                    elsif ( ss_stop_cond_detect = '1' ) then
                        i2c_state <= s_stop;
                    end if;
					
                    cnt_data := 0;
                    laddr <= '0';
                    lbyte1 <= '0';
                            lbyte2 <= '0';
                 
                when s_start =>
                    -- manage next state
                    ctrl_enable_data_receive <= '1';
                    if ( ss_data_in_detect = '1' ) then
                        i2c_state <= s_addr_cmd;
                    elsif ( ss_stop_cond_detect = '1' ) then
                        i2c_state <= s_stop;
                    end if;
					
                    laddr <= '0';
                    lbyte1 <= '0';
                            lbyte2 <= '0';
                        
                when s_addr_cmd =>
                    ctrl_enable_data_receive <= '1';

                    if ( ss_data_in_ready = '1' ) then
                        i2c_addr <= '0' & data_in(7 downto 1); 
                        i2c_cmd  <= data_in(0); 
						
						address <= i2c_addr; 
						
                        laddr <= '1';
                        lbyte1 <= '0';
                            lbyte2 <= '0';
                        
                        --if ( ( addr XOR x"2a" ) = x"00" ) then
                        -- manage next state
                        if ( ( '0' & data_in(7 downto 1) ) = x"2a"  ) then
                            i2c_state <= s_send_ack;
                        else
                            i2c_state <= s_idle;
                        end if;
                    end if;
                    
                when s_send_ack =>
                     if ( ss_ack_sent = '0' ) then   
                        ctrl_enable_send_ack <= '1';
                        ctrl_select_sda_out <= t_sda_out_ack;
                     else
                        ctrl_enable_send_ack <= '0';
                        ctrl_select_sda_out <= t_sda_out_idle;
                        if ( i2c_cmd = '0' ) then
                            i2c_state <= s_data_send;
                        else
                            i2c_state <= s_data_receive;
                        end if;
                     end if;   
					 
                     laddr <= '0';
                     
                            lbyte1 <= '0';
                            lbyte2 <= '0';
                     
                when s_data_receive =>
                    ctrl_enable_data_receive <= '1';

                    if ( ss_start_cond_detect = '1' ) then
                        i2c_state <= s_start;
                    elsif ( ss_stop_cond_detect = '1' ) then
                        i2c_state <= s_stop;
                    elsif ( ss_data_in_ready = '1' ) then
                        cnt_data := cnt_data + 1;
                        if ( cnt_data = 1 ) then
                            byte1 <= data_in;
                            laddr <= '0';
                            lbyte1 <= '1';
                            lbyte2 <= '0';
                        else
                            byte2 <= data_in;
                            laddr <= '0';
                            lbyte1 <= '0';
                            lbyte2 <= '1';
                        end if;

                        i2c_data <= data_in; 
                        i2c_state <= s_send_ack;
                        
                    end if;

                when s_data_send =>
                    ctrl_select_sda_out <= t_sda_out_data;
                    if ( ss_done_send_data_out = '0' ) then
                        ctrl_enable_send_data <= '1';
                    else
						ctrl_enable_send_data <= '0';
                        i2c_state <= s_receive_ack;
                    end if;


                when s_receive_ack =>
                    ctrl_enable_receive_ack <= '1';
                    if ( ss_ack_in_ready = '1' ) then
                        if ( ack_in = I2C_ACK ) then
                            i2c_state <= s_data_receive;
                        else
                            i2c_state <= s_idle;
                        end if;
                    end if;

                when s_stop =>
                    ctrl_select_sda_out <= t_sda_out_idle;
                     i2c_state <= s_idle;
					 laddr <= '0';
                    lbyte1 <= '0';
                     lbyte2 <= '0';
                    
            end case;

        end if;
    end process state_machine;
    
    
    cs_data_in_acq_en <= ctrl_enable_data_receive;
    cs_send_ack_en    <= ctrl_enable_send_ack;
    cs_send_data_out  <= ctrl_enable_send_data;
    cs_receive_ack_en <= ctrl_enable_receive_ack;

    cs_sda_out_sel    <= ctrl_select_sda_out;

    end block fsm;
            

end Behavioral;
