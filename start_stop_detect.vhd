
library IEEE;
use IEEE.STD_LOGIC_1164.ALL;


entity start_stop_detect is
    Port ( 
        clk      : in  STD_LOGIC;
        nreset   : in  STD_LOGIC;
        sync_rst : in  STD_LOGIC;
        scl      : in  STD_LOGIC;
        sda      : in  STD_LOGIC;
        start    : out STD_LOGIC;
        stop     : out STD_LOGIC
    );
end start_stop_detect;


architecture Behavioral of start_stop_detect is
    signal scl_array : STD_LOGIC_VECTOR(0 to 1);    -- position 1: most recent acquisition ; position 0: past acquisition
    signal sda_array : STD_LOGIC_VECTOR(0 to 1);    -- position 1: most recent acquisition ; position 0: past acquisition
    signal sta, sto  : STD_LOGIC := '0';
begin


    store: process( clk, nreset )
    begin
    if ( nreset = '0' ) then
        
            scl_array <= "00";
            sda_array <= "00";
                    
        elsif ( rising_edge( clk ) ) then
        
            if ( sync_rst = '1' ) then
               
                scl_array <= "00";
                sda_array <= "00";
            
            else             
            
                scl_array <= scl_array(1) & scl;
                sda_array <= sda_array(1) & sda;   
            
            end if;
            
        end if;
    end process store;


    detect: process( clk, nreset )
    begin
        if ( nreset = '0' ) then
        
            sta <= '0';
            sto <= '0';
                    
        elsif ( rising_edge( clk ) ) then
        
            if ( sync_rst = '1' ) then
                
                sta <= '0';
                sto <= '0';
            
            else             
               
                sta <= ( not sda_array(1) and sda_array(0) ) and ( scl_array(1) and scl_array(0) );
                sto <= ( sda_array(1) and not sda_array(0) ) and ( scl_array(1) and scl_array(0) );
                
            end if;
            
        end if;
    end process detect;
    
    
    start <= sta;
    stop <= sto;

end Behavioral;
