library ieee ;
use ieee.std_logic_1164.all ;
use ieee.numeric_std.all ;

entity overall_design is
    port (
        clock : in std_logic;
        push_button : in std_logic; -- starts filtering the image on pressing the push button
        switch : in std_logic  -- switch =0 means we want the smoothening filter, switvh =1 means we want the sharpening effect

        -- No outputs, we will directly write the pixels in the RAM alllocated to us.
    );
end overall_design ; 

architecture Artix of overall_design is

    component RAM_64Kx8 is
        port (
            clock : in std_logic;
            read_enable, write_enable : in std_logic; -- signals that enable read/write operation
            address : in std_logic_vector(15 downto 0); -- 2^16 = 64K
            data_in : in std_logic_vector(7 downto 0);
            data_out : out std_logic_vector(7 downto 0)
        );
    end component;

    component ROM_32x9 is
        port (
            clock : in std_logic;
            read_enable : in std_logic; -- signal that enables read operation
            address : in std_logic_vector(4 downto 0); -- 2^5 = 32
            data_out : out std_logic_vector(8 downto 0) -- because the coefficients are all 9 bit signed integers
        );
    end component;

    component MAC is
        port (
            clock : in std_logic;
            control : in std_logic; -- ‘0’ for initializing the sum
            data_in1, data_in2 : in std_logic_vector(17 downto 0);
            data_out : out std_logic_vector(17 downto 0)
        );
    end component;

    type state_type is (s0,s1,s2);  -- three states have been made in the FSM.
    signal state : state_type := s0;    -- gonna store the current of our FSM.

    signal start : std_logic:='0';   -- The value of this is made 1 by pressing the push button, start=1 if we want to start our image filter.
    signal mode : std_logic;    -- This will store the mode (smooth or sharp) and will change only when the filtering is finsihed.
                                -- So the use may change it's value even when the process is going on and it won't affect the current process.

    signal ii : integer:=1; -- This will store the row no. of the X matrix (the matrix containg the pixels of the input image)
    signal jj : integer:=1; -- This will store the column no. of the X matrix (the matrix containg the pixels of the input image)
    signal i : integer:=-1; -- This will store the row no. of the coefficient matrix
    signal j : integer:=-1; -- This will store the column no. of the coefficient matrix

    signal counter : integer:=0;    -- A counter that will be used to keep track of position in the coefficient matrix. 

    signal read_enable : std_logic :='0';   -- This is enabled during State1 when we are calculating the pixel value of the output image, this is disabled during the state2 when we are writing in the RAM
    signal write_enable : std_logic :='0';  -- This is enabled when we are writing in the RAM to store that pixel, this is disbled during state1 since we are rreading at that time.
    -- NOTE :- we defined 2 separate states for it because we can't write and read from the RAM at the same time since it may cause some clash.

    signal x_addr : integer:=0; -- This will store the relative address of a cell of input image matrix  (relative means the position if we start counting fromt the upper half)
    signal y_addr : integer:=0; -- This will store the realtive address of a cell of output image matrix (relative means the position if we start counting fromt the lower half)
    
    signal x_addr_bin : std_logic_vector(15 downto 0) ; -- 16 bit binary form of x_addr
    signal y_addr_bin : std_logic_vector(15 downto 0) ; -- 16 bit binary form of y_addr
    signal c_addr_bin : std_logic_vector(4 downto 0) ;  -- 5 bit binary form to store adress of coefficient which needs to be called.


    signal x_pixel : std_logic_vector(7 downto 0) ; -- 8 bit unsigned form to store the pixel of the original image 
    signal x_pixel_18 : std_logic_vector(18 downto 0) ; -- just the extendedd version of x_pixel which is 18 bit and in signed form
    signal y_pixel : std_logic_vector(17 downto 0) ;    -- 18 bit signed integer storing the value of pixel of output image
    signal c: std_logic_vector(8 downto 0) ;    --  stores value of the coefficient, in signed form
    signal c_18 : std_logic_vector(18 downto 0) ; -- extended version of c to 18 bits in signed notation
    signal p: std_logic_vector(17 downto 0) ;   -- this will store the product values we will get from MAC (ie will store X*C).
        
    constant m : integer := 120;    -- rows in X are 120
    constant n : integer := 160;    -- colums in Y are 160


begin

    process (switch, start) begin
        if (start = '0')    
            mode <= switch; -- switch may change whenever but mode will change only when the process is not going on should 
        end if;
    end process ;

    process ( clock , push_button) begin
        if (rising_edge(clock)) then
            if (rising_edge(push_button)) then
                start <= '1';   -- Now we will start the filtering process
            end if ;
        end if ;     
    end process ;

    process (clock, start) begin
        if start ='0' then
            state <= s0;    -- s0 means we are in off mode.
        elsif start = '1' then
            state <= s1;
            ii <= 1;    -- Inintialising the values 
            jj <= 1;
            i <= -1;
            j <= -1;
            counter <=0;
        end if;
    end process;

    x_addr <= n*(ii+i) + jj+j;  -- relative position in upper half
    x_addr_bin <= '0'&std_logic_vector(to_unsigned(x_addr,15)); -- exact posiiton in the RAM, since in upper half, the MSB is made 0.
    x_pixel_18 <= std_logic_vector(to_signed(to_integer(unsigned(x_pixel)),18));  -- 18 bit signed version of x_pixel

    y_addr <= (n-2)*(ii) +jj;   -- relative position in the lower half 
    y_addr_bin <= '1'&std_logic_vector(to_unsigned(y_addr,15)) ;    -- since in the lower half, MSB is 1 and remaing contains the relative position

    c_addr_bin <= mode&std_logic_vector(to_unsigned(counter,4)) ;   -- if mode =0 then smooth and coefficients of smooth are stored in the upper half, hence MSB is 0
                                                                    -- if mode =1 then sharp and coefficients of sharp are stored in the lower half, hence MSB is 1
    c_18 <= std_logic_vector(to_signed(to_integer(signed(c)),18));  -- 18 bit signed version

    get_pixel : RAM_64Kx8 port map (clock =>clock, read_enable => read_enable , write_enable=> '0' , address => x_addr_bin , data_in => y_pixel(14 downto 7) , data_out => x_pixel );
                                                                            --  Only reading from the x_addr                                                   we will get the pixel value as a binary 
    write_pixel : RAM_64Kx8 port map (clock =>clock, read_enable => '0' , write_enable=> write_enable; , address => y_addr_bin , data_in => y_pixel(14 downto 7) , data_out => x_pixel );
                                                                       -- Only writing is enabled on y-addr                      getting the 8 bits from 18 bit y_pixel
    get_coefficient : ROM_32x9 port map (clock => clock , read_enable =>'1', address => c_addr_bin, data_out => c );
                                                                                                             -- c will store the value of coefficient we will use
    get_product : MAC port map (clock => clock, control => '0' , data_in1 => x_pixel_18 , data_in2 => c_18, data_out => p);
                                                                        --   Passing 18 bit signed X and C              P will store the product value

    process (ii,jj,i,j, state) begin 
        case( state ) is
            when s1 =>
                write_enable <= '0';
                read_enable <= '1';
                y_pixel <= y_pixel + p; -- writing the partial product in the y_pixel
                counter <= counter +1;  -- updating the counter/

                if i=2 then    -- means we have iterated the coefficient matrix
                    state <= s2;    -- so now start the storing part
                else
                    if j=2 then     -- that row has been finished
                        i =i+1; 
                        j =-1;
                    else
                        j <= j+1;   
                    end if ;

                end if;
            
            when s2 =>
                read_enable ='0';
                write_enable = '1';
                if (y_pixel(17)='1') then 
                    y_pixel <= std_logic_vector(to_signed(0,18)) ;  -- making it zero, since pixel value can't be negative. 
                end if;
                if ii=m-1 then
                    start <= 0;
                    state <= s0;    -- since whole process has finished
                else
                    if jj =n-1 then -- only one row has finished and we gotta move to the next row
                        ii <= ii+1;
                        jj <= 1;
                    else
                        jj = jj+1;
                    end if;
                    counter <= 0;   -- reset the counter
                    state <= s1;    -- jump to state 1.
                end if;

            when others => 
                -- Nothing to do here.
        end case ;

    end process;

end architecture ;


-- Entitites RAM, ROM, MAC which are already given

library IEEE;
use IEEE.std_logic_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity RAM_64Kx8 is
    port (
        clock : in std_logic;
        read_enable, write_enable : in std_logic; -- signals that enable read/write operation
        address : in std_logic_vector(15 downto 0); -- 2^16 = 64K
        data_in : in std_logic_vector(7 downto 0);
        data_out : out std_logic_vector(7 downto 0)
    );
end RAM_64Kx8;
architecture Artix of RAM_64Kx8 is
        type Memory_type is array (0 to 65535) of std_logic_vector (7 downto 0);
        signal Memory_array : Memory_type;
    begin
        process (clock) begin
            if rising_edge (clock) then
                if (read_enable = '1') then -- the data read is available after the clock edge
                    data_out <= Memory_array (to_integer (unsigned (address)));
                end if;
                if (write_enable = '1') then -- the data is written on the clock edge
                    Memory_array (to_integer (unsigned(address))) <= data_in;
                end if;
            end if;
        end process;
end Artix;

entity ROM_32x9 is
    port (
        clock : in std_logic;
        read_enable : in std_logic; -- signal that enables read operation
        address : in std_logic_vector(4 downto 0); -- 2^5 = 32
        data_out : out std_logic_vector(8 downto 0) -- because the output of the cofficients are all 9 bit unsigned integers
    );
end ROM_32x9;
architecture Artix of ROM_32x9 is
        type Memory_type is array (0 to 31) of std_logic_vector (8 downto 0);
        signal Memory_array : Memory_type;
    begin
        process (clock) begin
            if rising_edge (clock) then
                if (read_enable = '1') then -- the data read is available after the clock edge
                    data_out <= Memory_array (to_integer (unsigned (address)));
                end if;
            end if;
        end process;
end Artix;

entity MAC is
    port (
        clock : in std_logic;
        control : in std_logic; -- ‘0’ for initializing the sum
        data_in1, data_in2 : in std_logic_vector(17 downto 0);
        data_out : out std_logic_vector(17 downto 0)
    );
end MAC;
architecture Artix of MAC is
        signal sum, product : signed (17 downto 0);
    begin
        data_out <= std_logic_vector (sum);
        product <= signed (data_in1) * signed (data_in2)
        process (clock) begin
            if rising_edge (clock) then -- sum is available after clock edge
                if (control = '0') then -- initialize the sum with the first product
                    sum <= std_logic_vector (product);
                else -- add product to the previous sum
                    sum <= std_logic_vector (product + signed (sum));
                end if;
            end if;
        end process;
end Artix;