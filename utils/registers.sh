   
set_reg()
{
  reg=$((0x1000 + ($1*4)))
  axireg $reg $2  
}

get_reg()
{
  reg=$((0x1000 + ($1*4)))
  axireg $reg  
}


   SREG_MODULE_REV=0  
SREG_PATTERN_WIDTH=1  
    SREG_FIFO_STAT=2  
     SREG_F0_COUNT=3  
     SREG_F1_COUNT=4  
  SREG_ACTIVE_FIFO=5
     CREG_FIFO_CTL=7
      CREG_UPPER32=8
      CREG_LOAD_F0=9 
      CREG_LOAD_F1=10  
        CREG_START=11            
    CREG_HARD_STOP=12 

