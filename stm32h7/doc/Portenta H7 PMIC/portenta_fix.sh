
#!/bin/bash
# this script will run the first time the raspberry pi boots.
# it runs as root.

echo '>>> Starting Portenta PMIC repair'
set -xv
i2cset -y 1 8 0x4f 0x00
i2cset -y 1 8 0x4c 0x05
i2cset -y 1 8 0x4d 0x03
# i2cset -y 1 8 0x4d 0x0f
i2cset -y 1 8 0x52 0x09
i2cset -y 1 8 0x53 0x0f
i2cset -y 1 8 0x58 0x01
# i2cset -y 1 8 0x58 0x03
sleep 1
i2cset -y 1 8 0x9c 0x80
i2cset -y 1 8 0x9e 0x20
sleep 1
i2cset -y 1 8 0x42 0x02
sleep 1
i2cset -y 1 8 0x94 0xa0  
i2cset -y 1 8 0x38 0x06  
# i2cset -y 1 8 0x38 0x07  
i2cset -y 1 8 0x39 0x06  
# i2cset -y 1 8 0x39 0x05  
i2cset -y 1 8 0x3a 0x06  
# i2cset -y 1 8 0x3a 0x05  
i2cset -y 1 8 0x3b 0x0f  
i2cset -y 1 8 0x32 0x06  
# i2cset -y 1 8 0x32 0x07  
i2cset -y 1 8 0x33 0x06  
# i2cset -y 1 8 0x33 0x05  
i2cset -y 1 8 0x34 0x06  
# i2cset -y 1 8 0x34 0x05  
i2cset -y 1 8 0x35 0x0f  
i2cset -y 1 8 0x3e 0x0d  
# i2cset -y 1 8 0x3e 0x0f  
i2cset -y 1 8 0x3f 0x0d 
# i2cset -y 1 8 0x3f 0x0e 
i2cset -y 1 8 0x40 0x0d 
# ic2set -y 1 8 0x40 0x0e  
i2cset -y 1 8 0x41 0x03
# i2cset -y 1 8 0x41 0x0f
sleep 1
# has to be the last one - will hang i2c
i2cset -y 1 8 0x50 0x0f 

