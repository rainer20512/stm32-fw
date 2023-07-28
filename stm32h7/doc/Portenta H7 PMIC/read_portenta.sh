
#!/bin/bash
# this script will run the first time the raspberry pi boots.
# it runs as root.
set -xv
echo '>>> reading PMIC registers...'
i2cget -y 1 8 0x4f
i2cget -y 1 8 0x4c
i2cget -y 1 8 0x4d 
i2cget -y 1 8 0x52 
i2cget -y 1 8 0x53 
i2cget -y 1 8 0x58 
sleep 1
i2cget -y 1 8 0x9c 
i2cget -y 1 8 0x9e 
sleep 1
i2cget -y 1 8 0x42 
sleep 1
i2cget -y 1 8 0x94 
i2cget -y 1 8 0x38 
i2cget -y 1 8 0x39 
i2cget -y 1 8 0x3a 
i2cget -y 1 8 0x3b 
i2cget -y 1 8 0x32 
i2cget -y 1 8 0x33 
i2cget -y 1 8 0x34 
i2cget -y 1 8 0x35 
i2cget -y 1 8 0x3e 
i2cget -y 1 8 0x3f 
i2cget -y 1 8 0x40 
i2cget -y 1 8 0x41 
sleep 1
# has to be the last one - will hang i2c
i2cget -y 1 8 0x50 

