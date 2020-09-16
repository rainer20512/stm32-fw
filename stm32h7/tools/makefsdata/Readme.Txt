HowTo use MakeFsData and it's output                               RHB 15.09.20
-------------------------------------------------------------------------------
0. the "make.cmd" file is coded with fiexed input and output directories
   - it will convert the whole "static" subdir of this directory
   - it will put the generated "fsdata_custom.c" to ..\..\common_h7\lwip\
   - after generation a postprocessing is required, see 1.   

1. After generation of fsdata_custom.c replace the first three 
   lines with #include#s with the following two
     #include "lwip/apps/fs.h"
     #include "lwip/def.h"

2. content of js/only_chng.js
 This js code fragment ensures that only modified input fields
 are resend to server. to enable, do the following
   a) tag every input field with onchange="add(this)"
   b) tag form with onSubmit="before_submit()"
