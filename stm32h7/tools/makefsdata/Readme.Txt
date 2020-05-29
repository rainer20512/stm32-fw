1. After generation of fs_data_customc. replace the first three 
   lines with #include#s with the following two
     #include "lwip/apps/fs.h"
     #include "lwip/def.h"

2. content of js/only_chng.js
 This js code fragment ensures that only modified input fields
 are resend to server. to enable, do the following
   a) tag every input field with onchange="add(this)"
   b) tag form with onSubmit="before_submit()"
