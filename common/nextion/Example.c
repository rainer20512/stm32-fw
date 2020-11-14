#include "Nextion.h"
#include "util/Utilities.h"
#include <string.h>
#include <stdlib.h>

#include "task/minitask.h"
#include "debug_helper.h"

struct NexObject button, slider, txt0;

struct NexObject *nex_listen_list[] = {
    &button,
    &slider};

uint32_t value;

#define buttonPageID 0
#define sliderPageID 0
#define buttonID 4
#define sliderID 5
#define txt0ID   2

void buttonCallback(void *ptr);
void sliderCallback(void *ptr);

void task_init_ser_in(void)
{
    CreateNexObject(button, buttonPageID, buttonID, "b1");
    CreateNexObject(slider, sliderPageID, sliderID, "h5");
    CreateNexObject(txt0,   sliderPageID, txt0ID, "t0");
    

    nexInit();
    nexDelay(50);

    NexSlider_getValue(&slider, &value);
    NexSlider_setValue(&slider, value + 20);

    NexTouch_attachPop(&button, buttonCallback, 0);
    NexTouch_attachPush(&button, buttonCallback, 1);
    NexTouch_attachPop(&slider, sliderCallback, 0);
}

void task_handle_serin(uint32_t arg)
{
    nexLoop(nex_listen_list);
}

void buttonCallback(void *ptr)
{
    uint32_t arg = ( uint32_t)ptr;
    if ( arg )
        DEBUG_PRINTF("On Button Push\n");
    else
        DEBUG_PRINTF("On Button Pop\n");
}

void sliderCallback(void *ptr)
{
    char buf[10];
    DEBUG_PRINTF("On Slider\n");
    NexSlider_getValue(&slider, &value);
    snprintf(buf, 10,"%03d",value);
    NexText_setText(&txt0, buf);
}