#include "Nextion.h"
#include "util/Utilities.h"
#include <string.h>
#include <stdlib.h>

#include "task/minitask.h"
#include "debug_helper.h"

struct NexObject button, slider;

struct NexObject *nex_listen_list[] = {
    &button,
    &slider};

uint32_t value;

#define buttonPageID 0
#define sliderPageID 0
#define buttonID 1
#define sliderID 5

void buttonCallback(void *ptr);
void sliderCallback(void *ptr);

void task_init_ser_in(void)
{
    CreateNexObject(button, buttonPageID, buttonID, "b1");
    CreateNexObject(slider, sliderPageID, sliderID, "h5");

    nexInit();

    NexSlider_getValue(&slider, &value);
    NexSlider_setValue(&slider, value + 20);

    NexTouch_attachPop(&button, buttonCallback, 0);
    NexTouch_attachPop(&slider, sliderCallback, 0);
}

void task_handle_serin(uint32_t arg)
{
    nexLoop(nex_listen_list);
}

void buttonCallback(void *ptr)
{
    DEBUG_PRINTF("On Button\n");
}

void sliderCallback(void *ptr)
{
    DEBUG_PRINTF("On Slider\n");
}