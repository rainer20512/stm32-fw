#include "../lv_examples.h"
#if LV_BUILD_EXAMPLES && LV_USE_LABEL

/**
 * Basic example to create a "Hello world" label
 */

#include <stdio.h>

#define TEXTLEN  20
static char text[TEXTLEN];
static void draw_label(lv_obj_t * label, int32_t x, int32_t y )
{
    snprintf(text, TEXTLEN, "%d,%d", x, y );
    lv_label_set_text(label, text);
    lv_obj_align(label, LV_ALIGN_CENTER, x, y);


}

static void anim_text_cb(void * var, int32_t v)
{
    lv_obj_t * label = (lv_obj_t *) var;
    snprintf(text, TEXTLEN, "%d", v );
    lv_label_set_text(label, text);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
}


void lv_example_get_started_1(void)
{
    /*Change the active screen's background color*/
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003a57), LV_PART_MAIN);
    lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff), LV_PART_MAIN);

    /*Create a white label, set its text and align it to the center*/
    lv_obj_t * label = lv_label_create(lv_screen_active());
    draw_label(label, 0,0 );

    /*Create a white label, set its text and align it to the center*/
    lv_obj_t * l2 = lv_label_create(lv_screen_active());
    draw_label(l2, -60,0 );

    /*Create a white label, set its text and align it to the center*/
    lv_obj_t * l3 = lv_label_create(lv_screen_active());
    draw_label(l3, 60,0 );

    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, label);
    lv_anim_set_values(&a, 0, 100);
    lv_anim_set_duration(&a, 2000);
    lv_anim_set_reverse_delay(&a, 10);
    lv_anim_set_reverse_duration(&a, 3000);
    lv_anim_set_repeat_delay(&a, 50);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    // lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_set_exec_cb(&a, anim_text_cb);
    lv_anim_start(&a);
}


#endif
