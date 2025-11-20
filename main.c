#include <stdio.h>
#include "LCD_1in3.h"
#include "lvgl/lvgl.h"
#include "pico/stdlib.h"

#define DISP_WIDTH 240
#define DISP_HEIGHT 240

uint32_t ms_since_boot() {
  return (uint32_t)(to_us_since_boot(get_absolute_time()) / 1000L);
}

/* Copy rendered image to screen.
 * area: both ends inclusive
 */
void my_flush_cb(lv_display_t* disp, const lv_area_t* area, uint8_t* px_buf) {
  LCD_1IN3_DisplayWindowsPartial(area->x1, area->y1, area->x2 + 1, area->y2 + 1,
                                 (uint16_t*)px_buf);
  /*
  uint16_t* buf16 =
      (uint16_t*)px_buf;
  for (int y = area->y1; y <= area->y2; y++) {
    for (int x = area->x1; x <= area->x2; x++) {
      // put_px(x, y, *buf16);
      LCD_1IN3_DisplayPoint(x, y, *buf16);

      buf16++;
    }
  }
  */

  lv_display_flush_ready(disp);
}

int main() {
  DEV_Module_Init();
  DEV_SET_PWM(50);
  LCD_1IN3_Init(HORIZONTAL);
  LCD_1IN3_Clear(0x8410);  // gray

  lv_init();

  lv_tick_set_cb(ms_since_boot);

  lv_display_t* display = lv_display_create(DISP_WIDTH, DISP_HEIGHT);

  static uint16_t buf[DISP_WIDTH * DISP_HEIGHT / 10];
  lv_display_set_buffers(display, buf, NULL, sizeof(buf),
                         LV_DISPLAY_RENDER_MODE_PARTIAL);

  /* This callback will display the rendered image */
  lv_display_set_flush_cb(display, my_flush_cb);

  /* Create widgets */
  // lv_obj_t* label = lv_label_create(lv_screen_active());
  // lv_label_set_text(label, "Hello LVGL!");

  /*Change the active screen's background color*/
  lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x003a57),
                            LV_PART_MAIN);

  /*Create a white label, set its text and align it to the center*/
  lv_obj_t* label = lv_label_create(lv_screen_active());
  lv_label_set_text(label, "Hello world");
  lv_obj_set_style_text_color(lv_screen_active(), lv_color_hex(0xffffff),
                              LV_PART_MAIN);
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

  /* Make LVGL periodically execute its tasks */
  while (1) {
    lv_timer_handler();
    sleep_ms(5);
  }
}
