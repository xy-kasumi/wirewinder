#include <stdio.h>
#include "LCD_1in3.h"
#include "lvgl/lvgl.h"
#include "pico/stdlib.h"

// See https://www.waveshare.com/wiki/Pico-LCD-1.3
#define DISP_WIDTH 240
#define DISP_HEIGHT 240

#define PIN_STICK_UP 2
#define PIN_STICK_DOWN 18
#define PIN_STICK_LEFT 16
#define PIN_STICK_RIGHT 20
#define PIN_STICK_PRESS 3
#define PIN_BTN_A 15
#define PIN_BTN_B 17
#define PIN_BTN_X 19
#define PIN_BTN_Y 21

typedef struct {
  uint32_t key;
  int pin;
} key_map_entry_t;

static key_map_entry_t key_map[] = {
    {LV_KEY_PREV, PIN_STICK_UP},     {LV_KEY_NEXT, PIN_STICK_DOWN},
    //{LV_KEY_LEFT, PIN_STICK_LEFT}, {LV_KEY_RIGHT, PIN_STICK_RIGHT},
    {LV_KEY_ENTER, PIN_BTN_A},
};

#define NUM_KEYS (sizeof(key_map) / sizeof(key_map_entry_t))

static bool last_key_state[NUM_KEYS];

uint32_t ms_since_boot() {
  return (uint32_t)(to_us_since_boot(get_absolute_time()) / 1000L);
}

/* Copy rendered image to screen.
 * area: both ends inclusive
 */
void flush_display(lv_display_t* disp, const lv_area_t* area, uint8_t* px_buf) {
  LCD_1IN3_DisplayWindowsPartial(area->x1, area->y1, area->x2 + 1, area->y2 + 1,
                                 (uint16_t*)px_buf);
  lv_display_flush_ready(disp);
}

static void indev_read_cb(lv_indev_t* indev, lv_indev_data_t* data) {
  // Stabilize chattering and detect first pressed key
  bool pressed = false;
  uint32_t pressed_key = 0;
  for (int i = 0; i < NUM_KEYS; i++) {
    bool curr = gpio_get(key_map[i].pin);
    bool prev = last_key_state[i];
    last_key_state[i] = curr;

    if (curr != prev) {
      // unstable; treat as no-data
      continue;
    }
    // low = pressed
    if (!curr) {
      pressed = true;
      pressed_key = key_map[i].key;
      break;
    }
  }

  if (pressed) {
    data->key = pressed_key;
    data->state = LV_INDEV_STATE_PRESSED;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

static lv_indev_t* init_gui() {
  // Initialize display
  DEV_Module_Init();
  DEV_SET_PWM(50);
  LCD_1IN3_Init(HORIZONTAL);
  LCD_1IN3_Clear(0x8410);  // gray

  // Initialize buttons
  for (int i = 0; i < NUM_KEYS; i++) {
    int pin = key_map[i].pin;
    gpio_init(pin);
    gpio_pull_up(pin);
  }

  // Initialize LVGL
  lv_init();
  lv_tick_set_cb(ms_since_boot);

  lv_display_t* display = lv_display_create(DISP_WIDTH, DISP_HEIGHT);
  static uint16_t buf[DISP_WIDTH * DISP_HEIGHT / 10];
  lv_display_set_buffers(display, buf, NULL, sizeof(buf),
                         LV_DISPLAY_RENDER_MODE_PARTIAL);
  lv_display_set_flush_cb(display, flush_display);

  lv_indev_t* indev = lv_indev_create();
  lv_indev_set_type(indev, LV_INDEV_TYPE_KEYPAD);
  lv_indev_set_read_cb(indev, indev_read_cb);
  return indev;
}

void btn_event_cb(lv_event_t* e) {
  lv_event_code_t ecode = lv_event_get_code(e);
  if (ecode != LV_EVENT_CLICKED) {
    return;
  }

  // TODO: do something
}

int main() {
  lv_indev_t* indev = init_gui();

  // Setup widgets
  lv_group_t* g = lv_group_create();
  lv_group_set_default(g);

  lv_obj_t* root = lv_obj_create(lv_screen_active());
  //lv_gridnav_add(root, LV_GRIDNAV_CTRL_NONE);
  lv_indev_set_group(indev, g);

  lv_obj_set_flex_flow(root, LV_FLEX_FLOW_ROW_WRAP);
  lv_obj_set_style_bg_color(root, lv_palette_lighten(LV_PALETTE_BLUE, 5), LV_STATE_FOCUSED);
  // lv_group_add_obj(g, root);

  {
    lv_obj_t* btn = lv_button_create(root);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);
    //lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
    //lv_group_remove_obj(btn);

    lv_obj_t* btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Start");
    lv_obj_center(btn_label);
  }

  {
    lv_obj_t* btn = lv_button_create(root);
    lv_obj_add_event_cb(btn, btn_event_cb, LV_EVENT_ALL, NULL);
    //lv_obj_add_flag(btn, LV_OBJ_FLAG_CHECKABLE);
    //lv_group_remove_obj(btn);

    lv_obj_t* btn_label = lv_label_create(btn);
    lv_label_set_text(btn_label, "Stop");
    lv_obj_center(btn_label);
  }

  /* Make LVGL periodically execute its tasks */
  while (1) {
    lv_timer_handler();
    sleep_ms(5);
  }
}
