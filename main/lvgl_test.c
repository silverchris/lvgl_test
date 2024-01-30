#include "freertos/FreeRTOS.h"
#include <stdio.h>
#include <esp_lcd_panel_rgb.h>
#include <esp_lcd_panel_ops.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include <esp_log.h>
#include <freertos/task.h>
#include "esp_lcd_panel_io_additions.h"
#include "esp_lcd_st7701.h"
#include <lvgl/lvgl.h>
#include <esp_timer.h>
#include <driver/ledc.h>
#include <freertos/semphr.h>

const char *TAG = "Test";

static const st7701_lcd_init_cmd_t buyadisplaycom[] = {
        {0xFF, (uint8_t[]) {0x77, 0x01, 0x00, 0x00, 0x10,},                                                 5,  0},
        {0xC0, (uint8_t[]) {0x3b, 0x00,},                                                                   2,  0},
        {0xC1, (uint8_t[]) {0x0b, 0x02,},                                                                   2,  0},
        {0xC2, (uint8_t[]) {0x07, 0x02,},                                                                   2,  0},
        {0xCC, (uint8_t[]) {0x10,},                                                                         1,  0},
        {0xCD, (uint8_t[]) {0x08,},                                                                         1,  0},
        {0xB0, (uint8_t[]) {0x00, 0x11, 0x16, 0x0E, 0x11, 0x06, 0x05, 0x09, 0x08, 0x21, 0x06, 0x13, 0x10, 0x29, 0x31,
                            0x18},                                                                          16, 0},
        {0xB1, (uint8_t[]) {0x00, 0x11, 0x16, 0x0E, 0x11, 0x07, 0x05, 0x09, 0x09, 0x21, 0x05, 0x13, 0x11, 0x2A, 0x31,
                            0x18},                                                                          16, 0},
        {0xFF, (uint8_t[]) {0x77, 0x01, 0x00, 0x00, 0x11,},                                                 5,  0},
        {0xB0, (uint8_t[]) {0x6D},                                                                          1,  0},
        {0xB1, (uint8_t[]) {0x37},                                                                          1,  0},
        {0xB2, (uint8_t[]) {0x81},                                                                          1,  0},
        {0xB3, (uint8_t[]) {0x80},                                                                          1,  0},
        {0xB5, (uint8_t[]) {0x43},                                                                          1,  0},
        {0xB7, (uint8_t[]) {0x85},                                                                          1,  0},
        {0xB8, (uint8_t[]) {0x20},                                                                          1,  0},
        {0xC1, (uint8_t[]) {0x78},                                                                          1,  0},
        {0xC2, (uint8_t[]) {0x78},                                                                          1,  0},

        {0xC3, (uint8_t[]) {0x8C},                                                                          1,  0},
        {0xD0, (uint8_t[]) {0x88},                                                                          1,  0},
        {0xE0, (uint8_t[]) {0x00, 0x00, 0x02},                                                              3,  0},
        {0xE1, (uint8_t[]) {0x03, 0xA0, 0x00, 0x00, 0x04},                                                  5,  0},
        {0xA0, (uint8_t[]) {0x00, 0x00, 0x00, 0x20, 0x20},                                                  5,  0},
        {0xE2, (uint8_t[]) {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,}, 13, 0},
        {0xE3, (uint8_t[]) {0x00, 0x00, 0x11, 0x00},                                                        4,  0},
        {0xE4, (uint8_t[]) {0x22, 0x00},                                                                    2,  0},
        {0xE5, (uint8_t[]) {0x05, 0xEC, 0xA0, 0xA0, 0x07, 0xEE, 0xA0, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00},                                                                          16, 0},
        {0xE6, (uint8_t[]) {0x00, 0x00, 0x11, 0x00},                                                        4,  0},
        {0xE7, (uint8_t[]) {0x22, 0x00},                                                                    2,  0},
        {0xE8, (uint8_t[]) {0x06, 0xED, 0xA0, 0xA0, 0x08, 0xEF, 0xA0, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                            0x00},                                                                          16, 0},
        {0xEB, (uint8_t[]) {0x00, 0x00, 0x40, 0x40, 0x00, 0x00, 0x00},                                      7,  0},
        {0xED, (uint8_t[]) {0xFF, 0xFF, 0xFF, 0xBA, 0x0A, 0xBF, 0x45, 0xFF, 0xFF, 0x54, 0xFB, 0xA0, 0xAB, 0xFF,
                            0xFF},                                                                          15, 0},
        {0xEF, (uint8_t[]) {0x10, 0x0D, 0x04, 0x08, 0x3F, 0x1F},                                            6,  0},
        {0xFF, (uint8_t[]) {0x77, 0x01, 0x00, 0x00, 0x13},                                                  5,  0},
        {0xEF, (uint8_t[]) {0x08},                                                                          1,  0},
        {0xFF, (uint8_t[]) {0x77, 0x01, 0x00, 0x00, 0x00},                                                  5,  0},
        {0x11, (uint8_t[]) {},                                                                              0,  120},
        {0x29, (uint8_t[]) {},                                                                              0,  0},
//        {0x23, (uint8_t[]) {},                                                                              0,  0},

};

static lv_disp_t *disp;
static TaskHandle_t lvgl_task;
static esp_timer_handle_t lvgl_tick_timer;
static uint8_t *buf1;
static uint8_t *buf2;
static esp_lcd_panel_io_handle_t io_handle;
static esp_lcd_panel_handle_t panel_handle;
static SemaphoreHandle_t lvgl_mux = NULL;
//static lv_disp_draw_buf_t disp_buf;
//static lv_disp_drv_t disp_drv;



bool lvgl_lock(int timeout_ms) {
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}


void lvgl_unlock() {
    xSemaphoreGiveRecursive(lvgl_mux);
}

//static void flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map){
static void flush_cb(lv_disp_t *drv, const lv_area_t *area, uint8_t *color_map) {
    ESP_LOGI(TAG, "flush_cb");
//    lv_draw_sw_rgb565_swap(color_map, 480*480);
    esp_lcd_panel_draw_bitmap(panel_handle,
                              area->x1,
                              area->y1,
                              area->x2 + 1,
                              area->y2 + 1,
                              color_map);
    lv_display_flush_ready(drv);
}

static void tick(void *arg) {
    lv_tick_inc(2);
}

#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
void task(void *arg) {
    uint32_t delay = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (true) {
//        lvgl_lock(-1);
        delay = lv_timer_handler();
//        lvgl_unlock();
        if (delay > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            delay = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (delay < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            delay = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(delay / portTICK_PERIOD_MS);
    }
}


void app_main(void) {
    ESP_LOGI(TAG, "Turn on LCD backlight");

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
            .speed_mode       = LEDC_LOW_SPEED_MODE,
            .duty_resolution  = LEDC_TIMER_8_BIT,
            .timer_num        = LEDC_TIMER_0,
            .freq_hz          = 4000,  // Set output frequency at 4 kHz
            .clk_cfg          = LEDC_USE_RC_FAST_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
            .gpio_num       = GPIO_NUM_45,
            .speed_mode     = LEDC_LOW_SPEED_MODE,
            .channel        = LEDC_CHANNEL_0,
            .intr_type      = LEDC_INTR_DISABLE,
            .timer_sel      = LEDC_TIMER_0,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0,(uint32_t)(256/(100/100.00))-1);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ESP_ERROR_CHECK(gpio_sleep_sel_dis(GPIO_NUM_45));

    ESP_LOGI(TAG, "Install 3-wire SPI panel IO");

    esp_lcd_panel_io_3wire_spi_config_t io_config = {
            .line_config = {
                    .cs_io_type = IO_TYPE_GPIO,             // Set to `IO_TYPE_GPIO` if using GPIO, same to below
                    .cs_gpio_num = 35,
                    .scl_io_type = IO_TYPE_GPIO,
                    .scl_gpio_num = 3,
                    .sda_io_type = IO_TYPE_GPIO,
                    .sda_gpio_num = 4,
                    .io_expander = NULL,                        // Set to NULL if not using IO expander
            },
            .expect_clk_speed = 100 * 1000,
            .spi_mode = 0,
            .lcd_cmd_bytes = 1,
            .lcd_param_bytes = 1,
            .flags = {
                    .use_dc_bit = 1,
                    .dc_zero_on_data = 0,
                    .lsb_first = 0,
                    .cs_high_active = 0,
                    .del_keep_cs_inactive = 1,
            },
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_io_3wire_spi(&io_config, &io_handle));

    ESP_LOGI(TAG, "Install ST7701S panel driver");
    esp_lcd_rgb_panel_config_t rgb_config = {
//            .clk_src = LCD_CLK_SRC_XTAL,
            .clk_src = LCD_CLK_SRC_DEFAULT,
            .timings = {
                    .pclk_hz = 10 * 1000 * 1000,
                    .h_res = 480,
                    .v_res = 480,
                    .hsync_pulse_width = 6,
                    .hsync_back_porch = 18,
                    .hsync_front_porch = 24,
                    .vsync_pulse_width = 4,
                    .vsync_back_porch = 10,
                    .vsync_front_porch = 16,
                    .flags = {
                            .hsync_idle_low = 0,
                            .vsync_idle_low = 0,
                            .de_idle_high = 0,
                            .pclk_active_neg = false,
                            .pclk_idle_high = false,}
            },

            .data_width = 16,
            .bits_per_pixel = 16,
            .num_fbs = 2,
            .bounce_buffer_size_px = 0, //10*H_RES,
            .psram_trans_align = 64,
            .hsync_gpio_num = 48,
            .vsync_gpio_num = 47,
            .de_gpio_num = 33,
            .pclk_gpio_num = 34,
            .disp_gpio_num = -1,
            .data_gpio_nums = {11, 12, 13, 3, 4, 5, 6, 7, 14, 15, 16, 17, 18, 8, 9, 10},


            .flags = {
                    .fb_in_psram = 1,
            }
    };
    st7701_vendor_config_t vendor_config = {
            .rgb_config = &rgb_config,
            .init_cmds = buyadisplaycom,
            .init_cmds_size = sizeof(buyadisplaycom) / sizeof(st7701_lcd_init_cmd_t),
            .flags = {
                    .auto_del_panel_io = 1,
            },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
            .reset_gpio_num = 39,
            .rgb_endian = LCD_RGB_ENDIAN_BGR,
            .bits_per_pixel = 18,
            .vendor_config = &vendor_config,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7701(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    panel_st7701_get_frame_buffer(panel_handle, 2, (void **) &buf1, (void **) &buf2);

//    uint8_t *buf3 = malloc(480*20*2);

//    memset(buf1, 255, 480 * 480 * 2);
//    esp_lcd_panel_draw_bitmap(panel_handle,0,0,480,480,buf1);

    lvgl_mux = xSemaphoreCreateRecursiveMutex();

    lv_init();

    const esp_timer_create_args_t lvgl_tick_timer_args = {
            .callback = &tick,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "lvgl_tick",
            .skip_unhandled_events = true
    };
    lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));


    xTaskCreate(task, "LVGL", 10000, NULL, 2, &lvgl_task);

//    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, 480*480*2);
//    lv_disp_drv_init(&disp_drv);
//    disp_drv.hor_res = 480;
//    disp_drv.ver_res = 480;
//    disp_drv.flush_cb = flush_cb;
//    disp_drv.draw_buf = &disp_buf;
//    disp_drv.full_refresh = 1;
//    disp_drv.direct_mode = 1;
//    lv_disp_drv_register(&disp_drv);

    disp = lv_display_create(480, 480);
    lv_display_set_flush_cb(disp, flush_cb);
    lv_display_set_buffers(disp, buf1, buf2, 480 * 480 * 2, LV_DISPLAY_RENDER_MODE_DIRECT);

    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, 2 * 1000));
//    lvgl_lock(-1);


    lv_obj_t *obj = lv_obj_create(lv_scr_act());
    lv_obj_set_flex_flow(obj, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_size(obj, LV_PCT(75),LV_PCT(75));
    lv_obj_center(obj);


    /*lv_file_explorer obj style*/
    lv_obj_set_style_radius(obj, 0, 0);
    lv_obj_set_style_bg_color(obj, lv_color_hex(0xf2f1f6), 0);

    /*main container style*/
    lv_obj_set_style_radius(obj, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_clip_corner(obj, true, 0);
    lv_obj_set_style_pad_bottom(obj, 50, LV_PART_MAIN);
    lv_obj_set_style_pad_top(obj, 50, LV_PART_MAIN);
    lv_obj_set_style_border_side(obj, LV_BORDER_SIDE_NONE, LV_PART_MAIN);
    lv_obj_t *label = lv_label_create(obj);
    lv_obj_center(label);
    lv_label_set_text(label, "It works?");
//    lvgl_unlock();
    while(true){
        //SPPPIIIIN
        vTaskDelay(100/portTICK_PERIOD_MS);
    }
}
