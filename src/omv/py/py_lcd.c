/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * LCD Python module.
 *
 */
#include <mp.h>
#include <objstr.h>
#include <spi.h>
#include <systick.h>
#include "imlib.h"
#include "fb_alloc.h"
#include "ff_wrapper.h"
#include "py_assert.h"
#include "py_helper.h"
#include "py_image.h"
#include "ili9341_regs.h"

#define RST_PORT            GPIOC
#define RST_PIN             GPIO_PIN_4
#define RST_PIN_WRITE(bit)  HAL_GPIO_WritePin(RST_PORT, RST_PIN, bit);

#define RS_PORT             GPIOA
#define RS_PIN              GPIO_PIN_5
#define RS_PIN_WRITE(bit)   HAL_GPIO_WritePin(RS_PORT, RS_PIN, bit);

#define CS_PORT             GPIOB
#define CS_PIN              GPIO_PIN_12
#define CS_PIN_WRITE(bit)   HAL_GPIO_WritePin(CS_PORT, CS_PIN, bit);

static int lcd_has_blctl = 0;
static enum { LCD_PORTRAIT, LCD_PORTRAIT1, LCD_LANDSCAPE, LCD_LANDSCAPE1 } lcd_rotation = LCD_LANDSCAPE;

#define LED_PORT            GPIOD
#define LED_PIN             GPIO_PIN_4
#define LED_PIN_WRITE(bit)  do {                                   \
		if (lcd_has_blctl)                                    \
			HAL_GPIO_WritePin(LED_PORT, LED_PIN, bit); \
	} while (0)

extern mp_obj_t pyb_spi_send(mp_uint_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args);
extern mp_obj_t pyb_spi_make_new(mp_obj_t type_in, mp_uint_t n_args, mp_uint_t n_kw, const mp_obj_t *args);
extern mp_obj_t pyb_spi_deinit(mp_obj_t self_in);

static mp_obj_t spi_port = NULL;
static int width = 0;
static int height = 0;
static enum { LCD_NONE, LCD_Z180, LCD_Z240, LCD_TYPE_MAX = LCD_Z240 } type = LCD_NONE;
static bool backlight_init = false;

// Send out 8-bit data using the SPI object.
static void lcd_write_command_byte(uint8_t data_byte)
{
    mp_map_t arg_map;
    arg_map.all_keys_are_qstrs = true;
    arg_map.is_fixed = true;
    arg_map.is_ordered = true;
    arg_map.used = 0;
    arg_map.alloc = 0;
    arg_map.table = NULL;

    CS_PIN_WRITE(false);
    RS_PIN_WRITE(false); // command
    pyb_spi_send(
        2, (mp_obj_t []) {
            spi_port,
            mp_obj_new_int(data_byte)
        },
        &arg_map
    );
    CS_PIN_WRITE(true);
}

// Send out 8-bit data using the SPI object.
static void lcd_write_data_byte(uint8_t data_byte)
{
    mp_map_t arg_map;
    arg_map.all_keys_are_qstrs = true;
    arg_map.is_fixed = true;
    arg_map.is_ordered = true;
    arg_map.used = 0;
    arg_map.alloc = 0;
    arg_map.table = NULL;

    CS_PIN_WRITE(false);
    RS_PIN_WRITE(true); // data
    pyb_spi_send(
        2, (mp_obj_t []) {
            spi_port,
            mp_obj_new_int(data_byte)
        },
        &arg_map
    );
    CS_PIN_WRITE(true);
}

// Send out 8-bit data using the SPI object.
static void lcd_write_command(uint8_t data_byte, uint32_t len, uint8_t *dat)
{
    lcd_write_command_byte(data_byte);
    for (uint32_t i=0; i<len; i++) lcd_write_data_byte(dat[i]);
}

// Send out 8-bit data using the SPI object.
static void lcd_write_data(uint32_t len, uint8_t *dat)
{
    mp_obj_str_t arg_str;
    arg_str.base.type = &mp_type_bytes;
    arg_str.hash = 0;
    arg_str.len = len;
    arg_str.data = dat;

    mp_map_t arg_map;
    arg_map.all_keys_are_qstrs = true;
    arg_map.is_fixed = true;
    arg_map.is_ordered = true;
    arg_map.used = 0;
    arg_map.alloc = 0;
    arg_map.table = NULL;

    CS_PIN_WRITE(false);
    RS_PIN_WRITE(true); // data
    pyb_spi_send(
        2, (mp_obj_t []) {
            spi_port,
            &arg_str
        },
        &arg_map
    );
    CS_PIN_WRITE(true);
}

static mp_obj_t py_lcd_deinit()
{
    switch (type) {
        case LCD_NONE:
            return mp_const_none;
        case LCD_Z180:
        case LCD_Z240:
            HAL_GPIO_DeInit(RST_PORT, RST_PIN);
            HAL_GPIO_DeInit(RS_PORT, RS_PIN);
            HAL_GPIO_DeInit(CS_PORT, CS_PIN);
            pyb_spi_deinit(spi_port);
            spi_port = NULL;
            width = 0;
            height = 0;
            type = LCD_NONE;
            if (backlight_init) {
		    if (lcd_has_blctl) {
			    HAL_GPIO_DeInit(LED_PORT, LED_PIN);
		    }
                backlight_init = false;
            }
            return mp_const_none;
    }
    return mp_const_none;
}

static void lcd_gpio_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.Pull  = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
	GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_OD;

	GPIO_InitStructure.Pin = CS_PIN;
	CS_PIN_WRITE(true); // Set first to prevent glitches.
	HAL_GPIO_Init(CS_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;

	GPIO_InitStructure.Pin = RST_PIN;
	RST_PIN_WRITE(true); // Set first to prevent glitches.
	HAL_GPIO_Init(RST_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.Pin = RS_PIN;
	RS_PIN_WRITE(true); // Set first to prevent glitches.
	HAL_GPIO_Init(RS_PORT, &GPIO_InitStructure);
}

#define lcd_cmd  lcd_write_command_byte
#define lcd_data lcd_write_data_byte

static void lcd_z240_init(void)
{
	lcd_cmd(0xEF);
	lcd_data(0x03);
	lcd_data(0x80);
	lcd_data(0x02);
	lcd_cmd(ILI9341_SWRESET);
	systick_sleep(50);
	lcd_cmd(ILI9341_POWERA);
	lcd_data(0x39);
	lcd_data(0x2C);
	lcd_data(0x00);
	lcd_data(0x34);
	lcd_data(0x02);
	lcd_cmd(ILI9341_POWERB);
	lcd_data(0x00);
	lcd_data(0xC1);
	lcd_data(0x30);
	lcd_cmd(ILI9341_DTCA);
	lcd_data(0x85);
	lcd_data(0x00);
	lcd_data(0x78);
	lcd_cmd(ILI9341_DTCB);
	lcd_data(0x00);
	lcd_data(0x00);
	lcd_cmd(ILI9341_POWER_SEQ);
	lcd_data(0x64);
	lcd_data(0x03);
	lcd_data(0x12);
	lcd_data(0x81);
	lcd_cmd(ILI9341_PRC);
	lcd_data(0x20);
	lcd_cmd(ILI9341_POWER1);
	lcd_data(0x23);
	lcd_cmd(ILI9341_POWER2);
	lcd_data(0x10);
	lcd_cmd(ILI9341_VCOM1);
	lcd_data(0x3E);
	lcd_data(0x28);
	lcd_cmd(ILI9341_VCOM2);
	lcd_data(0x86);
	lcd_cmd(ILI9341_MAC);
        if (lcd_rotation == LCD_PORTRAIT)
		lcd_data(0x48);
	else
		lcd_data(0xe8); /* landscape */
	lcd_cmd(ILI9341_PIXEL_FORMAT);
	lcd_data(0x55);
	lcd_cmd(ILI9341_FRC);
	lcd_data(0x00);
	lcd_data(0x18);
	lcd_cmd(ILI9341_DFC);
	lcd_data(0x08);
	lcd_data(0x82);
	lcd_data(0x27);
	lcd_cmd(ILI9341_3GAMMA_EN);
	lcd_data(0x00);
	lcd_cmd(ILI9341_COLUMN_ADDR);
	lcd_data(0x00);
	lcd_data(0x00);
	lcd_data(0x00);
	lcd_data(0xEF);
	lcd_cmd(ILI9341_PAGE_ADDR);
	lcd_data(0x00);
	lcd_data(0x00);
	lcd_data(0x01);
	lcd_data(0x3F);
	lcd_cmd(ILI9341_GAMMA);
	lcd_data(0x01);
	lcd_cmd(ILI9341_PGAMMA);
	lcd_data(0x0F);
	lcd_data(0x31);
	lcd_data(0x2B);
	lcd_data(0x0C);
	lcd_data(0x0E);
	lcd_data(0x08);
	lcd_data(0x4E);
	lcd_data(0xF1);
	lcd_data(0x37);
	lcd_data(0x07);
	lcd_data(0x10);
	lcd_data(0x03);
	lcd_data(0x0E);
	lcd_data(0x09);
	lcd_data(0x00);
	lcd_cmd(ILI9341_NGAMMA);
	lcd_data(0x00);
	lcd_data(0x0E);
	lcd_data(0x14);
	lcd_data(0x03);
	lcd_data(0x11);
	lcd_data(0x07);
	lcd_data(0x31);
	lcd_data(0xC1);
	lcd_data(0x48);
	lcd_data(0x08);
	lcd_data(0x0F);
	lcd_data(0x0C);
	lcd_data(0x31);
	lcd_data(0x36);
	lcd_data(0x0F);
	lcd_cmd(ILI9341_SLEEP_OUT);

	systick_sleep(100);

	lcd_cmd(ILI9341_DISPLAY_ON);
	lcd_cmd(ILI9341_GRAM);
}

static void lcd_z180_init(void)
{
	lcd_write_command_byte(0x11); // Sleep Exit
	systick_sleep(120);

	// Memory Data Access Control
	lcd_write_command(0x36, 1, (uint8_t []) {0xC0});

	// Interface Pixel Format
	lcd_write_command(0x3A, 1, (uint8_t []) {0x05});

	// Display on
	lcd_write_command_byte(0x29);
}

static mp_obj_t py_lcd_init(uint n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
    int blctl_default;
    int rot_default;
    int lcd_type;

    py_lcd_deinit();

    lcd_type = py_helper_lookup_int(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_type), LCD_Z240);
    if (lcd_type == LCD_NONE || lcd_type > LCD_TYPE_MAX) {
	    lcd_type = LCD_NONE;
	    lcd_has_blctl = 0;
	    return mp_const_none;
    }

    if (lcd_type == LCD_Z240) {
	    blctl_default = 1;
	    rot_default = LCD_LANDSCAPE;
    } else {
	    blctl_default = 0;
	    rot_default = LCD_PORTRAIT;
    }

    lcd_has_blctl = py_helper_lookup_int(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_has_blctl), blctl_default);
    
    lcd_rotation = py_helper_lookup_int(kw_args, MP_OBJ_NEW_QSTR(MP_QSTR_rotation), rot_default);

    /* for z180 & z240 */
    lcd_gpio_init();

    spi_port = pyb_spi_make_new(NULL,
                        2, // n_args
                        3, // n_kw
                        (mp_obj_t []) {
                                MP_OBJ_NEW_SMALL_INT(2), // SPI Port
                                MP_OBJ_NEW_SMALL_INT(SPI_MODE_MASTER),
				MP_OBJ_NEW_QSTR(MP_QSTR_baudrate),
				MP_OBJ_NEW_SMALL_INT(1000000000/66), // 66 ns clk period
				MP_OBJ_NEW_QSTR(MP_QSTR_polarity),
				MP_OBJ_NEW_SMALL_INT(0),
				MP_OBJ_NEW_QSTR(MP_QSTR_phase),
				MP_OBJ_NEW_SMALL_INT(0)
			}
    );
    backlight_init = false;

    RST_PIN_WRITE(false);
    systick_sleep(100);
    RST_PIN_WRITE(true);
    systick_sleep(100);

    type = lcd_type;

    if (lcd_type == LCD_Z180) {
        if (lcd_rotation == LCD_PORTRAIT) {
            width = 128;
	    height = 160;
	} else {
            width = 160;
	    height = 128;
	}
	lcd_z180_init();	
    } else if (lcd_type == LCD_Z240) {
        if (lcd_rotation == LCD_PORTRAIT) {
            width = 240;
	    height = 320;
	} else {
            width = 320;
	    height = 240;
	}
	lcd_z240_init();
    }

    return mp_const_none;
}

static mp_obj_t py_lcd_width()
{
    if (type == LCD_NONE) return mp_const_none;
    return mp_obj_new_int(width);
}

static mp_obj_t py_lcd_height()
{
    if (type == LCD_NONE) return mp_const_none;
    return mp_obj_new_int(height);
}

static mp_obj_t py_lcd_type()
{
    if (type == LCD_NONE) return mp_const_none;
    return mp_obj_new_int(type);
}

static mp_obj_t py_lcd_set_backlight(mp_obj_t state_obj)
{
    switch (type) {
        case LCD_NONE:
            return mp_const_none;
        case LCD_Z240:
        case LCD_Z180:
        if (lcd_has_blctl) {
            bool bit = !!mp_obj_get_int(state_obj);
            if (!backlight_init) {
                GPIO_InitTypeDef GPIO_InitStructure;
                GPIO_InitStructure.Pull  = GPIO_NOPULL;
                GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
                GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
                GPIO_InitStructure.Pin = LED_PIN;
                LED_PIN_WRITE(bit); // Set first to prevent glitches.
                HAL_GPIO_Init(LED_PORT, &GPIO_InitStructure);
                backlight_init = true;
            }
            LED_PIN_WRITE(bit);
            return mp_const_none;
        } else  {
	    return mp_const_none;
	}
    }

    return mp_const_none;
}

static mp_obj_t py_lcd_get_backlight()
{
    switch (type) {
        case LCD_NONE:
            return mp_const_none;
        case LCD_Z240:
        case LCD_Z180:
            if (!backlight_init) {
                return mp_const_none;
            }
	    if (lcd_has_blctl)
		    return mp_obj_new_int(HAL_GPIO_ReadPin(LED_PORT, LED_PIN));
	    else
		    return mp_const_none;
    }
    return mp_const_none;
}

void lcd_z240_set_xy_range(uint16_t x1, uint16_t x2, uint16_t y1, uint16_t y2)
{
        lcd_cmd(ILI9341_COLUMN_ADDR);
        lcd_data(x1 >> 8);
        lcd_data(x1 & 0xff);
        lcd_data(x2 >> 8);
        lcd_data(x2 & 0xff);

        lcd_cmd(ILI9341_PAGE_ADDR);
        lcd_data(y1 >> 8);
        lcd_data(y1  &0xff);
        lcd_data(y2 >> 8);
        lcd_data(y2  &0xff);
}

void lcd_z240_set_xy(uint16_t x, uint16_t y)
{
	lcd_z240_set_xy_range(x, width - 1, y, height - 1);
}

#if 0
void lcd_z240_data16bit(uint16_t data)
{
	gpio_set_value(PIN_LCD_DC, 1);
	
	gpio_set_value(PIN_LCD_CS, 0);
	HAL_SPI_Transmit(spi_bus, (uint8_t*)&data, 2, HAL_MAX_DELAY);
	gpio_set_value(PIN_LCD_CS, 1);
}
#endif

static mp_obj_t py_lcd_display_soft_padding(uint n_args, const mp_obj_t *args, mp_map_t *kw_args);
static mp_obj_t py_lcd_display(uint n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
	if (type == LCD_NONE)
		return mp_const_none;

	if (type == LCD_Z180)
		return py_lcd_display_soft_padding(n_args, args, kw_args);

	image_t *arg_img = py_image_cobj(args[0]);
	PY_ASSERT_FALSE_MSG(IM_IS_JPEG(arg_img),
			    "Operation not supported on JPEG");

	mp_map_elem_t *kw_roi = mp_map_lookup(kw_args,
				    MP_OBJ_NEW_QSTR(MP_QSTR_roi), MP_MAP_LOOKUP);

	if (kw_roi) {
		return py_lcd_display_soft_padding(n_args, args, kw_args);
	}

	rectangle_t rect;
	rect.x = 0;
	rect.y = 0;
	rect.w = arg_img->w;
	rect.h = arg_img->h;

	if (rect.w > width || rect.h > height || !IM_IS_RGB565(arg_img)) {
		return py_lcd_display_soft_padding(n_args, args, kw_args);
	}

	if (rect.w > 220 || rect.h > 160) {
		return py_lcd_display_soft_padding(n_args, args, kw_args);
	}

	// Fit X.
	int l_pad = 0, r_pad = 0;
	if (rect.w < width) {
		int adjust = width - rect.w;
		l_pad = adjust / 2;
		r_pad = (adjust + 1) / 2;
	}

	// Fit Y.
	int t_pad = 0, b_pad = 0;
	if (rect.h < height) {
		int adjust = height - rect.h;
		t_pad = adjust / 2;
		b_pad = (adjust + 1) / 2;
	}

	lcd_z240_set_xy_range(0 + l_pad, width - r_pad - 1, 0 + t_pad, height - b_pad - 1);
	lcd_write_command_byte(0x2C); /* ILI9341_GRAM */

	if (IM_IS_RGB565(arg_img)) {
		lcd_write_data(rect.w * rect.h * 2, arg_img->data);
	}

	return mp_const_none;
}

/* yuanjun : py_lcd_display_soft_padding is original version lcd_display */
static mp_obj_t py_lcd_display_soft_padding(uint n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
    image_t *arg_img = py_image_cobj(args[0]);
    PY_ASSERT_FALSE_MSG(IM_IS_JPEG(arg_img),
            "Operation not supported on JPEG");

    rectangle_t arg_r;
    py_helper_lookup_rectangle(kw_args, arg_img, &arg_r);
    rectangle_t rect;
    if (!rectangle_subimg(arg_img, &arg_r, &rect)) ff_no_intersection(NULL);

    // Fit X.
    int l_pad = 0, r_pad = 0;
    if (rect.w > width) {
        int adjust = rect.w - width;
        rect.w -= adjust;
        rect.x += adjust / 2;
    } else if (rect.w < width) {
        int adjust = width - rect.w;
        l_pad = adjust / 2;
        r_pad = (adjust + 1) / 2;
    }

    // Fit Y.
    int t_pad = 0, b_pad = 0;
    if (rect.h > height) {
        int adjust = rect.h - height;
        rect.h -= adjust;
        rect.y += adjust / 2;
    } else if (rect.h < height) {
        int adjust = height - rect.h;
        t_pad = adjust / 2;
        b_pad = (adjust + 1) / 2;
    }

    switch (type) {
        case LCD_NONE:
            return mp_const_none;
        case LCD_Z240:
		lcd_z240_set_xy_range(0, width, 0, height);
        case LCD_Z180:
            lcd_write_command_byte(0x2C);
            uint8_t *zero = fb_alloc0(width*2);
            uint16_t *line = fb_alloc(width*2);
            for (int i=0; i<t_pad; i++) {
                lcd_write_data(width*2, zero);
            }
            for (int i=0; i<rect.h; i++) {
                if (l_pad) {
                    lcd_write_data(l_pad*2, zero); // l_pad < width
                }
                if (IM_IS_GS(arg_img)) {
                    for (int j=0; j<rect.w; j++) {
                        uint8_t pixel = IM_GET_GS_PIXEL(arg_img, (rect.x + j), (rect.y + i));
                        line[j] = IM_RGB565(IM_R825(pixel),IM_G826(pixel),IM_B825(pixel));
                    }
                    lcd_write_data(rect.w*2, (uint8_t *) line);
                } else {
                    lcd_write_data(rect.w*2, (uint8_t *)
                        (((uint16_t *) arg_img->pixels) +
                        ((rect.y + i) * arg_img->w) + rect.x));
                }
                if (r_pad) {
                    lcd_write_data(r_pad*2, zero); // r_pad < width
                }
            }
            for (int i=0; i<b_pad; i++) {
                lcd_write_data(width*2, zero);
            }
            fb_free();
            fb_free();
            return mp_const_none;
    }
    return mp_const_none;
}

static mp_obj_t py_lcd_clear()
{
    switch (type) {
        case LCD_NONE:
            return mp_const_none;
        case LCD_Z240:
		lcd_z240_set_xy_range(0, width, 0, height);
        case LCD_Z180:
            lcd_write_command_byte(0x2C);
            uint8_t *zero = fb_alloc0(width*2);
            for (int i=0; i<height; i++) {
                lcd_write_data(width*2, zero);
            }
            fb_free();
            return mp_const_none;
    }
    return mp_const_none;
}

STATIC MP_DEFINE_CONST_FUN_OBJ_KW(py_lcd_init_obj, 0, py_lcd_init);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_lcd_deinit_obj, py_lcd_deinit);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_lcd_width_obj, py_lcd_width);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_lcd_height_obj, py_lcd_height);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_lcd_type_obj, py_lcd_type);
STATIC MP_DEFINE_CONST_FUN_OBJ_1(py_lcd_set_backlight_obj, py_lcd_set_backlight);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_lcd_get_backlight_obj, py_lcd_get_backlight);
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(py_lcd_display_obj, 1, py_lcd_display);
STATIC MP_DEFINE_CONST_FUN_OBJ_0(py_lcd_clear_obj, py_lcd_clear);
static const mp_map_elem_t globals_dict_table[] = {
    { MP_OBJ_NEW_QSTR(MP_QSTR___name__),        MP_OBJ_NEW_QSTR(MP_QSTR_lcd) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_init),            (mp_obj_t)&py_lcd_init_obj          },
    { MP_OBJ_NEW_QSTR(MP_QSTR_deinit),          (mp_obj_t)&py_lcd_deinit_obj        },
    { MP_OBJ_NEW_QSTR(MP_QSTR_width),           (mp_obj_t)&py_lcd_width_obj         },
    { MP_OBJ_NEW_QSTR(MP_QSTR_height),          (mp_obj_t)&py_lcd_height_obj        },
    { MP_OBJ_NEW_QSTR(MP_QSTR_type),            (mp_obj_t)&py_lcd_type_obj          },
    { MP_OBJ_NEW_QSTR(MP_QSTR_set_backlight),   (mp_obj_t)&py_lcd_set_backlight_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_backlight),   (mp_obj_t)&py_lcd_get_backlight_obj },
    { MP_OBJ_NEW_QSTR(MP_QSTR_display),         (mp_obj_t)&py_lcd_display_obj       },
    { MP_OBJ_NEW_QSTR(MP_QSTR_clear),           (mp_obj_t)&py_lcd_clear_obj         },
    { NULL, NULL },
};
STATIC MP_DEFINE_CONST_DICT(globals_dict, globals_dict_table);

const mp_obj_module_t lcd_module = {
    .base = { &mp_type_module },
    .name = MP_QSTR_lcd,
    .globals = (mp_obj_t)&globals_dict,
};

void py_lcd_init0()
{
    py_lcd_deinit();
}
