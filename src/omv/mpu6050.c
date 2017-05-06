/*
 * This file is part of the Micro Vision Device MVD project.
 * Copyright (c) 2016 Micro Vision Device <mvdevice@outlook.com>
 *
 * MPU6050 driver.
 *
 */
#include <stdio.h>
#include STM32_HAL_H
#include "sccb.h"
#include "mpu6050.h"
#include <mp.h>
#include <objstr.h>
#include <systick.h>
#include "py_assert.h"
#include "py_helper.h"

extern const mp_obj_type_t pyb_accel_type;

#define CONFIG_MOTION_DEV_I2C_ADDR (0x68 << 1)

#define X 0
#define Y 1
#define Z 2

#if 0
static void mpu6050_set_clksrc(int clksrc)
{

}
#endif

static uint16_t mpu6050_accel_read(int axis)
{
	uint16_t v = 0;

	return v;
}

static uint16_t mpu6050_gyro_read(int axis)
{
	uint16_t v = 0;

	return v;	
}

static uint8_t mpu6050_read_devid(void)
{
	return SCCB_ReadMem(CONFIG_MOTION_DEV_I2C_ADDR, MPU6050_RA_WHO_AM_I);
}

static uint8_t mpu6050_read_reg(uint8_t raddr)
{
	return SCCB_ReadMem(CONFIG_MOTION_DEV_I2C_ADDR, raddr);
}

static uint8_t mpu6050_write_reg(uint8_t raddr, uint16_t data)
{
	uint8_t ret;
	printf("set reg %02x to %02x\n", raddr, data);
	ret = SCCB_WriteMem(CONFIG_MOTION_DEV_I2C_ADDR, raddr, data);
	printf("read back reg %02x, get %02x\n", raddr, mpu6050_read_reg(raddr));

	return ret;
}

STATIC mp_obj_t pyb_mpu6050_read_reg(mp_obj_t reg_obj)
{
        uint8_t v;
        uint8_t reg = (uint8_t)(mp_obj_get_int(reg_obj) & 0xff);

	v = mpu6050_read_reg(reg);
        return mp_obj_new_int(v);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_mpu6050_read_reg_obj, pyb_mpu6050_read_reg);


STATIC mp_obj_t pyb_mpu6050_write_reg(mp_obj_t reg_obj, mp_obj_t data_obj)
{
        uint8_t v;
        uint8_t reg = (uint8_t)(mp_obj_get_int(reg_obj) & 0xff);
        uint8_t data = (uint8_t)(mp_obj_get_int(data_obj) & 0xff);

        v = mpu6050_write_reg(reg, data);
        return mp_obj_new_int(v);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(pyb_mpu6050_write_reg_obj, pyb_mpu6050_write_reg);

int mpu6050_init0(void)
{
	uint8_t id = mpu6050_read_devid();
	if (id == 0x68) {
		printf("mpu6050 probed, dev_id = 0x68\n");
		return 0;
	} else {
		printf("mpu6050 probe failed!\n");
		return -1;
	}
}

/// \method start_chan()
/// Start mpu6050
STATIC mp_obj_t pyb_mpu6050_start_chan(uint n_args, const mp_obj_t *args, mp_map_t *kw_args)
{
	printf("pyb_mpu6050_start_chan\n");
        return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(pyb_mpu6050_start_chan_obj, 0, pyb_mpu6050_start_chan);

STATIC mp_obj_t pyb_mpu6050_accel_read(mp_obj_t axis_obj)
{
	uint16_t v;
	int axis = mp_obj_get_int(axis_obj);
	
	v = mpu6050_accel_read(axis);
	return mp_obj_new_int(v);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_mpu6050_accel_read_obj, pyb_mpu6050_accel_read);

STATIC mp_obj_t pyb_mpu6050_gyro_read(mp_obj_t axis_obj)
{
	uint16_t v;
	int axis = mp_obj_get_int(axis_obj);

	v = mpu6050_gyro_read(axis);
	return mp_obj_new_int(v);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(pyb_mpu6050_gyro_read_obj, pyb_mpu6050_gyro_read);

STATIC const mp_map_elem_t globals_dict_table_mpu6050[] = {
	{ MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_mpu6050) },

	// MPU6050 functions
	{ MP_OBJ_NEW_QSTR(MP_QSTR_start_chan),          (mp_obj_t)&pyb_mpu6050_start_chan_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_read_mpureg),            (mp_obj_t)&pyb_mpu6050_read_reg_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_write_mpureg),           (mp_obj_t)&pyb_mpu6050_write_reg_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_accel_read),          (mp_obj_t)&pyb_mpu6050_accel_read_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_gyro_read),           (mp_obj_t)&pyb_mpu6050_gyro_read_obj },
};

STATIC MP_DEFINE_CONST_DICT(globals_dict_mpu6050, globals_dict_table_mpu6050);

const mp_obj_module_t mpu6050_module = {
	.base = { &mp_type_module },
	.name = MP_QSTR_mpu6050,
	.globals = (mp_obj_t)&globals_dict_mpu6050,
};

#if 0
#define AXIS_X 0
#define AXIS_Y 1
#define AXIS_Z 2

STATIC mp_obj_t read_axis(int axis) {
//	uint8_t data[1];
//	HAL_I2C_Mem_Read(&I2CHandle1, MMA_ADDR, axis, I2C_MEMADD_SIZE_8BIT, data, 1, 200);
	return mp_obj_new_int(axis + 6);
}

static int motion_start(void)
{
	printf("motion_start ...\n");
	return 0;
}

#define NUM_AXIS   (3)
#define FILT_DEPTH (4)

/// \method start()
/// Start motion 
STATIC mp_obj_t pyb_accel_start(void) {
	motion_start();
	return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(pyb_accel_start_obj, pyb_accel_start);

/// \method x()
/// Get the x-axis value.
STATIC mp_obj_t pyb_accel_x(void) {
	return read_axis(0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(pyb_accel_x_obj, pyb_accel_x);

/// \method y()
/// Get the y-axis value.
STATIC mp_obj_t pyb_accel_y(void) {
	return read_axis(1);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(pyb_accel_y_obj, pyb_accel_y);

/// \method z()
/// Get the z-axis value.
STATIC mp_obj_t pyb_accel_z(void) {
	return read_axis(2);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(pyb_accel_z_obj, pyb_accel_z);

STATIC const mp_map_elem_t pyb_accel_globals_dict_table[] = {
	{ MP_OBJ_NEW_QSTR(MP_QSTR___name__),    MP_OBJ_NEW_QSTR(MP_QSTR_motion) },
	// TODO add init, deinit, and perhaps reset methods
	{ MP_OBJ_NEW_QSTR(MP_QSTR_start), (mp_obj_t)&pyb_accel_start_obj },
	{ MP_OBJ_NEW_QSTR(MP_QSTR_x), (mp_obj_t)&pyb_accel_x_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_y), (mp_obj_t)&pyb_accel_y_obj },
        { MP_OBJ_NEW_QSTR(MP_QSTR_z), (mp_obj_t)&pyb_accel_z_obj },
};

STATIC MP_DEFINE_CONST_DICT(globals_dict, pyb_accel_globals_dict_table);

const mp_obj_module_t motion_module = {
	.base = { &mp_type_module },
	.name = MP_QSTR_motion,
	.globals = (mp_obj_t)&globals_dict,
};
#endif
