/**
 * smbus_func.h was mostly copied and edited from: https://github.com/mozilla-b2g/i2c-tools.git
 *      smbus.c - SMBus level access helper functions
 *      Copyright (C) 1995-97 Simon G. Vogl
 *      Copyright (C) 1998-99 Frodo Looijaard <frodol@dds.nl>
 *      Copyright (C) 2012    Jean Delvare <khali@linux-fr.org>
 * 
*/

#pragma once

#include <errno.h>
#include <stddef.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

/* Compatibility defines */
#ifndef I2C_SMBUS_I2C_BLOCK_BROKEN
#define I2C_SMBUS_I2C_BLOCK_BROKEN I2C_SMBUS_I2C_BLOCK_DATA
#endif
#ifndef I2C_FUNC_SMBUS_PEC
#define I2C_FUNC_SMBUS_PEC I2C_FUNC_SMBUS_HWPEC_CALC
#endif

/*
 * Data for SMBus Messages
 */
#define I2C_SMBUS_BLOCK_MAX     32      /* As specified in SMBus standard */

union bno_i2c_smbus_data {
        __u8 byte;
        __u16 word;
        __u8 block[I2C_SMBUS_BLOCK_MAX + 2]; /* block[0] is used for length */
                                                    /* and one more for PEC */
};

/* This is the structure as used in the I2C_SMBUS ioctl call */
struct bno_i2c_smbus_ioctl_data {
	char read_write;                /* */
	__u8 command;                   /* */
	int size;                       /* */
	union bno_i2c_smbus_data *data;
};

/* This is the structure as used in the I2C_RDWR ioctl call */
struct bno_i2c_rdwr_ioctl_data {
	struct i2c_msg *msgs;	/* pointers to i2c_msgs */
	int nmsgs;		/* number of i2c_msgs */
};


static inline __s32 bno_i2c_smbus_access(int file, char read_write, __u8 command,
		       int size, union bno_i2c_smbus_data *data)
{
	struct bno_i2c_smbus_ioctl_data args;
	__s32 err;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;

	err = ioctl(file, I2C_SMBUS, &args);
	if (err == -1)
		err = -errno;
	return err;
}


static inline __s32 bno_i2c_smbus_write_quick(int file, __u8 value)
{
	return bno_i2c_smbus_access(file, value, 0, I2C_SMBUS_QUICK, NULL);
}

static inline __s32 bno_i2c_smbus_read_byte(int file)
{
	union bno_i2c_smbus_data data;
	int err;

	err = bno_i2c_smbus_access(file, I2C_SMBUS_READ, 0, I2C_SMBUS_BYTE, &data);
	if (err < 0)
		return err;

	return 0x0FF & data.byte;
}

static inline __s32 bno_i2c_smbus_write_byte(int file, __u8 value)
{
	return bno_i2c_smbus_access(file, I2C_SMBUS_WRITE, value,
				I2C_SMBUS_BYTE, NULL);
}

static inline __s32 bno_i2c_smbus_read_byte_data(int file, __u8 command)
{
	union bno_i2c_smbus_data data;
	int err;

	err = bno_i2c_smbus_access(file, I2C_SMBUS_READ, command,
			       I2C_SMBUS_BYTE_DATA, &data);
	if (err < 0)
		return err;

	return 0x0FF & data.byte;
}

static inline __s32 bno_i2c_smbus_write_byte_data(int file, __u8 command, __u8 value)
{
	union bno_i2c_smbus_data data;
	data.byte = value;
	return bno_i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BYTE_DATA, &data);
}

static inline __s32 bno_i2c_smbus_read_word_data(int file, __u8 command)
{
	union bno_i2c_smbus_data data;
	int err;

	err = bno_i2c_smbus_access(file, I2C_SMBUS_READ, command,
			       I2C_SMBUS_WORD_DATA, &data);
	if (err < 0)
		return err;

	return 0x0FFFF & data.word;
}

static inline __s32 bno_i2c_smbus_write_word_data(int file, __u8 command, __u16 value)
{
	union bno_i2c_smbus_data data;
	data.word = value;
	return bno_i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
				I2C_SMBUS_WORD_DATA, &data);
}

static inline __s32 bno_i2c_smbus_process_call(int file, __u8 command, __u16 value)
{
	union bno_i2c_smbus_data data;
	data.word = value;
	if (bno_i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
			     I2C_SMBUS_PROC_CALL, &data))
		return -1;
	else
		return 0x0FFFF & data.word;
}

/* Returns the number of read bytes */
static inline __s32 bno_i2c_smbus_read_block_data(int file, __u8 command, __u8 *values)
{
	union bno_i2c_smbus_data data;
	int i, err;

	err = bno_i2c_smbus_access(file, I2C_SMBUS_READ, command,
			       I2C_SMBUS_BLOCK_DATA, &data);
	if (err < 0)
		return err;

	for (i = 1; i <= data.block[0]; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}

static inline __s32 bno_i2c_smbus_write_block_data(int file, __u8 command, __u8 length,
				 const __u8 *values)
{
	union bno_i2c_smbus_data data;
	int i;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return bno_i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
				I2C_SMBUS_BLOCK_DATA, &data);
}

/* Returns the number of read bytes */
/* Until kernel 2.6.22, the length is hardcoded to 32 bytes. If you
   ask for less than 32 bytes, your code will only work with kernels
   2.6.23 and later. */
static inline __s32 bno_i2c_smbus_read_i2c_block_data(int file, __u8 command, __u8 length,
				    __u8 *values)
{
	union bno_i2c_smbus_data data;
	int i, err;

	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	data.block[0] = length;

	err = bno_i2c_smbus_access(file, I2C_SMBUS_READ, command,
			       length == 32 ? I2C_SMBUS_I2C_BLOCK_BROKEN :
				I2C_SMBUS_I2C_BLOCK_DATA, &data);
	if (err < 0)
		return err;

	for (i = 1; i <= data.block[0]; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}

static inline __s32 bno_i2c_smbus_write_i2c_block_data(int file, __u8 command, __u8 length,
				     const __u8 *values)
{
	union bno_i2c_smbus_data data;
	int i;
	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;
	return bno_i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
				I2C_SMBUS_I2C_BLOCK_BROKEN, &data);
}

/* Returns the number of read bytes */
static inline __s32 bno_i2c_smbus_block_process_call(int file, __u8 command, __u8 length,
				   __u8 *values)
{
	union bno_i2c_smbus_data data;
	int i, err;

	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	for (i = 1; i <= length; i++)
		data.block[i] = values[i-1];
	data.block[0] = length;

	err = bno_i2c_smbus_access(file, I2C_SMBUS_WRITE, command,
			       I2C_SMBUS_BLOCK_PROC_CALL, &data);
	if (err < 0)
		return err;

	for (i = 1; i <= data.block[0]; i++)
		values[i-1] = data.block[i];
	return data.block[0];
}