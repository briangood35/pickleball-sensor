/*
 * Copyright (c) 2022 Michal Morsisko
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <zephyr/sys/util.h>

/* Custom Service Variables */
#define BT_UUID_CUSTOM_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

static struct bt_uuid_128 primary_service_uuid = BT_UUID_INIT_128(
	BT_UUID_CUSTOM_SERVICE_VAL);

static struct bt_uuid_128 read_accelX_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static struct bt_uuid_128 read_accelY_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2));

static struct bt_uuid_128 read_accelZ_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef3));

static struct bt_uuid_128 read_gyroX_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef4));

static struct bt_uuid_128 read_gyroY_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef5));

static struct bt_uuid_128 read_gyroZ_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef6));

static struct bt_uuid_128 read_timestamp_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef7));

// most recent sensor readings
static float accelX, accelY, accelZ;
static float gyroX, gyroY, gyroZ;

// buffer of 5 readings to be sent in a BLE Notification payload
static float accelX_buffer[5], accelY_buffer[5], accelZ_buffer[5];
static float gyroX_buffer[5], gyroY_buffer[5], gyroZ_buffer[5];
static uint32_t timestamp_buffer[5];

// boolean that is set in notification callback after notification is sent
static bool accelX_notified, accelY_notified, accelZ_notified;
static bool gyroX_notified, gyroY_notified, gyroZ_notified;
static bool timestamp_notified;

static struct bt_le_adv_param adv_param;

static ssize_t read_accelX(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	float values[5];
	memcpy(values, &accelX_buffer, sizeof(float) * 5);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, values,
				 sizeof(float)*5);
}

static ssize_t read_accelY(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	float values[5];
	memcpy(values, &accelY_buffer, sizeof(float) * 5);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, values,
				 sizeof(float)*5);
}

static ssize_t read_accelZ(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	float values[5];
	memcpy(values, &accelZ_buffer, sizeof(float) * 5);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, values,
				 sizeof(float)*5);
}

static ssize_t read_gyroX(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	float values[5];
	memcpy(values, &gyroX_buffer, sizeof(float) * 5);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, values,
				 sizeof(float)*5);
}

static ssize_t read_gyroY(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	float values[5];
	memcpy(values, &gyroY_buffer, sizeof(float) * 5);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, values,
				 sizeof(float)*5);
}

static ssize_t read_gyroZ(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	float values[5];
	memcpy(values, &gyroZ_buffer, sizeof(float) * 5);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, values,
				 sizeof(float)*5);
}

static ssize_t read_timestamp(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	float values[5];
	memcpy(values, &timestamp_buffer, sizeof(uint32_t) * 5);

	return bt_gatt_attr_read(conn, attr, buf, len, offset, values,
				 sizeof(uint32_t)*5);
}

static void accelX_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	accelX_notified = value == BT_GATT_CCC_NOTIFY;
}

static void accelY_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	accelY_notified = value == BT_GATT_CCC_NOTIFY;
}

static void accelZ_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	accelZ_notified = value == BT_GATT_CCC_NOTIFY;
}

static void gyroX_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	gyroX_notified = value == BT_GATT_CCC_NOTIFY;
}

static void gyroY_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	gyroY_notified = value == BT_GATT_CCC_NOTIFY;
}

static void gyroZ_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	gyroZ_notified = value == BT_GATT_CCC_NOTIFY;
}

static void timestamp_changed(const struct bt_gatt_attr *attr,
				 uint16_t value)
{
	timestamp_notified = value == BT_GATT_CCC_NOTIFY;
}

/* Vendor Primary Service Declaration */
BT_GATT_SERVICE_DEFINE(primary_service,
	BT_GATT_PRIMARY_SERVICE(&primary_service_uuid),
	BT_GATT_CHARACTERISTIC(&read_accelX_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_accelX, NULL, NULL),
	BT_GATT_CCC(accelX_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&read_accelY_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_accelY, NULL, NULL),
	BT_GATT_CCC(accelY_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&read_accelZ_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_accelZ, NULL, NULL),
	BT_GATT_CCC(accelZ_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&read_gyroX_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_gyroX, NULL, NULL),
	BT_GATT_CCC(gyroX_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&read_gyroY_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_gyroY, NULL, NULL),
	BT_GATT_CCC(gyroY_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&read_gyroZ_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_gyroZ, NULL, NULL),
	BT_GATT_CCC(gyroZ_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(&read_timestamp_uuid.uuid,
			       BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ,
			       read_timestamp, NULL, NULL),
	BT_GATT_CCC(timestamp_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR))
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL)
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	adv_param = *BT_LE_ADV_CONN_NAME;

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
	} else {
		printk("Advertising successfully started\n");
	}
}

static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

static int print_samples;
static int lsm6dsl_trig_cnt;
static struct sensor_value accel_x_out, accel_y_out, accel_z_out;
static struct sensor_value gyro_x_out, gyro_y_out, gyro_z_out;

static void lsm6dsl_trigger_handler(const struct device *dev,
				    const struct sensor_trigger *trig)
{
	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;

	lsm6dsl_trig_cnt++;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);

	/* lsm6dsl gyro */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

	accel_x_out = accel_x;
	accel_y_out = accel_y;
	accel_z_out = accel_z;

	gyro_x_out = gyro_x;
	gyro_y_out = gyro_y;
	gyro_z_out = gyro_z;

	// if (print_samples) {
	// 	print_samples = 0;

	// 	accel_x_out = accel_x;
	// 	accel_y_out = accel_y;
	// 	accel_z_out = accel_z;

	// 	gyro_x_out = gyro_x;
	// 	gyro_y_out = gyro_y;
	// 	gyro_z_out = gyro_z;

	// }

}

int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}

	bt_ready();

	char out_str[64];
	struct sensor_value odr_attr;
	const struct device *const lsm6dsl_dev = DEVICE_DT_GET_ONE(st_lsm6dsl);

	if (!device_is_ready(lsm6dsl_dev)) {
		printk("sensor: device not ready.\n");
		sprintf(out_str, "sensor: device not ready.\n");
		return 0;
	}

	/* set accel/gyro sampling frequency to 104 Hz */
	odr_attr.val1 = 104;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return 0;
	}

	if (sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		return 0;
	}

	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;

	if (sensor_trigger_set(lsm6dsl_dev, &trig, lsm6dsl_trigger_handler) != 0) {
		printk("Could not set sensor type and channel\n");
		return 0;
	}

	if (sensor_sample_fetch(lsm6dsl_dev) < 0) {
		printk("Sensor sample update error\n");
		return 0;
	}

	uint32_t timestamp;

	while (1) {

		accelX = out_ev(&accel_x_out);
		accelY = out_ev(&accel_y_out);
		accelZ = out_ev(&accel_z_out);

		gyroX = out_ev(&gyro_x_out);
		gyroY = out_ev(&gyro_y_out);
		gyroZ = out_ev(&gyro_z_out);

		timestamp = k_uptime_get_32();

		bt_gatt_notify(NULL, &primary_service.attrs[2], &accelX, sizeof(float));
		bt_gatt_notify(NULL, &primary_service.attrs[5], &accelY, sizeof(float));
		bt_gatt_notify(NULL, &primary_service.attrs[8], &accelZ, sizeof(float));

		bt_gatt_notify(NULL, &primary_service.attrs[11], &gyroX, sizeof(float));
		bt_gatt_notify(NULL, &primary_service.attrs[14], &gyroY, sizeof(float));
		bt_gatt_notify(NULL, &primary_service.attrs[17], &gyroZ, sizeof(float));

		bt_gatt_notify(NULL, &primary_service.attrs[20], &timestamp, sizeof(uint32_t));

		k_sleep(K_MSEC(50));

// #define CONSOLE_PRINT
#if defined(CONSOLE_PRINT)
		printk("\0033\014");
		printf("LSM6DSL sensor samples:\n\n");

		/* lsm6dsl accel */
		sprintf(out_str, "accel x:%f ms/2 y:%f ms/2 z:%f ms/2",
							  accelX,
							  accelY,
							  accelZ);
		printk("%s\n", out_str);

		/* lsm6dsl gyro */
		sprintf(out_str, "gyro x:%f dps y:%f dps z:%f dps",
							   gyroX,
							   gyroY,
							   gyroZ);
		printk("%s\n", out_str);

		print_samples = 1;
		k_sleep(K_MSEC(100));
#endif
	}
	return 0;
}
