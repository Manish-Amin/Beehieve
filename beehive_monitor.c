/*
 * beehive_monitor.c
 *
 * Beehive Environmental Monitor - Proof of Concept Firmware
 * Myriota Module + MI319 Temperature Sensor (I2C)
 *
 * Operation:
 *   - Reads temperature from MI319 sensor once per hour (manual/on-demand mode)
 *   - Buffers four readings (4 hours worth)
 *   - Transmits all four readings as a single satellite message
 *   - Sensor remains in deep sleep (~0.7uA) between reads
 *
 * Author: Manish Amin
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "myriota_user_api.h"
#include "myriota_hardware_api.h"

/* Compile-time logging switch: set to 1 for debug builds, 0 for production. */
#ifndef BEEHIVE_ENABLE_LOG
#define BEEHIVE_ENABLE_LOG 0
#endif

#if BEEHIVE_ENABLE_LOG
#define LOGF(...) printf(__VA_ARGS__)
#else
#define LOGF(...) do { } while (0)
#endif

/* -------------------------------------------------------------------------
 * MI319 Sensor Configuration
 * -------------------------------------------------------------------------
 * I2C device address for the MI319 sensor (fixed at manufacture).
 */
#define MI319_I2C_ADDRESS       0x6C

/* Expected device ID value - used to validate sensor presence on startup */
#define MI319_DEVICE_ID         0x74

/* Register addresses from the MI319 datasheet */
#define MI319_REG_TEMP_MSB      0xAA    /* Temperature[11:8] in bits [3:0] */
#define MI319_REG_TEMP_LSB      0xAB    /* Temperature[7:0]  */
#define MI319_REG_CAL_A         0xAC    /* Calibration A (set at manufacture) */
#define MI319_REG_CAL_B         0xAD    /* Calibration B (set at manufacture) */
#define MI319_REG_READ_MODE     0xA8    /* Read mode: 0x0=idle, 0x1=performing read */
#define MI319_REG_CONFIG1       0xA7    /* period_A[7:4], period_B[3:0] */
#define MI319_REG_CONFIG2       0xA6    /* Read accuracy in bits [1:0] */
#define MI319_REG_ID            0xA5    /* Device ID register */

/*
 * Config 1: period_A = 0 selects manual (on-demand) querying mode.
 * When period_A is 0, the sensor does not auto-sample; we trigger reads
 * via the Read Mode register. This keeps the sensor in deep sleep (~0.7uA)
 * between reads, maximising battery life.
 */
#define MI319_CONFIG1_MANUAL_MODE   0x00

/*
 * Config 2: Read accuracy / power mode selection.
 *
 * 0x0 = Ultra low power (1x oversampling, 2.7uA @ 1Hz)
 * 0x1 = Low power      (2x oversampling, 4.2uA @ 1Hz)
 * 0x2 = Standard       (4x oversampling, 7.0uA @ 1Hz)
 * 0x3 = High resolution(8x oversampling, 12.7uA @ 1Hz)
 *
 * We select Ultra Low Power. For beehive temperature monitoring,
 * 1x oversampling gives adequate accuracy. The power saving over
 * high resolution is ~10uA per reading, which is significant for
 * a device expected to run unattended for months or years.
 */
#define MI319_CONFIG2_ULTRA_LOW_POWER   0x00

/* Read mode register values */
#define MI319_READ_MODE_IDLE        0x00
#define MI319_READ_MODE_TRIGGER     0x01

/* -------------------------------------------------------------------------
 * Application Configuration
 * -------------------------------------------------------------------------
 */

/*
 * Number of temperature readings to buffer before transmitting.
 * Task requires 4 readings (4 hours) per satellite message.
 */
#define READINGS_PER_MESSAGE    4

/*
 * Reading interval in hours.
 * One reading per hour as specified in the task.
 */
#define READING_INTERVAL_HOURS  1

/* -------------------------------------------------------------------------
 * Message Structure
 * -------------------------------------------------------------------------
 * Packed struct ensures no compiler padding is inserted between fields.
 * This is essential for satellite messages where the byte layout must
 * be deterministic and match the server-side decoder.
 *
 * Each temperature is stored as a signed 16-bit integer in degrees Celsius
 * (scaled x100 for 2 decimal places, e.g. 2350 = 23.50°C).
 *
 * Total message size: 2 + (4 * 2) = 10 bytes, well within MAX_MESSAGE_SIZE.
 */
typedef struct {
    uint16_t sequence_number;               /* Monotonic counter for message ordering */
    int16_t  temperature_cdegC[READINGS_PER_MESSAGE]; /* Temperatures in centidegrees */
} __attribute__((packed)) beehive_message_t;

/* -------------------------------------------------------------------------
 * Module-level state
 * -------------------------------------------------------------------------
 */
static int16_t  reading_buffer[READINGS_PER_MESSAGE];   /* Circular buffer of readings */
static uint8_t  reading_count = 0;                      /* Number of readings buffered */
static uint16_t message_sequence = 0;                   /* Monotonic message counter */
static uint8_t  calibration_a = 0;                      /* Factory calibration A */
static uint8_t  calibration_b = 0;                      /* Factory calibration B */

/*
 * TryScheduleBufferedMessage - Queue the 4-hour cumulative payload when ready.
 *
 * Returns:
 *   0 on success (or if buffer is not full yet)
 *  -1 if scheduling failed and data is retained for retry.
 */
static int TryScheduleBufferedMessage(void)
{
    if (reading_count < READINGS_PER_MESSAGE) {
        return 0;
    }

    beehive_message_t msg;
    msg.sequence_number = message_sequence;
    memcpy(msg.temperature_cdegC, reading_buffer,
           sizeof(int16_t) * READINGS_PER_MESSAGE);

    if (MessageSlotsFree() <= 0 || MessageBytesFree() < sizeof(msg)) {
        LOGF("BeeHive: Queue full, retaining 4-hour window for retry\n");
        return -1;
    }

    int result = ScheduleMessage((uint8_t *)&msg, sizeof(msg));
    if (result >= 0) {
        LOGF("BeeHive: Message %u scheduled (%u bytes). Queue load: %d\n",
             msg.sequence_number, (unsigned)sizeof(msg), result);
        message_sequence++;
        reading_count = 0;
        memset(reading_buffer, 0, sizeof(reading_buffer));
        return 0;
    }

    LOGF("BeeHive: Failed to schedule message %u, retaining data for retry\n",
         msg.sequence_number);
    return -1;
}

/* -------------------------------------------------------------------------
 * I2C Helper Functions
 * -------------------------------------------------------------------------
 */

/*
 * ReadRegister8 - Read a single 8-bit register from the MI319 via I2C.
 *
 * Returns the register value (0-255) on success, or -1 on failure.
 *
 * The Myriota I2C API performs a write of the register address followed
 * immediately by a read of the response byte, which matches the MI319
 * register read protocol.
 */
static int ReadRegister8(uint8_t reg)
{
    uint8_t rx = 0;
    if (I2CRead(MI319_I2C_ADDRESS, &reg, sizeof(reg), &rx, sizeof(rx)) == 0) {
        return (int)rx;
    }
    return -1;
}

/*
 * WriteRegister8 - Write a single 8-bit value to a register on the MI319.
 *
 * Returns 0 on success, -1 on failure.
 *
 * The MI319 I2C write protocol expects the register address followed by
 * the value in the same transaction.
 */
static int WriteRegister8(uint8_t reg, uint8_t value)
{
    uint8_t tx[2];
    tx[0] = reg;
    tx[1] = value;
    if (I2CWrite(MI319_I2C_ADDRESS, tx, sizeof(tx)) == 0) {
        return 0;
    }
    return -1;
}

/* -------------------------------------------------------------------------
 * MI319 Sensor Functions
 * -------------------------------------------------------------------------
 */

/*
 * MI319_Init - Initialise and validate the MI319 sensor.
 *
 * Steps:
 *   1. Read the device ID register and verify it matches the expected value
 *      (0x74). This confirms the sensor is present and the I2C bus is
 *      functioning correctly before we attempt any configuration.
 *   2. Set Config1 to manual mode (period_A = 0). This disables the
 *      sensor's internal auto-sampling timer and puts it in deep sleep
 *      until we explicitly trigger a read.
 *   3. Set Config2 to Ultra Low Power mode.
 *
 * Returns 0 on success, -1 on failure.
 */
static int MI319_Init(void)
{
    /* Step 1: Validate device ID */
    int id = ReadRegister8(MI319_REG_ID);
    if (id != MI319_DEVICE_ID) {
        LOGF("MI319: ID check failed. Expected 0x%02X, got 0x%02X\n",
             MI319_DEVICE_ID, id);
        return -1;
    }
    LOGF("MI319: Device ID OK (0x%02X)\n", id);

    /* Step 2: Configure manual mode - sensor sleeps until we request a read */
    if (WriteRegister8(MI319_REG_CONFIG1, MI319_CONFIG1_MANUAL_MODE) != 0) {
        LOGF("MI319: Failed to set manual mode\n");
        return -1;
    }

    /* Step 3: Set ultra low power read accuracy */
    if (WriteRegister8(MI319_REG_CONFIG2, MI319_CONFIG2_ULTRA_LOW_POWER) != 0) {
        LOGF("MI319: Failed to set power mode\n");
        return -1;
    }

    /* Calibration registers are set at manufacture and can be cached. */
    int cal_a = ReadRegister8(MI319_REG_CAL_A);
    int cal_b = ReadRegister8(MI319_REG_CAL_B);
    if (cal_a < 0 || cal_b < 0) {
        LOGF("MI319: Failed to read calibration registers\n");
        return -1;
    }
    calibration_a = (uint8_t)cal_a;
    calibration_b = (uint8_t)cal_b;

    LOGF("MI319: Initialised in manual mode, ultra low power\n");
    return 0;
}

/*
 * MI319_ReadTemperatureCdegC - Trigger a manual read and return temperature.
 *
 * Process:
 *   1. Set Read Mode register to 0x01 to trigger an on-demand measurement.
 *   2. Read the 12-bit raw temperature from MSB and LSB registers.
 *      - MSB register 0xAA holds Temperature[11:8] in bits [3:0]
 *        (bits [7:4] are reserved)
 *      - LSB register 0xAB holds Temperature[7:0]
 *      - Combined: raw_temp = (msb & 0x0F) << 8 | lsb
 *   3. Apply the calibration formula using cached factory calibration values:
 *      Temperature = (raw_temp * cal_A) >> cal_B
 *      This yields temperature in degrees Celsius as an integer.
 *   4. We scale to centidegrees (x100) for two decimal places of
 *      resolution in the transmitted message without using floats.
 *      (Floating point is avoided in embedded firmware where possible
 *      to reduce code size and avoid soft-float overhead.)
 *
 * Returns temperature in centidegrees Celsius, or INT16_MIN on failure.
 */
static int16_t MI319_ReadTemperatureCdegC(void)
{
    int16_t result = INT16_MIN;
    int read_triggered = 0;

    /* Trigger manual read */
    if (WriteRegister8(MI319_REG_READ_MODE, MI319_READ_MODE_TRIGGER) != 0) {
        LOGF("MI319: Failed to trigger read\n");
        goto cleanup;
    }
    read_triggered = 1;

    /*
     * Small delay to allow the sensor to complete the measurement.
     * At ultra low power (1x oversampling), measurement time is
     * approximately 2ms. We wait 5ms to be safe.
     */
    Delay(5);

    /* Read raw temperature (12-bit, split across two registers) */
    int temp_msb = ReadRegister8(MI319_REG_TEMP_MSB);
    int temp_lsb = ReadRegister8(MI319_REG_TEMP_LSB);
    if (temp_msb < 0 || temp_lsb < 0) {
        LOGF("MI319: Failed to read temperature registers\n");
        goto cleanup;
    }

    /*
     * Reconstruct 12-bit raw temperature value.
     * MSB register bits [3:0] hold Temperature[11:8].
     * LSB register bits [7:0] hold Temperature[7:0].
     */
    uint16_t raw_temp = (uint16_t)((temp_msb & 0x0F) << 8) | (uint8_t)temp_lsb;

    /*
     * Apply calibration formula from datasheet:
     * Temperature (degC) = (raw_temperature * calibration_A) >> calibration_B
     *
     * We scale by 100 before the shift to retain centidegree precision
     * without floating point arithmetic.
     */
    int32_t temp_cdegC = ((int32_t)raw_temp * (int32_t)calibration_a * 100) >> calibration_b;
    LOGF("MI319: raw=%u cal_A=%u cal_B=%u temp_cdegC=%ld\n",
         raw_temp, (unsigned)calibration_a, (unsigned)calibration_b,
         (long)temp_cdegC);
    result = (int16_t)temp_cdegC;

cleanup:
    if (read_triggered && WriteRegister8(MI319_REG_READ_MODE, MI319_READ_MODE_IDLE) != 0) {
        LOGF("MI319: Failed to return sensor to idle mode\n");
    }
    return result;
}

/* -------------------------------------------------------------------------
 * Application Jobs
 * -------------------------------------------------------------------------
 */

/*
 * ReadTemperatureJob - Scheduled job: read sensor and buffer reading.
 *
 * This function is called by the Myriota scheduler every READING_INTERVAL_HOURS.
 *
 * Behaviour:
 *   - Reads temperature from MI319
 *   - Stores in reading_buffer at current index
 *   - Increments reading_count
 *   - When buffer is full (READINGS_PER_MESSAGE readings collected):
 *       - Packages all readings into a beehive_message_t struct
 *       - Calls ScheduleMessage to queue for satellite transmission
 *       - Resets buffer
 *
 * Returns the time at which this job should next run (1 hour from now).
 */
static time_t ReadTemperatureJob(void)
{
    if (I2CInit() != 0) {
        LOGF("BeeHive: I2C init failed in scheduled job\n");
        return HoursFromNow(READING_INTERVAL_HOURS);
    }

    /* If a previous 4-hour payload could not be queued, retry and avoid overwrite. */
    if (reading_count >= READINGS_PER_MESSAGE) {
        (void)TryScheduleBufferedMessage();
        I2CDeinit();
        return HoursFromNow(READING_INTERVAL_HOURS);
    }

    int16_t temp = MI319_ReadTemperatureCdegC();

    if (temp == INT16_MIN) {
        /*
         * Keep strict hourly cadence by storing a sentinel for failed reads.
         * This preserves "4 hours worth of data" windows even with sensor faults.
         */
        LOGF("BeeHive: Temperature read failed, storing sentinel value\n");
    }
    reading_buffer[reading_count] = temp;
    reading_count++;
    LOGF("BeeHive: Reading %u of %u stored: %d cdegC\n",
         reading_count, READINGS_PER_MESSAGE, (int)temp);

    /*
     * Transmit when we have a full buffer of READINGS_PER_MESSAGE readings.
     *
     * Design note: we transmit even if some readings failed (INT16_MIN is used
     * as a sentinel value that the server can detect and discard). Alternatively
     * we could skip transmission if any reading failed, but for a beehive monitor
     * it is better to send partial data than to drop an entire 4-hour window.
     */
    if (reading_count >= READINGS_PER_MESSAGE) {
        (void)TryScheduleBufferedMessage();
    }

    I2CDeinit();
    return HoursFromNow(READING_INTERVAL_HOURS);
}

/* -------------------------------------------------------------------------
 * Application Entry Point
 * -------------------------------------------------------------------------
 */

/*
 * AppInit - Called once by the Myriota system on startup.
 *
 * Initialises the MI319 sensor and schedules the temperature reading job
 * to run every hour.
 *
 * If sensor initialisation fails, no jobs are scheduled. This prevents
 * the firmware from running in a degraded state and wasting battery
 * transmitting empty or corrupt messages.
 */
void AppInit(void)
{
    LOGF("BeeHive Monitor: Starting up\n");
    LOGF("BeeHive Monitor: %d readings per message, %d hour interval\n",
         READINGS_PER_MESSAGE, READING_INTERVAL_HOURS);

    if (I2CInit() != 0) {
        LOGF("BeeHive Monitor: I2C init failed - no jobs scheduled\n");
        return;
    }

    if (MI319_Init() != 0) {
        LOGF("BeeHive Monitor: Sensor init failed - no jobs scheduled\n");
        I2CDeinit();
        return;
    }
    I2CDeinit();

    /* First sample in one hour: first payload then represents a full 4-hour window. */
    if (ScheduleJob(ReadTemperatureJob, HoursFromNow(READING_INTERVAL_HOURS)) != 0) {
        LOGF("BeeHive Monitor: Failed to schedule temperature job\n");
        return;
    }

    LOGF("BeeHive Monitor: Initialised. First reading in %d hour.\n",
         READING_INTERVAL_HOURS);
}
