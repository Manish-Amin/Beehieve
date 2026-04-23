# Beehive Monitor - Proof of Concept Firmware

## Overview

Firmware for the Myriota Module to monitor temperature in remote beehives.
The firmware wakes once per hour, takes one temperature sample from the MI319 sensor,
and goes back to sleep. Every four hourly samples are sent as a single satellite message.

## Hardware

- Myriota Module (I2C master)
- MI319 Temperature Sensor (I2C address 0x6C)

## Design Decisions

### Manual sensor mode (not scheduled mode)
The MI319 is configured in manual/on-demand mode (Config1 period_A = 0).
The Myriota Module triggers each read explicitly via the Read Mode register.
Between reads, the sensor remains in deep sleep (~0.7uA), maximising battery life.

### Ultra low power accuracy mode
Config2 is set to 0x00 (Ultra Low Power, 1x oversampling, 2.7uA @ 1Hz).
For beehive temperature monitoring, 1x oversampling provides adequate accuracy.
The power saving over High Resolution (8x) is approximately 10uA per reading,
which is material for a device running unattended in a remote location.

### I2C peripheral duty-cycled
The MCU I2C peripheral is initialised only when needed during start-up/read jobs,
then deinitialised straight after use. This reduces always-on peripheral
leakage and helps maximise sleep current performance on bare-metal targets.

### Four readings per satellite message
Readings are buffered in RAM and transmitted as a single message once four
hourly readings have been collected. This minimises satellite transmissions
which are the most power-expensive operation on the device.

### Strict 4-hour cadence with failure placeholders
The firmware keeps a strict hourly cadence even if a sensor read fails.
On a failed read, it stores `INT16_MIN` as a sentinel value in the hourly slot.
This ensures each message always represents a true 4-hour window (4 hourly slots),
rather than "4 successful reads" over an unknown duration.

### Calibration values cached at start-up
MI319 factory calibration values (`calibration_A`, `calibration_B`) are read once
at sensor initialisation and reused for each hourly sample. This removes two
I2C register reads per wake cycle, shortening active time and saving energy.

### Queue-safe retry (no 4-hour data loss)
If `ScheduleMessage()` cannot queue the payload (for example, queue full),
the 4-hour buffer is retained and retried on the next hourly wake.
No new sample is taken while an unsent 4-hour block is pending, preventing
overwrite and preserving chronological integrity of cumulative hourly data.

### Production logging disabled by default
Debug logging is compile-time gated via `BEEHIVE_ENABLE_LOG` (default `0`).
This avoids bringing in heavy `printf`/float formatting paths for production
builds, reducing code size and active CPU time.

### Packed message struct
The beehive_message_t struct uses __attribute__((packed)) to prevent compiler
padding. This ensures the byte layout is deterministic and matches the
server-side decoder exactly.

### Integer arithmetic (centidegrees)
Temperatures are stored as int16_t in centidegrees Celsius (e.g. 2350 = 23.50C).
This avoids floating point arithmetic in the firmware (reduces code size and
avoids soft-float overhead), while retaining two decimal places of resolution.

### Device ID validation on init
The firmware reads the MI319 ID register (always 0x74) on startup before
configuring or reading the sensor. If the ID does not match, no jobs are
scheduled. This prevents wasted transmissions if the sensor is absent or
the I2C bus is faulty.

### I2C initialisation
The firmware explicitly calls `I2CInit()` during start-up before communicating
with the MI319 sensor. If I2C init fails, jobs are not scheduled.

### Startup timing
The first sample is scheduled one hour after boot. This means the first
transmitted payload contains a full four-hour accumulation window.

## Message Format

| Field            | Type     | Size   | Description                          |
|------------------|----------|--------|--------------------------------------|
| sequence_number  | uint16_t | 2 bytes| Monotonic counter for ordering       |
| temperature[0]   | int16_t  | 2 bytes| Hour 1 reading in centidegrees C     |
| temperature[1]   | int16_t  | 2 bytes| Hour 2 reading in centidegrees C     |
| temperature[2]   | int16_t  | 2 bytes| Hour 3 reading in centidegrees C     |
| temperature[3]   | int16_t  | 2 bytes| Hour 4 reading in centidegrees C     |
| **Total**        |          | **10 bytes** |                              |

To convert to degrees Celsius on the server: temperature_degC = value / 100.0

INT16_MIN (-32768) is used as a sentinel value indicating a failed reading.

## Efficiency Notes (10-year battery target)

This implementation is power-aware but 10-year battery life depends on whole-system
budget, especially satellite transmission profile and sleep current of the full PCB.
The dominant energy cost is transmission, so reducing bytes and transmission count
has the largest impact after sleep-current optimisation.

### Packet size reduction (not implemented)

A potential optimisation is reducing telemetry packet size (for example, packed raw
sensor values or delta encoding). I have not implemented this in the current
submission. Reason: transmission energy-per-byte (or an equivalent airtime/power
model) was not available, so the battery-life benefit could not be quantified
properly against added firmware complexity and decoder changes.

## Delta Update Strategy

For device-side update efficiency, use Myriota system update primitives:
`SystemUpdateStart()`, `SystemUpdateXfer()`, and `SystemUpdateFinish()`.
To minimise update airtime and power:

- Use delta payloads generated server-side against a known firmware baseline.
- Include manifest metadata: target version, base version, size, and hash.
- Download/update in resumable chunks (aligned to larger chunk sizes where possible).
- Verify hash/signature before `SystemUpdateFinish()`.
- Fall back to a full image only if delta preconditions fail.

In addition to firmware deltas, consider payload-level deltas for telemetry
(e.g. base + per-hour deltas) if cloud decoder changes are acceptable.

## Building

With the SDK unpacked locally in `SDK-master`:

    make

### Build verification in this environment

The code compiles and links successfully with:

    make obj/beehive_monitor.elf ARM_TOOLCHAIN_PATH=C:/Users/man_a/armgcc

### Windows 11 packaging note

Packaging has been validated in a Windows 11 environment by building from a
path without spaces and running:

    make beehive_monitor.bin ARM_TOOLCHAIN_PATH=C:/Users/man_a/armgcc BUILD_WITH_NETWORKINFO=1

This produces `beehive_monitor.bin` successfully.
In this command, `BUILD_WITH_NETWORKINFO=1` skips the network-info merge step
and packages the application image directly, which avoids Windows shell/Python
merge issues in this environment.

## SDK References

- AppInit() and ScheduleJob() pattern: examples/hello_space
- I2C read/write API: I2CRead(), I2CWrite() from myriota_user_api.h
- Message transmission: ScheduleMessage(), HoursFromNow()

## Dependency Sources

- Flex SDK: [Myriota/Flex-SDK](https://github.com/Myriota/Flex-SDK)
- Flex reference applications: [Myriota/Flex-Reference-Applications](https://github.com/Myriota/Flex-Reference-Applications)
