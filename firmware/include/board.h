/*
 * Pin map and hardware constants (ESP32 DevKit, esp32dev).
 *
 * Power stage: VNH5019 H-bridge carrier, three-wire interface PWM + INA/INB.
 * The modelled motor draws 12 V / 2.4 ohm = 5 A at standstill, so the
 * bridge must handle that; a TB6612FNG has the same PWM + IN1/IN2 interface
 * but is rated 3.2 A peak and would only suit a smaller motor.
 *
 * None of the GPIOs is a boot-strapping pin, so the motor wiring cannot change
 * the ESP32 boot mode.
 */
#ifndef BOARD_H
#define BOARD_H

#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/pcnt.h>

/* H-bridge */
#define PIN_BRIDGE_PWM GPIO_NUM_25   /* VNH5019 PWM */
#define PIN_BRIDGE_INA GPIO_NUM_26   /* VNH5019 INA */
#define PIN_BRIDGE_INB GPIO_NUM_27   /* VNH5019 INB */
#define PIN_BRIDGE_DIAG GPIO_NUM_33  /* VNH5019 ENA/DIAGA + ENB/DIAGB, pulled up on the carrier */

/* Quadrature encoder (Hall sensors supplied with 3.3 V) */
#define PIN_ENC_A GPIO_NUM_18
#define PIN_ENC_B GPIO_NUM_19

/* 20 kHz is above the audible range and within the VNH5019 limit. With the
 * 80 MHz APB clock, 80e6 / 20e3 = 4000 timer ticks per period allow at
 * most 11 bits of duty resolution. */
#define BRIDGE_PWM_FREQ_HZ 20000u
#define BRIDGE_PWM_BITS LEDC_TIMER_11_BIT
#define BRIDGE_LEDC_MODE LEDC_HIGH_SPEED_MODE
#define BRIDGE_LEDC_TIMER LEDC_TIMER_0
#define BRIDGE_LEDC_CHANNEL LEDC_CHANNEL_0

/* PCNT: 16-bit counter that the hardware resets to 0 at +-ENC_PCNT_LIMIT. At
 * the 1 kHz loop rate the counter moves at most a few counts per sample, far
 * below the LIMIT/2 that the 32-bit extension (mc_encoder.h) tolerates. */
#define ENC_PCNT_UNIT PCNT_UNIT_0
#define ENC_PCNT_LIMIT 30000
/* Glitch filter in APB cycles: pulses shorter than 1.25 us are ignored. The
 * fastest real edge spacing (36 rad/s free-running speed, 1320 counts/rev)
 * is ~130 us. */
#define ENC_PCNT_FILTER 100u

#endif /* BOARD_H */
