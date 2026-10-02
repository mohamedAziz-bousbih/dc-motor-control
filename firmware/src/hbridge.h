/*
 * Voltage output through an H-bridge with PWM + two direction inputs.
 *
 * PWM goes to the bridge's PWM input, the sign of the voltage to INA/INB.
 * During the PWM off-time the bridge brakes through its low-side switches
 * (slow decay), so the average terminal voltage is duty * V_supply as long
 * as the current does not drop to zero within a PWM period. That is the
 * averaged actuator model used in the simulations.
 */
#ifndef HBRIDGE_H
#define HBRIDGE_H

#include <stdbool.h>

/* Configures LEDC and the GPIOs; the bridge starts with 0 V output. */
bool hbridge_init(float v_supply);

/* Applies the voltage u (clamped to +-v_supply; NaN or inf gives 0 V). */
void hbridge_set_voltage(float u);

/* true while the bridge reports a fault (over-temperature, short circuit,
 * under-voltage) by pulling its DIAG line low. */
bool hbridge_fault(void);

#endif /* HBRIDGE_H */
