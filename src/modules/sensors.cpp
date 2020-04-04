//
// Sensor Module
//

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>

#include "../pt/pt.h"

#include "../hal/sensor/airflow.h"
#include "../hal/sensor/battery.h"
#include "../hal/sensor/pressure.h"

#include "../modules/module.h"
#include "../modules/control.h"
#include "../modules/sensors.h"

#define PRESSURE_WINDOW 4

// TODO: Define these thresholds accurately
#define PEEP_PRESSURE_FLAT_THRESHOLD 20

#define INHALATION_DETECTION_ABSOLUTE_THRESHOLD 0
#define INHALATION_DETECTION_PEEP_THRESHOLD     100

#define max(a, b) ((a) > (b)) ? (a) : (b)

// Public variables
struct sensors sensors;

// Private Variables
static struct pt sensorsThread;
static struct pt sensorsPressureThread;
static struct pt sensorsAirFlowThread;
static struct pt sensorsBatteryThread;

static PT_THREAD(sensorsPressureThreadMain(struct pt* pt))
{
  static int16_t currentMaxPressure = INT16_MIN;
  static bool setPeakPressure = false;
  static int16_t plateauPressureSum = 0;
  static int8_t plateauPressureSampleCount = 0;
  static int16_t previousPressure[PRESSURE_WINDOW];

  PT_BEGIN(pt);

  PT_WAIT_UNTIL(pt, pressureSensorHalFetch() != HAL_IN_PROGRESS);

  int16_t rawPressure;
  pressureSensorHalGetValue(&rawPressure);
  
  // TODO: any filtering needed on the raw pressure readings
  int16_t pressure = rawPressure;
  
  // Shift in previous pressures
  for (int i = PRESSURE_WINDOW - 1; i > 0; i--) {
    previousPressure[i] = previousPressure[i - 1];
  }
  previousPressure[0] = pressure;
  
  // Derive Peak Pressure from pressure readings; updating the public value upon
  // entry into CONTROL_HOLD_IN state
  if ((control.state == CONTROL_HOLD_IN) && !setPeakPressure) {
    sensors.peakPressure = currentMaxPressure;
    currentMaxPressure = INT16_MIN;
    setPeakPressure = true;
  } else {
    currentMaxPressure = max(currentMaxPressure, pressure);
    if (control.state == CONTROL_EXHALATION) {
      setPeakPressure = false;
    }
  }
  
  // Derive Plateau Pressure from pressure readings during hold in; updating
  // public value upon entry into CONTROL_EXHALATION state
  if (control.state == CONTROL_HOLD_IN) {
    plateauPressureSum += pressure;
    plateauPressureSampleCount++;
  } else if ((control.state == CONTROL_EXHALATION) &&
             (plateauPressureSampleCount > 0)) {
    sensors.plateauPressure = plateauPressureSum / plateauPressureSampleCount;
    plateauPressureSum = 0;
    plateauPressureSampleCount = 0;
  }
  
  // Derive PEEP Pressure from pressure readings during exhalation after values
  // have "stabilized" (ie, the difference of any one point from their average
  // is less than some threshold)
  if (control.state == CONTROL_EXHALATION) {
    int32_t sum = 0;
    for (int i = 0; i < PRESSURE_WINDOW; i++) {
      sum += previousPressure[i];
    }
    
    int16_t average = sum / PRESSURE_WINDOW;
    
    bool flat = true;
    for (int i = 0; i < PRESSURE_WINDOW; i++) {
      if (abs(average - previousPressure[i]) > PEEP_PRESSURE_FLAT_THRESHOLD) {
        flat = false;
      }
    }
    
    // TODO: determine if we want this reading to update so long as it can, or
    // measure once per breath, like other derived pressures
    if (flat) {
      sensors.peepPressure = pressure;
    }
  }
  
  // Derive inhalation detection from pressure readings during exhalation by
  // looking for a dip in pressure below the PEEP threshold (or below an
  // absolute pressure threshold)
  if (control.state == CONTROL_EXHALATION) {
    if ((pressure < INHALATION_DETECTION_ABSOLUTE_THRESHOLD) ||
        (sensors.peepPressure - pressure > INHALATION_DETECTION_PEEP_THRESHOLD)) {
      sensors.inhalationDetected = true;
    }
  } else if (control.state == CONTROL_INHALATION) {
    sensors.inhalationDetected = false;
  }

  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(sensorsAirFlowThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  
  // TODO: airflow sensor

  PT_RESTART(pt);
  PT_END(pt);
}

static PT_THREAD(sensorsBatteryThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);
  
  // TODO: battery sensor
  
  PT_RESTART(pt);
  PT_END(pt);
}

PT_THREAD(sensorsThreadMain(struct pt* pt))
{
  PT_BEGIN(pt);

  if (!PT_SCHEDULE(sensorsPressureThreadMain(&sensorsPressureThread))) {
    PT_EXIT(pt);
  }

  if (!PT_SCHEDULE(sensorsAirFlowThreadMain(&sensorsAirFlowThread))) {
    PT_EXIT(pt);
  }

  if (!PT_SCHEDULE(sensorsBatteryThreadMain(&sensorsBatteryThread))) {
    PT_EXIT(pt);
  }

  PT_RESTART(pt);
  PT_END(pt);
}

int sensorsModuleInit(void)
{
  // TODO: Improve error propagation for all hal init failures
  if (pressureSensorHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }
  if (airflowSensorHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }
  if (batterySensorHalInit() != HAL_OK) {
    return MODULE_FAIL;
  }

  PT_INIT(&sensorsThread);
  PT_INIT(&sensorsPressureThread);
  PT_INIT(&sensorsAirFlowThread);
  PT_INIT(&sensorsBatteryThread);
}

int sensorsModuleRun(void)
{
  return (PT_SCHEDULE(sensorsThreadMain(&sensorsThread))) ? MODULE_OK : MODULE_FAIL;
}
