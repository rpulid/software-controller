
#include "hal/alarm.h"
#include "hal/watchdog.h"

#include "modules/link.h"
#include "modules/module.h"
#include "modules/sensors.h"
#include "modules/control.h"
#include "modules/parameters.h"

#include "util/alarm.h"

#define DEBUG
#define DEBUG_MODULE "main"
#include "util/debug.h"

void mainSetup(void)
{
  DEBUG_BEGIN;
  
  // TODO: cleanup prints
  DEBUG_PRINT("setup");

  // TODO: Handle failure conditions
  controlModuleInit();
  sensorsModuleInit();
  parametersModuleInit();
  linkModuleInit();
}

void mainLoop(void)
{
  // TODO: Clean up prints
  DEBUG_PRINT_EVERY(10000, "in main loop");
  // Run all modules in RR; take specified actions in the event of failure
  
  if (controlModuleRun() != MODULE_OK) {
    // TODO: control module exited, trigger severe error
  }
  
  if (sensorsModuleRun() != MODULE_OK) {
    // TODO: sensor module exited, trigger severe error  
  }

  if (parametersModuleRun() != MODULE_OK) {
    // TODO: parameters module exited, attempt recovery
  }

  if (linkModuleRun() != MODULE_OK) {
    // TODO: link module exited, attempt recovery
  }

  // Scan through all alarms, looking for any set warning that need to be addressed
  struct alarmProperties properties = {0};
  if (alarmCheckAll(&properties)) {
    // At least one alarm has fired, determine correct actions
    if (!properties.preventWatchdog) {
      watchdogHalReset();
    }
    
    switch (properties.priority) {
      case ALARM_PRIORITY_SEVERE:
      alarmHalRing(ALARM_HAL_CONSTANT);
      break;
      case ALARM_PRIORITY_HIGH:
      alarmHalRing(ALARM_HAL_1HZ);
      break;
      case ALARM_PRIORITY_MODERATE:
      alarmHalRing(ALARM_HAL_0_25HZ);
      break;
      case ALARM_PRIORITY_LOW:
      alarmHalRing(ALARM_HAL_OFF);
      break;
    }
  } else {
    watchdogHalReset();
    alarmHalRing(ALARM_HAL_OFF);
  }
}