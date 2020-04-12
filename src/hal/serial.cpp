#include <stdint.h>

#include "../pt/pt.h"
#include "../hal/hal.h"
#include "../hal/serial.h"
#include "../modules/link.h"

uint32_t last_send_ms = 0;        // this is used for send interval
uint32_t current_send_ms = 0;     // current time is referenced a few times so refer to this variable 
uint16_t bytesSent;               // Serial.write() returns this - we should increase the default Serial buffer size so that the function does not block
uint8_t inByte;                   // store each byte read
int incoming_index = 0;           // index for array of bytes received as we are getting them one at a time

uint32_t watchdog_start_ms;       // save the watchdog start time
bool read_active = false;     // watchdog timer in progress (waiting for command/confirm packet)
bool watchdog_exceeded = false;   // flag for watchdog timer received. this is for link.cpp.
bool clear_input = false;         // if we get a bad packet (wrong sequence id or crc) set this to true and trash input buffer just before sending next data packet

// this will be used by module/link to send packets
uint16_t sequence_count = 0;      // this is the sequence count stored in the data packet and confirmed in the command packet. wrapping is fine as crc checks are done.
uint16_t last_sequence_count; // what to expect

// this is the storage for getting bytes Serial.readBytes(1), reading bytes one at a time. probably could use pointer, but used union command bytes to get one byte per call
// when the full packet is received this is put copied to the public command data structure.
typedef union command_packet_union{
command_packet_def command_packet;
uint8_t command_bytes[sizeof(command_packet_def)];
};

command_packet_union command_packet_u;

extern command_packet_def command_packet_from_serial;

int serialHalInit(void)
{
  SERIAL_UI.begin(BAUD_RATE);  // this function does not return anything
  // not sure how we can do something like while (!serial)
  // return MODULE_FAIL;
  do
  { SERIAL_UI.read();
  } while (SERIAL_UI.available() > 0);
  
#ifdef SERIAL_DEBUG
  SERIAL_DEBUG.begin(BAUD_RATE);
#endif  
  return HAL_OK;
}

int serialHalGetData(void)
{
  if (read_active != true)
  {
    return HAL_IN_PROGRESS;
  }
  // change to while if reading all the bytes currently available
  if (SERIAL_UI.available())
  {       
    SERIAL_UI.readBytes(&inByte, 1);
    
    command_packet_u.command_bytes[incoming_index] = inByte;
    
    incoming_index++;

    if (incoming_index >= sizeof(comm.public_command_packet))
    {
      // save a copy for other modules, but keep a reference in case the shared copy gets modified
      incoming_index = 0;
      read_active = false;
      watchdog_exceeded = false;

      memcpy((void *)&command_packet_from_serial, (void *)&command_packet_u.command_packet, sizeof(command_packet_from_serial));
#ifdef SERIAL_DEBUG
      SERIAL_DEBUG.print("sequence from Rpi: ");
      SERIAL_DEBUG.print(command_packet_from_serial.sequence_count, DEC);
      SERIAL_DEBUG.print(" ");
      SERIAL_DEBUG.println(command_packet_from_serial.crc, DEC);
      SERIAL_DEBUG.println(" ");
#endif      
      return HAL_OK;
    } 
  }
  if (read_active == true)
  {
    if ((millis() - watchdog_start_ms) > WATCHDOG_MS)
    {
#ifdef SERIAL_DEBUG
      SERIAL_DEBUG.print("!!!Watchdog timeout, bytes received count: ");
      SERIAL_DEBUG.println(incoming_index, DEC);
      SERIAL_DEBUG.print("first byte (sequence count): ");
      SERIAL_DEBUG.println(command_packet_u.command_packet.sequence_count, DEC);
#endif        
      read_active = false; 
      watchdog_exceeded = true; 
      incoming_index = 0;
      return HAL_OK;        
    }
  } // if serial.available
  return HAL_IN_PROGRESS;
}

int serialHalSendData()
{
  current_send_ms = millis();
  if ((current_send_ms - last_send_ms) >= SEND_INTERVAL_MS && read_active != true)
  {
    if (clear_input == true)
    {
      clear_input = false;
      do
      { SERIAL_UI.read();
      } while (SERIAL_UI.available() > 0);
    }
#ifdef SERIAL_DEBUG
    SERIAL_DEBUG.print("Sent to Rpi sequence: ");
    SERIAL_DEBUG.print(update_crc_data_packet.sequence_count, DEC);
    SERIAL_DEBUG.print(" CRC: ");
    SERIAL_DEBUG.print(update_crc_data_packet.crc, DEC);
    SERIAL_DEBUG.println(" ");
#endif 
       
    bytesSent = SERIAL_UI.write((byte *)&update_crc_data_packet, sizeof(update_crc_data_packet));
    
    if (bytesSent != sizeof(update_crc_data_packet)) {
      // handle error
    }
       
    last_sequence_count = sequence_count;
    sequence_count++;
    last_send_ms = current_send_ms;
    watchdog_start_ms = millis();
    incoming_index = 0; // set this for incoming bytes
#ifdef SERIAL_DEBUG
    SERIAL_DEBUG.print("watchdog_start_ms initialized: ");
    SERIAL_DEBUG.println(watchdog_start_ms, DEC);
#endif    
    read_active = true;
    return HAL_OK;
  }
  return HAL_IN_PROGRESS;
}

