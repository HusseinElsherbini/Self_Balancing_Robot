#ifndef SMART_PORT_H
#define SMART_PORT_H

#include "stdint.h"
#include "uart.h"


// Value type definitions for different measurements
typedef enum {
    SPORT_TYPE_VARIO     = 0x0110,  // Vertical speed
    SPORT_TYPE_CURRENT   = 0x0200,  // Current draw
    SPORT_TYPE_VOLTAGE   = 0x0210,  // Voltage measurement
    SPORT_TYPE_RPM       = 0x0500,  // Motor RPM
    SPORT_TYPE_CAPACITY  = 0x0600,  // Battery capacity used
    SPORT_TYPE_ALTITUDE  = 0x0610,  // Altitude
    SPORT_TYPE_GPS_SPEED = 0x0830   // GPS ground speed
} sport_value_type_t;

// Structure for receiving poll requests
typedef struct {
    uint8_t header;     // Should be SPORT_START_POLL (0x7E)
    uint8_t sensor_id;  // Which sensor is being polled
}sport_poll_frame_t;

// Structure for sending sensor data
typedef struct {
    uint8_t header;        // Should be SPORT_START_DATA (0x10)
    uint16_t value_type;   // Type of data being sent (from sport_value_type_t)
    union {
        int32_t  signed_value;   // For values that can be negative (vario, altitude)
        uint32_t unsigned_value; // For values that are always positive
    } data;
    uint8_t checksum;      // XOR of bytes 1-6
} sport_data_frame_t;

// Structure to hold a sensor's configuration
typedef struct {
    uint8_t sensor_id;            // ID this sensor responds to
    sport_value_type_t *types;    // Array of supported value types
    uint8_t num_types;            // Number of supported value types
    uint8_t current_type_index;   // Which value type to send next
} sport_sensor_config_t;

// SmartPort data structure
typedef struct {
    sport_sensor_config_t *sensors; // Array of configured sensors
    uint8_t num_sensors;          // Number of configured sensors
    uint8_t rx_buffer[2];         // Buffer for receiving frames
    uint8_t rx_index;
    uint8_t tx_buffer[8];         // Buffer for transmitting frames
    uint8_t tx_index;
    uint32_t last_tx_time;       // Timestamp of last transmission
    UART_HandleTypeDef *uartHandle;
} SmartPort_HandleTypeDef;

// SmartPort initialization status
typedef enum {
    SMARTPORT_OK = 0,
    SMARTPORT_ERROR_NULL_HANDLE,
    SMARTPORT_ERROR_GPIO,
    SMARTPORT_ERROR_UART,
    SMARTPORT_ERROR_CLOCK
} SmartPort_StatusTypeDef;

extern SmartPort_HandleTypeDef smart_port_handle;
extern UART_HandleTypeDef smartPortUart;

// SmartPort protocol constants
#define SPORT_START_POLL    0x7E    // Start of polling frame
#define SPORT_START_DATA    0x10    // Start of data frame

// Common sensor IDs (add more as needed)
#define SPORT_SENSOR_ID_1   0x00    // Known working sensor ID
#define SPORT_SENSOR_ID_2   0xA1    // Known working sensor ID



#endif // SMART_PORT_H