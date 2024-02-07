
/* 
* The definitions below must precede the NMEA2000 includes to ensure correct selection of CAN library. 
*/
#define ESP32_CAN_TX_PIN GPIO_NUM_4 // If you use ESP32 and do not have TX on default IO 16, uncomment this and and modify definition to match your CAN TX pin.
#define ESP32_CAN_RX_PIN GPIO_NUM_2 // If you use ESP32 and do not have RX on default IO 4, uncomment this and and modify definition to match your CAN TX pin.

#include <Preferences.h>
#include <NMEA2000_CAN.h>  // This will automatically choose right CAN library and create suitable NMEA2000 object
#include <N2kMessages.h>
#include <N2kMaretron.h>


#define N2K_MANUFACTURER_ID "1" // Manufacturer's Model serial code
#define N2K_PRODUCT_ID 401 // Manufacturer's product code
#define N2K_MODEL_ID "MCC400" // Manufacturer's Model ID
#define N2K_SOFTWARE_VERSION "0.0.1-1 (2023-01-01)" // Manufacturer's Software version code
#define N2K_HARDWARE_VERSION "0.0.1-1 (2023-01-01)" // Manufacturer's Model version
#define N2k_DEVICE_FUNCTION 160 // Device function=Device that brings information from an engine used for propulsion onto the NMEA 2000 network
#define N2k_DEVICE_CLASS 50 // Device class=Propulsion
#define N2k_DEVICE_INDUSTRY_GROUP 2002 // From code list on 

uint32_t getDeviceId(void);                         // get a unique identifier for the processor from its MAC address
void    getNodeAddress(void);                       // Get the node address from the chip id
void    n2KInit(void);                              // initialize the NMEA2000 library
void    HandleNMEA2000Msg(const tN2kMsg &N2kMsg);   // NMEA 2000 message handler

#define ID getDeviceId()

uint32_t timestamp = 0;

#define HOUSE_BATTERY_ID 1
#define ALT_TEMP_DS18B20 0 // The one-wire device that measures the alternator temp
int NodeAddress;            // N2K device node address
Preferences preferences;    // Nonvolatile storage on ESP32 - to save last N2K device node address

// Set the information for other bus devices, which messages we support
const unsigned long TransmitMessages[] PROGMEM = { 0 }; 

typedef struct {
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg); 
} tNMEA2000Handler;

tNMEA2000Handler NMEA2000Handlers[]={{0,0}};

// *****************************************************************************
// Callback for NMEA2000 open. This will be called when NMEA2000 bus communication starts.
// See NMEA2000.SetOnOpen(OnN2kOpen); on setup()
void OnN2kOpen() {
  // Start schedulers now.
  Serial.println("NMEA2000 instance is open");
    /* 
  If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  */
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode, NodeAddress);  
}

/// @brief Generates a unique id from the ESP32 WiFi MAC Address.
/// @return A unique id as integer.
uint32_t getDeviceId(){
  
  // variables for chip id
  uint8_t chipid[6];
  uint32_t id = 0, i;  
  // Generate unique number from chip id
  esp_read_mac(chipid, ESP_MAC_WIFI_STA);
  for (i = 0; i < 6; i++) id += (chipid[i] << (7 * i));
  Serial.println("ID: " + String(id));
  return id;
}

/// @brief Retrieves the last node address from preferences, if it was saved, otherwise
/// uses the ID value (unique id from the ESP32 WiFi MAC Address).
void getNodeAddress(){ 
  preferences.begin("nvs", false);                          // Open nonvolatile storage (nvs)
  NodeAddress = preferences.getInt("LastNodeAddress", ID);  // Read stored last NodeAddress, default 34
  preferences.end();
  Serial.printf("\nN2K-NodeAddress=%d\n", NodeAddress);
    
}

void n2KInit(){
  NMEA2000.SetN2kCANSendFrameBufSize(150);
  NMEA2000.SetN2kCANReceiveFrameBufSize(150);
  NMEA2000.SetProductInformation(N2K_MANUFACTURER_ID, 
                                 N2K_PRODUCT_ID, 
                                 N2K_MODEL_ID, 
                                 N2K_SOFTWARE_VERSION, 
                                 N2K_HARDWARE_VERSION 
                                );
  NMEA2000.SetDeviceInformation(ID, // Unique number. Use e.g. Serial number.
                                N2k_DEVICE_FUNCTION, 
                                N2k_DEVICE_CLASS,  
                                N2k_DEVICE_INDUSTRY_GROUP 
                               );
  NMEA2000.EnableForward(false); 
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.ExtendTransmitMessages(TransmitMessages);
  NMEA2000.SetOnOpen(OnN2kOpen);
  NMEA2000.Open();
}

// NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  int iHandler;
  // Find handler
  // Serial.println("Handling message " + String(N2kMsg.PGN) + " from NMEA2000 bus");
  for ( iHandler=0; 
        NMEA2000Handlers[iHandler].PGN!=0 && !(N2kMsg.PGN==NMEA2000Handlers[iHandler].PGN); 
        iHandler++);
  if (NMEA2000Handlers[iHandler].PGN!=0) {
    NMEA2000Handlers[iHandler].Handler(N2kMsg); 
  } else {
    // Serial.println("NMEA2000 handler not found");
  }
}