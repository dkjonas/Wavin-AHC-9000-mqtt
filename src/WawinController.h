#include <inttypes.h>


class WawinController
{
  public:
    WawinController(uint8_t pin, bool swapSerialPins, uint16_t timeout_ms);
    bool readRegisters(uint8_t category, uint8_t page, uint8_t index, uint8_t count, uint16_t *reply);
    bool writeRegister(uint8_t category, uint8_t page, uint8_t index, uint16_t value);

    static const uint8_t CATEGORY_MAIN =        0x00;
    static const uint8_t CATEGORY_ELEMENTS =    0x01;
    static const uint8_t CATEGORY_PACKED_DATA = 0x02;
    static const uint8_t CATEGORY_CHANNELS =    0x03;
    static const uint8_t CATEGORY_RELAYS =      0x04;
    static const uint8_t CATEGORY_CLOCK =       0x05;
    static const uint8_t CATEGORY_SCHEDULES =   0x06;
    static const uint8_t CATEGORY_INFO =        0x07;
      
    static const uint8_t ELEMENTS_AIR_TEMPERATURE = 0x04;
    static const uint8_t ELEMENTS_BATTERY_STATUS  = 0x0A;
    static const uint8_t ELEMENTS_SYNC_GROUP      = 0x0B;
        
    static const uint8_t PACKED_DATA_MANUAL_TEMPERATURE = 0x00;

    static const uint8_t CHANNELS_TIMER_EVENT = 0x00;
    static const uint8_t CHANNELS_TIMER_EVENT_OUTP_ON_MASK = 0x10;
        
  private:
    uint8_t txEnablePin;
    uint16_t recieveTimeout_ms;
    void transmit(uint8_t *data, uint8_t lenght);
    bool recieve(uint16_t *reply, uint8_t cmdtype);
    unsigned int calculateCRC(unsigned char *frame, unsigned char bufferSize);

    const uint8_t MODBUS_DEVICE = 0x01;
    const uint8_t MODBUS_READ_REGISTER =  0x43;
    const uint8_t MODBUS_WRITE_REGISTER = 0x44;

    // Largest page contains 22 registers of 2 bytes + 5 bytes header
    const uint8_t RECIEVE_BUFFER_SIZE =  22 * 2 + 5;
};
