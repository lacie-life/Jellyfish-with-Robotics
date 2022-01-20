#include "../../include/dynamixel_sdk/dynamixel_sdk.h"

int getch();

int kbhit(void);

void scan(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler1);

void write(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length, uint32_t value);

int read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length);

void dump(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t len);


