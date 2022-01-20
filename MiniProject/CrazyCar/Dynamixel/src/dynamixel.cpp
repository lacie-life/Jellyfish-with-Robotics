//
// Created by lacie on 09/09/2019.
//

#include "../../include/dynamixel.h"

int getch()
{
//#if defined(__linux__) || defined(__APPLE__)
  //  struct termios oldt, newt;
   // int ch;
  //  tcgetattr(STDIN_FILENO, &oldt);
  //  newt = oldt;
  //  newt.c_lflag &= ~(ICANON | ECHO);
  //  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  //  ch = getchar();
  //  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  //  return ch;
//#elif defined(_WIN32) || defined(_WIN64)
  //  return _getch();
//#endif
}

int kbhit(void)
{
//#if defined(__linux__) || defined(__APPLE__)
  //  struct termios oldt, newt;
//    int ch;
//    int oldf;
//
 //   tcgetattr(STDIN_FILENO, &oldt);
  //  newt = oldt;
 //   newt.c_lflag &= ~(ICANON | ECHO);
 //   tcsetattr(STDIN_FILENO, TCSANOW, &newt);
//    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
//    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

//    ch = getchar();

 //   tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
 //   fcntl(STDIN_FILENO, F_SETFL, oldf);

//    if (ch != EOF)
 //   {
  //      ungetc(ch, stdin);
   //     return 1;
  //  }

    return 0;
//#elif defined(_WIN32) || defined(_WIN64)
  //  return _kbhit();
//#endif
}

void scan(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler1)
{
    uint8_t dxl_error;
    uint16_t dxl_model_num;

    fprintf(stderr, "\n");
    fprintf(stderr, "Scan Dynamixel Using Protocol 1.0\n");
    for (int id = 1; id < 253; id++)
    {
        if (packetHandler1-> ping(portHandler, id, &dxl_model_num, &dxl_error)== COMM_SUCCESS)
        {
            fprintf(stderr, "\n                                          ... SUCCESS \r");
            fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", id, dxl_model_num);
        }
        else
            fprintf(stderr, ".");

        if (kbhit())
        {
            char c = getch();
            if (c == 0x1b)
                break;
        }
    }
    fprintf(stderr, "\n\n");

    /*fprintf(stderr, "Scan Dynamixel Using Protocol 2.0\n");
    for (int id = 1; id < 253; id++)
    {
        if (packetHandler2-> ping(portHandler, id, &dxl_model_num, &dxl_error)== COMM_SUCCESS)
        {
            fprintf(stderr, "\n                                          ... SUCCESS \r");
            fprintf(stderr, " [ID:%.3d] Model No : %.5d \n", id, dxl_model_num);
        }
        else
        {
            fprintf(stderr, ".");
        }

        if (kbhit())
        {
            char c = getch();
            if (c == 0x1b) break;
        }
    }
    fprintf(stderr, "\n\n");*/
}

void write(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length, uint32_t value)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    if (length == 1)
    {
        dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, addr, (uint8_t)value, &dxl_error);
    }
    else if (length == 2)
    {
        dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, addr, (uint16_t)value, &dxl_error);
    }
    else if (length == 4)
    {
        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, addr, (uint32_t)value, &dxl_error);
    }

    if (dxl_comm_result == COMM_SUCCESS)
    {
        if (dxl_error != 0) printf("%s\n", packetHandler->getRxPacketError(dxl_error));
        //fprintf(stderr, "\n Success to write\n\n");
    }
    else
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        fprintf(stderr, "\n Fail to write! \n\n");
    }
}

int read(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t length)
{
    uint8_t dxl_error = 0;
    int     dxl_comm_result = COMM_TX_FAIL;

    int8_t  value8    = 0;
    int16_t value16   = 0;
    int32_t value32   = 0;


    if (length == 1)
    {
        dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, addr, (uint8_t*)&value8, &dxl_error);
    }
    else if (length == 2)
    {
        dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, addr, (uint16_t*)&value16, &dxl_error);
    }
    else if (length == 4)
    {
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, addr, (uint32_t*)&value32, &dxl_error);
    }

    if (dxl_comm_result == COMM_SUCCESS)
    {
        if (dxl_error != 0) printf("%s\n", packetHandler->getRxPacketError(dxl_error));

        if (length == 1)
        {
       //     fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint8_t)value8, value8);
            return value8;
        }
        else if (length == 2)
        {
            fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint16_t)value16, value16);
            return value16;
        }
        else if (length == 4)
        {
            fprintf(stderr, "\n READ VALUE : (UNSIGNED) %u , (SIGNED) %d \n\n", (uint32_t)value32, value32);
            return value32;
        }
    }
    else
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        fprintf(stderr, "\n Fail to read! \n\n");
        return COMM_RX_FAIL;

    }
}

void dump(dynamixel::PortHandler *portHandler, dynamixel::PacketHandler *packetHandler, uint8_t id, uint16_t addr, uint16_t len)
{
    uint8_t  dxl_error       = 0;
    int      dxl_comm_result = COMM_TX_FAIL;
    uint8_t *data            = (uint8_t*)calloc(len, sizeof(uint8_t));

    dxl_comm_result = packetHandler->readTxRx(portHandler, id, addr, len, data, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS)
    {
        if (dxl_error != 0)
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));

        if (id != BROADCAST_ID)
        {
            fprintf(stderr, "\n");
            for (int i = addr; i < addr+len; i++)
                fprintf(stderr, "ADDR %.3d [0x%.4X] :     %.3d [0x%.2X] \n", i, i, data[i-addr], data[i-addr]);
            fprintf(stderr, "\n");
        }
    }
    else
    {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        fprintf(stderr, "\n Fail to read! \n\n");
    }

    free(data);
}
