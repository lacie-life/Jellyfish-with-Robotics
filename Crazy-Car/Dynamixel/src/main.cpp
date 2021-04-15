#include <iostream>
#include <string.h>
#include <unistd.h>
#include <thread>
#include <chrono>
#include "../../include/dynamixel.h"

#define PROTOCOL_VERSION1 1.0 //PROTOCOL_VERSION1
#define DEVICE_NAME "/dev/ttyUSB1"
#define BAUDRATE  1000000

void setup_(dynamixel::PortHandler *portHandler,dynamixel::PacketHandler *packetHandler1);

int main()
{
//	std::cout << "hello";
   	dynamixel::PacketHandler *packetHandler1 = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION1);
	char *deviceName = (char*)DEVICE_NAME;
	int value = 0;
	int id = 0;
	
	int matrix[14][4] =	{
		512,0,0,80,
		512,100,150,150,
		512,130,310,340,
		512,180,260,450,
		512,200,235,580,
		512,230,235,675,
		370,230,235,675,
		620,230,235,675,
		512,230,235,675,
		512,200,235,580,
		512,180,260,450,
		512,130,310,340,
		512,100,150,150,
		512,0,10,80
	};
	int matrix2[13][4] =	{
		512,0,0,80,
		512,150,380,600,
		512,130,500,600,
		512,130,580,580,
		512,130,380,350,
		512,130,580,580,
		512,130,380,350,
		512,130,580,580,
		512,130,380,350,
		512,130,580,580,
		512,130,500,600,
		512,150,380,600,
		512,0,0,80
	};
	int matrix3[3][4] =	{
		512,0,0,80,
		512,100,100,100,
		512,555,393,42,
	};
	int matrix4[3][4] =	{
		512,300,200,100,
		512,100,100,100,
		512,0,0,80
	};
	int cmd = 0;
	deviceName = strdup(deviceName);
	dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(deviceName);
	setup_(portHandler,packetHandler1);
/*	while(1)
	{
		std::cin >> id >> value;
		write(portHandler,packetHandler1,id,30,2,value);
	}*/
	while(1)
	{
		std::cin  >> cmd;
		switch(cmd)
		{
			case 1:
			{
				scan(portHandler,packetHandler1);
				break;
			}
			case 2:// cay 1
			{
				for(int i=0; i<14; i++)
				{
					//std::cout << "\n";
					std::cout << matrix[i][0] << " ";
					write(portHandler,packetHandler1,1,30,2,matrix[i][0]);
					std::cout << matrix[i][1] << " " ;
					
					write(portHandler,packetHandler1,2,30,2,matrix[i][1]);
					std::cout << matrix[i][2] << "  ";
					write(portHandler,packetHandler1,3,30,2,matrix[i][2]);
					std::cout << matrix[i][3] << std::endl;
					write(portHandler,packetHandler1,4,30,2,matrix[i][3]);
					std::cout << i << " " << std::endl;
					int check[4] = {1,1,1,1};
					while(!(check[0]==0&&check[1]==0&&check[2]==0&&check[3]==0))
					{
					//	std::cout << "check: " << check[0] << std::endl;
						check[0] = read(portHandler,packetHandler1,1,46,1);
						usleep(1000);
					//	std::cout << "check: " << check[1] << std::endl;
						check[1] = read(portHandler,packetHandler1,2,46,1);
						usleep(1000);
					//	std::cout << "check: " << check[2] << std::endl;
						check[2] = read(portHandler,packetHandler1,3,46,1);
						usleep(1000);	
					//	std::cout << "check: " << check[3] << std::endl;		
						check[3] = read(portHandler,packetHandler1,4,46,1);
						usleep(1000);
					}
				}
				break;
			}
			case 3:// gat dau
			{
				for(int i=0; i<13; i++)
				{
					//std::cout << "\n";
					std::cout << matrix2[i][0] << " ";
					write(portHandler,packetHandler1,1,30,2,matrix2[i][0]);
					std::cout << matrix2[i][1] << " " ;
					
					write(portHandler,packetHandler1,2,30,2,matrix2[i][1]);
					std::cout << matrix2[i][2] << "  ";
					write(portHandler,packetHandler1,3,30,2,matrix2[i][2]);
					std::cout << matrix2[i][3] << std::endl;
					write(portHandler,packetHandler1,4,30,2,matrix2[i][3]);
					std::cout << i << " " << std::endl;
					int check[4] = {1,1,1,1};
					while(!(check[0]==0&&check[1]==0&&check[2]==0&&check[3]==0))
					{
					//	std::cout << "check: " << check[0] << std::endl;
						check[0] = read(portHandler,packetHandler1,1,46,1);
						usleep(1000);
					//	std::cout << "check: " << check[1] << std::endl;
						check[1] = read(portHandler,packetHandler1,2,46,1);
						usleep(1000);
					//	std::cout << "check: " << check[2] << std::endl;
						check[2] = read(portHandler,packetHandler1,3,46,1);
						usleep(1000);	
					//	std::cout << "check: " << check[3] << std::endl;		
						check[3] = read(portHandler,packetHandler1,4,46,1);
						usleep(1000);
					}
				}
				break;
			}
			case 4:
			{
				for(int i=0; i<3; i++)
				{
					//std::cout << "\n";
					std::cout << matrix3[i][0] << " ";
					write(portHandler,packetHandler1,1,30,2,matrix3[i][0]);
					std::cout << matrix3[i][1] << " " ;
					
					write(portHandler,packetHandler1,2,30,2,matrix3[i][1]);
					std::cout << matrix3[i][2] << "  ";
					write(portHandler,packetHandler1,3,30,2,matrix3[i][2]);
					std::cout << matrix3[i][3] << std::endl;
					write(portHandler,packetHandler1,4,30,2,matrix3[i][3]);
					std::cout << i << " " << std::endl;
					int check[4] = {1,1,1,1};
					while(!(check[0]==0&&check[1]==0&&check[2]==0&&check[3]==0))
					{
					//	std::cout << "check: " << check[0] << std::endl;
						check[0] = read(portHandler,packetHandler1,1,46,1);
						usleep(1000);
					//	std::cout << "check: " << check[1] << std::endl;
						check[1] = read(portHandler,packetHandler1,2,46,1);
						usleep(1000);
					//	std::cout << "check: " << check[2] << std::endl;
						check[2] = read(portHandler,packetHandler1,3,46,1);
						usleep(1000);	
					//	std::cout << "check: " << check[3] << std::endl;		
						check[3] = read(portHandler,packetHandler1,4,46,1);
						usleep(1000);
					}
				}
				break;
			}
			case 5:
			{
				write(portHandler,packetHandler1,1,32,2,100);
				write(portHandler,packetHandler1,2,32,2,100);
				write(portHandler,packetHandler1,3,32,2,100);
				write(portHandler,packetHandler1,4,32,2,100);
				for(int i=0; i<3; i++)
				{
					//std::cout << "\n";
					std::cout << matrix4[i][0] << " ";
					write(portHandler,packetHandler1,1,30,2,matrix4[i][0]);
					std::cout << matrix4[i][1] << " " ;
					
					write(portHandler,packetHandler1,2,30,2,matrix4[i][1]);
					std::cout << matrix4[i][2] << "  ";
					write(portHandler,packetHandler1,3,30,2,matrix4[i][2]);
					std::cout << matrix4[i][3] << std::endl;
					write(portHandler,packetHandler1,4,30,2,matrix4[i][3]);
					std::cout << i << " " << std::endl;
					int check[4] = {1,1,1,1};
					while(!(check[0]==0&&check[1]==0&&check[2]==0&&check[3]==0))
					{
					//	std::cout << "check: " << check[0] << std::endl;
						check[0] = read(portHandler,packetHandler1,1,46,1);
						usleep(1000);
					//	std::cout << "check: " << check[1] << std::endl;
						check[1] = read(portHandler,packetHandler1,2,46,1);
						usleep(1000);
					//	std::cout << "check: " << check[2] << std::endl;
						check[2] = read(portHandler,packetHandler1,3,46,1);
						usleep(1000);	
					//	std::cout << "check: " << check[3] << std::endl;		
						check[3] = read(portHandler,packetHandler1,4,46,1);
						usleep(1000);
					}
				}
				break;
			}
			case 6:
			{
				for (int i=1; i<5; i++)
				{
					std::cout << read(portHandler,packetHandler1,i,36,2) << " ";
				}
				
				break;
			}
		}	
	}
    return 0;
}
void setup_(dynamixel::PortHandler *portHandler,dynamixel::PacketHandler *packetHandler1)
{
	
	if(portHandler->openPort())
	{
		portHandler->getBaudRate();
	}
	portHandler->setBaudRate(BAUDRATE);
	write(portHandler,packetHandler1,1,32,2,150);
	write(portHandler,packetHandler1,2,32,2,150);
	write(portHandler,packetHandler1,3,32,2,150);
	write(portHandler,packetHandler1,4,32,2,150);
}

