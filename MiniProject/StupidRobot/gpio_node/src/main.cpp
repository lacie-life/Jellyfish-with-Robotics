#include <iostream>

#include <chrono> 
#include <thread>

#include <signal.h>

#include <JetsonGPIO.h>

using namespace std;

// Pin Definitions
int output_pin = 15; // BOARD pin 18, BCM pin 24

bool end_this_program = false;

inline void delay(int s){
	this_thread::sleep_for(chrono::seconds(s));
}

void signalHandler (int s){
	end_this_program = true;
}


int main(){
	// When CTRL+C pressed, signalHandler will be called
	signal(SIGINT, signalHandler);

	// Pin Setup. 
	GPIO::setmode(GPIO::BCM);
	// set pin as an output pin with optional initial state of HIGH
	GPIO::setup(output_pin, GPIO::OUT, GPIO::HIGH);

	cout << "Strating demo now! Press CTRL+C to exit" << endl;
	int curr_value = GPIO::HIGH;

	while(!end_this_program){
		delay(3);
		// Toggle the output every second
		cout << "Outputting " << curr_value << " to pin ";
		cout << output_pin << endl;
		GPIO::output(output_pin, curr_value);
		curr_value ^= GPIO::HIGH;
	}

	GPIO::cleanup();

	return 0;
}
