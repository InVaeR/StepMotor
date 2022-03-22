#if !DEVICE_CAN
#error [NOT_SUPPORTED] CAN not supported for this target
#endif

//#include "stdio.h"
//#include "string"
#include "mbed.h"
#include "platform/mbed_thread.h"
#include "TextLCD.h"

// Create a BufferedSerial object to be used by the system I/O retarget code
//                                TX     RX     BaudRate
static BufferedSerial serial_port(USBTX, USBRX, 115200);

// Create a CAN object
//           RXD,   TXD     pins params
CAN can_port(PA_11, PA_12);

// Create built-in indicator in the board
DigitalOut indicator(LED1);

// Create
DigitalOut heater(PC_3);

// Create a LCD Display object
//         (  rs,    e,   d4,    d5,   d6,   d7, Type)
TextLCD lcd(PB_2, PB_1, PB_4, PB_10, PA_8, PA_9, TextLCD::LCD16x2);


float z = 0.0; // Load sensor value (Lz)
float r = 0.0; // Load sensor value (Lr)
float l = 0.0; // Load sensor value (Ll)
float t = 0.0; // Temperature sensor value (T)
float s = 0.0; // Speed sensor value (V)


const int z_ID = 1284; // ID Load sensor z          (0x504) or 1348 (544) [4]
const int r_ID = 1285; // ID Load sensor r          (0x505) or 1349 (545) [5]
const int l_ID = 1283; // ID Load sensor l          (0x503) or 1347 (543) [3]
const int t_ID = 1034; // ID Temperature sensor T   (0x40A) or            [2]

Ticker ticker_data_sender;

Ticker ticker_pid;

AnalogIn setpointReader(A0);

float setpoint = 0;         // PID regulator T setpoint
float Kp = 0.0;             // k P
float Ki = 0.0;             // k I
float Kd = 0.0;             // k D
const float dt = 1;

float Iprev = 0.0;          //
float Eprev = 0.0;          //

float constrain(float x, float minOut, float maxOut);

// PID control
void compute_PID()
{
    float input = t;
    
    float E = setpoint - input;
    float I = constrain(Iprev + E * dt * Ki, 0, 1);
    float D = (E - Eprev) / dt;
    Eprev = E;
    Iprev = I;
    //heater = constrain(E * Kp + I + D * Kd, 0, 1);
}

float constrain(float x, float minOut, float maxOut)
{
    if (x < minOut)
        return minOut;
    if (x > maxOut)
        return maxOut;
    return x;
}

// Check CAN port and read value
void check_CAN();

// Sending values from sensors to serial port
void send_serial();

int main()
{
    // Setup serial settings
    // serial_port.set_format(8, BufferedSerial::None, 1);

    // Ticker call send_serial 20 Hz
    ticker_data_sender.attach(&send_serial, 50ms);
    //ticker_10Hz.attach(&compute_PID, 1s);

    // Silent mode ?!
    can_port.monitor(true);
    // can_port.attach(&check_CAN, CAN::RxIrq);

    // Start an endless loop
    while (1) {
        check_CAN();
    }
}

// Sending values from sensors to serial port
/*void send_serial()
{
    serial_port.write("z", 1);
    serial_port.write(&z, 4);
    serial_port.write("\n", 1);

    serial_port.write("r", 1);
    serial_port.write(&r, 4);
    serial_port.write("\n", 1);

    serial_port.write("l", 1);
    serial_port.write(&l, 4);
    serial_port.write("\n", 1);

    serial_port.write("t", 1);
    serial_port.write(&t, 4);
    serial_port.write("\n", 1);

    serial_port.write("s", 1);
    serial_port.write(&s, 4);
    serial_port.write("\n", 1);
}*/

void send_serial()
{
    const char ff = 255;
    serial_port.write(&ff, 1);
    serial_port.write("z", 1);
    serial_port.write("d", 1);
    serial_port.write(&z,  4);
    serial_port.write(&ff, 1);
    serial_port.write(&ff, 1);
    serial_port.write("\n", 1);

    serial_port.write(&ff, 1);
    serial_port.write("r", 1);
    serial_port.write("d", 1);
    serial_port.write(&r,  4);
    serial_port.write(&ff, 1);
    serial_port.write(&ff, 1);
    serial_port.write("\n", 1);

    serial_port.write(&ff, 1);
    serial_port.write("l", 1);
    serial_port.write("d", 1);
    serial_port.write(&l,  4);
    serial_port.write(&ff, 1);
    serial_port.write(&ff, 1);
    serial_port.write("\n", 1);

    serial_port.write(&ff, 1);
    serial_port.write("t", 1);
    serial_port.write("d", 1);
    serial_port.write(&t,  4);
    serial_port.write(&ff, 1);
    serial_port.write(&ff, 1);
    serial_port.write("\n", 1);

    serial_port.write(&ff, 1);
    serial_port.write("s", 1);
    serial_port.write("d", 1);
    serial_port.write(&s,  4);
    serial_port.write(&ff, 1);
    serial_port.write(&ff, 1);
    serial_port.write("\n", 1);

    /*serial_port.write(&ff, 1);
    serial_port.write("o", 1);
    serial_port.write("d", 1);
    serial_port.write(&state, 4);
    serial_port.write(&ff, 1);
    serial_port.write(&ff, 1);
    serial_port.write("\n", 1);*/
}

// Get value from CAN message
float get_value(CANMessage _msg)
{
    if (_msg.len >= 4)
        return *(float*) _msg.data;
    else
        return 0;
}


void check_CAN()
{
    CANMessage msg;
    if (can_port.read(msg)) {
        float val = get_value(msg);
        if (val != 0) {
            switch(msg.id) {
                case z_ID:
                    z = val;
                    break;
                case r_ID:
                    r = val;
                    break;
                case l_ID:
                    l = val;
                    break;
                case t_ID:
                    t = val;
                    break;
            }
        }
    }
}
