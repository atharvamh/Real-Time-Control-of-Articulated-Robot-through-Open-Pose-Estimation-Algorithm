
// Receive a number as text and convert it to an int
#include <Servo.h>
const byte numChars = 32;
char receivedChars[numChars];                       // an array to store the received data

boolean newData = false;                           // flag variable for new_data_packet arrival
int ar = 0;
int a = 0;
int b = 0;
int dataNumber = 0;             // new for this version
int current_angle_shoulder = 0;
int current_angle_elbow = 0;

// declare servo objects for 3 servo motors used

Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");           // Setting up Arduino for serial communication
    servo1.attach(9);
    servo2.attach(11);
    servo3.attach(5);
    pinMode(13,OUTPUT);
}

void loop() {
    recvWithEndMarker();
    showNewNumber();
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void showNewNumber() {
    if (newData == true) {
        dataNumber = 0;             // new for this version
        dataNumber = atoi(receivedChars);   // new for this version
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        Serial.print("Data as Number ... ");    // new for this version
        Serial.println(dataNumber);     // new for this version

        String RD = String(receivedChars);
        String Ar = RD.substring(0,1);
        String A = RD.substring(1,4);
        String B = RD.substring(4,7);

        int ar = Ar.toInt();
        int a = A.toInt();
        int b = B.toInt();

        Serial.println(ar);
        Serial.println(a);
        Serial.println(b);
        int k=0;
        if (ar)
          k = 180;

        servo3.write(k);
        
        while(current_angle_shoulder != a)
        {
            current_angle_shoulder = (current_angle_shoulder < a) ? (current_angle_shoulder+1) : (current_angle_shoulder-1);
            servo1.write(a);
            delay(5);
        }

        while(current_angle_elbow !=b)
        {
            current_angle_elbow = (current_angle_elbow < b) ? (current_angle_elbow+1) : (current_angle_elbow-1);
            servo2.write(b);
            delay(5);
        }
        
        newData = false;
    }
}
