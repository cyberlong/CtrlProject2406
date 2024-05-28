/*
This library while useful inside development, it isn't used in the final product
*/  

#define SerialReady 69420
#define Order1      69421
#define Order2      69422
#define Order3      69423
#define Order4      69424
#define Order5      69425
#define Order6      69426
#define Order7      69427
#define Order8      69428
#define Order9      69429

    // Serial vars
const byte numChars = 32; // Max Parsing chars
char receivedChars[numChars]; // Message buffer
char oldBuffer[numChars];
boolean newData = false; // Is theree new data

void BeginEzSerial (int Baud) { // Inits Serial Comms
  Serial.begin(Baud);           // and prints ready
  Serial.println(SerialReady);
}

void SerialRead() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc; // char buffer
 
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {       // while the end marker isn't sent
            if (rc != endMarker) {          // it accumulates the recieved chars
                receivedChars[ndx] = rc;    // in the Message buffer
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0';  // if the end marker is sent
                recvInProgress = false;     // ends the string
                ndx = 0;                    // and the new data flag
                newData = true;             // is sent to true
            }
        }

        else if (rc == startMarker) { // if the starter marker is sent
            recvInProgress = true;    // receiving process starts
        }
    }
}

char* returnData() {
    if (newData == true) {              // if there es new data
        Serial.print("Command sent: "); // a message is sent to serial
        Serial.println(receivedChars);  // and returns the new data
        newData = false;
        memcpy(receivedChars,oldBuffer,sizeof(receivedChars));
        return receivedChars;
    } else {                            // if there isn't new data
        return oldBuffer;               // sends only the old data
    }
    
}

int Serial2Int(char command[]) {                  // this shi
  byte len = sizeof(command)/sizeof(command[0]);  // ain't workin
  Serial.println(len);                            // use sscanf
  int total = 0;                                  // or whatever ig
  for (byte digit = 0 ; digit <= len ; digit++){
    total *= 10;
    total += command[digit] - '0';
  }
  return total;
}
