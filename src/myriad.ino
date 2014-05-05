#include <IRLib.h>
#include <SPI.h>
#include <boards.h>
#include <RBL_nRF8001.h>
#include <services.h> 

// The pin your IR receiver should be on
#define RECV_PIN        5

// The pin your IR LED should be on
#define EMIT_PIN        2

// Max buffer size we can receive from central BLE device
#define BLE_RECV_BUFFER_MAX   32

// Time in milliseconds that controls how we determine if
// multiple IR signals are considered to be one logical group
#define CODE_SKIP_TIME        1000 // in milliseconds

#define OP_CODE_ACK           "ACK"
#define OP_CODE_FIN           "FIN"

// IR receiving stuff
IRrecv MyriadReceiver(RECV_PIN);
IRsend MyriadEmitter;
IRdecode MyriadDecoder;

// Stores the incoming data received from the central BLE device
char ble_buffer[BLE_RECV_BUFFER_MAX] = {0};
unsigned char ble_len = 0;

// Controls whether we are in IR receiving mode
bool receive_enabled = false;
// Stores the last time we wrote an IR code over BLE
long last_ble_write = -1;

// Temporary buffer we'll use to convert the decoded IR signal
// struct into a string to send over BLE
char decode_buffer[32] = {0};

/**
 * Shortcut method to enable the IR receive mode that
 * will begin "listening" for IR signals.
**/
void enable_receiver()
{
    MyriadReceiver.enableIRIn(); // Start the receiver
    receive_enabled = true;
    Serial.println("Receive enabled");
}

/**
 * Shortcut method to disable IR receive mode
**/
void disable_receiver() {
   receive_enabled = false;
  
  // other stuff? 
}

/**
 * Attemps to read the available bytes from the BLE
 * central device and stores them in `ble_buffer`.
 *
 * @returns (int) number of bytes read
**/
int myriad_ble_read() {
  ble_len = 0;
  memset(ble_buffer, 0, sizeof(ble_buffer));
  
  if(ble_available()) {
    while (ble_available()) {
      if(ble_len < BLE_RECV_BUFFER_MAX - 1) {
        ble_buffer[ble_len++] = ble_read();
      }    
    }
  }
  return ble_len;
}

/**
 * Builds up a buffer to send via Bluetooth to the central device
 * in a CSV format: protocol,hexvalue,bits. These three values are
 * pulled directly from the decoder.
 *
 * Note that we're also recording the time the last write was sent out
 * which is used elsewhere to determine if the signals are part of
 * a bundle and should be treated as one logical IR code.
 *
 * @returns (int) number of bytes written
**/
int ble_write_signal() {
  // Temp buffer for converting values to strings
  char *s;
  // Temp buffer that stores the full string we're building
  unsigned char b[32]  = {0};
  // The current length of the temp buffer `b`
  int i = 0;
  // Temp variable for looping
  int j = 0;

  // Convert the decoded protocol type into a decimal string  
  s = itoa(MyriadDecoder.decode_type, decode_buffer, DEC);
  for(j = 0; j < strlen(s); ++j) {
    b[i++] = s[j];
  }
  
  // Add commas
  b[i++] = ',';

  // Convert the decoded integer value into a hex string
  s = ultoa(MyriadDecoder.value, decode_buffer, HEX);
  for(j = 0; j < strlen(s); ++j) {
    b[i++] = s[j];
  }
  
  // Add commas
  b[i++] = ',';

  // Convert the decoded bits into a decimal string
  s = itoa(MyriadDecoder.bits, decode_buffer, DEC);
  for(j = 0; j < strlen(s); ++j) {
    b[i++] = s[j];
  }
  // Add a null byte to the end of the buffer
  b[i] = '\0';
  
  // Send the data
  ble_write_bytes(b, i);
  
  // Store the time we wrote the last code
  last_ble_write = millis();
  
  // Return the number of bytes we wrote over BLE
  return i;
}

/**
 * Shortcut method for sending OP codes back to the central device
 *
 * @param (char *) op - the op code string to send back
**/
void ble_write_op_code(char *op) {
  ble_write_bytes((unsigned char *)op, strlen(op));
}

/**
 * Compares a string with what's currently in the `ble_buffer`
 * (what we've received from BLE central device)
 *
 * @param cmp - the string we're comparing against
 * @returns bool - true if `cmp` is equal to `ble_buffer`, false
 *                 if not
**/
bool cmp(char *cmp) {
  // cmp is a string to compare to the read bytes from the BLE device    
  return strcmp(ble_buffer, cmp) == 0;
}

/**
 * The Arduino setup method. See inline comments.
**/
void setup()
{ 
  // Enable serial output at 57600 baud rate
  Serial.begin(57600);
  delay(2000);while(!Serial); // delay for Leonardo
  
  Serial.println(F("Setting up bluetooth."));
  
  // Change the name of the BLE peripheral for advertising
  // and enable BLE advertising
  ble_set_name("Myriad");
  ble_begin();
  
  // Reset the last time we wrote something to the central
  last_ble_write = -1;
  
  Serial.println(F("Setup complete..."));
  
  // XXX - Probably should remove this after testing so
  // it's not constantly waiting for an incoming IR signal
  enable_receiver();
}

/**
 * The main Arduino loop.
**/
void loop() {
  
    // Read from BLE central and if we received something, we check to
    // see if it's an operational code, if it is we'll handle that, but
    // if it's not we will assume it's an IR code (if we're not in receive
    // mode) and forward the code on to the IR LED.
    int read_bytes = myriad_ble_read();

    if(read_bytes > 0) {
      
      // START op code - Enables the "learning" mode which will enable
      // IR mode and start listening for IR signals
      if(cmp("START")) {
        Serial.println(F("Listening mode ON."));
        ble_write_op_code(OP_CODE_ACK);
        //enable_receiver();
      }       
      
      // STOP op code - Disables "learning" mode and will ignore any future
      // IR signals.
      else if(cmp("STOP")) {
        Serial.println(F("Listening mode OFF."));
        //ble_write_op_code(OP_CODE_ACK);
        //disable_receiver();
      }
      
      // Non op code received and we'll assume it's an encoded IR signal we
      // need to emit with the LED
      else {
        
        Serial.print(F("IR code received: "));
        Serial.println(ble_buffer);

        int i;
        char c;
        int protocol = 0;
        long code = 0;
        int bits;

        // Decode the sent in string of the form (int)protocol,(long)hex,(int)bits into
        // the individual int and long values to use with the IRLib send routines.

        // The protocol is a single-digit (for now), so grab it and convert
        // to an integer.
        // TODO(abemusic) - need convert based on the comma instead
        protocol = String(ble_buffer[0]).toInt();

        // Converts hex string
        // TODO(abemusic) - should the first and last comma as the range to
        // loop over
        for(i = 2; i < strrchr(ble_buffer, ',') - ble_buffer; ++i) {
            c = tolower(ble_buffer[i]);
            if((c >= '0') && (c <= '9')) {
                c = c - '0';
            }
            else if ((c >= 'a') && (c <= 'f')) {
                c = c - 'a' + 10;
            }
            code = c + (code << 4);
        }

        // Starts at the character past the last comma and builds a String object
        // which we convert to an integer.
        String s = "";
        for(i = strrchr(ble_buffer, ',') - ble_buffer + 1; i < ble_len; ++i) {
            s.concat(ble_buffer[i]);
        }
        bits = s.toInt();

        /*Serial.println(protocol, DEC);
        Serial.println(code, HEX);
        Serial.println(bits, DEC);*/

        // Send the IR signal out
        MyriadEmitter.send(IRTYPES(protocol), code, bits);
      }
    } 
    
    // 
    ble_do_events();
    
    // Only enable and listen for IR codes when needed
    if(receive_enabled && MyriadReceiver.GetResults(&MyriadDecoder)) {
      
      // Restart the receiver so it can be capturing another code
      // while we are working on decoding this one.
      MyriadReceiver.resume(); 
      
      // Decode the IR signal into the internal decoder struct
      MyriadDecoder.decode();
      //MyriadDecoder.DumpResults();
              
      // If this is a supported decoded signal, send it out
      if(MyriadDecoder.decode_type > 0) {
        Serial.print(F("Received decoded signal: "));
        Serial.print(MyriadDecoder.decode_type, DEC);
        Serial.print(",");
        Serial.print(MyriadDecoder.value, HEX);
        Serial.print(",");
        Serial.println(MyriadDecoder.bits, DEC);
        ble_write_signal();
      } 
    }
    
    // Check to see if we should send a special message to let the central
    // know if we've received all IR signals available
    if(last_ble_write > 0) {
      unsigned long now = millis();
      
      if(now - last_ble_write > CODE_SKIP_TIME) {
        Serial.print(CODE_SKIP_TIME, DEC);
        Serial.println(F("ms have passed since the last code was received.")); 
        ble_write_op_code(OP_CODE_FIN);        
        last_ble_write = 0;
      }
    }    
}
