// Copyright (C) 2017 Vorpal Robotics, LLC.

//////////// For more information:
// Main website: http://www.vorpalrobotics.com
// Store (for parts and kits): http://store.vorpalrobotics.com
// Wiki entry for this project: http://vorpalrobotics.com/wiki/index.php?title=Vorpal_The_Hexapod
// Wiki information on our other projecs: http://www.vorpalrobotics.com/wiki

///////////  License:
// This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
// Attribution for derivations of this work should be made to: Vorpal Robotics, LLC
//
// You may use this work for noncommercial purposes without cost.
// For information on licensing this work for commercial purposes, please send email to support@vorpalrobotics.com


//////////// Required IDE Version and libraries:
// This program does not require any libraries that aren't already built-in to the Arduino IDE. It uses SPI, wire,
// SD, and SoftwareSerial but they are all standard in the IDE so you won't need to load any new libraries.
// This has been tested mainly using Arduino IDE version 1.8.5 and it may not work properly in earlier versions.

//////////// Processor notes:
// The processor we use on the gamepad is the Arduino Nano ATMEGA328p running at 16 mHz. It would probably work
// on other processors that have at least as much RAM and program memory as the Nano. It's a good idea to use
// a Nano with an FT232 usb serial IO chip rather than the cheaper CH34x chip, because the FT232 will work much more seamlessly
// with Mac computers for use with the Scratch Features. If you don't care about using Scratch on Mac then it doesn't matter.

// This version fixes some issues with Scratch Record/Play and adds "trim mode" support for fine adjustment of servos.

const char *Version = "#V1r8j";

int debugmode = 0;          // Set to 1 to get more debug messages. Warning: this may make Scratch unstable so don't leave it on.

#define DEBUG_SD  1          // Set this to 1 if you want debugging info about the SD card and record/play functions.
                            // This is very verbose and may affect scratch performance so only do it if you have to.
                            // It also takes up a lot of memory and you might get warnings from Arduino about too little
                            // RAM being left. But it seems to work for me even with that warning.

#define DEBUG_BUTTONS 0     // There is some manufacturer who is selling defective DPAD modules that look just like the
                            // ones sold by Vorpal Robotics, and they have a different set of output values for each
                            // button (and only use a small percentage of the range of values, which is why I consider
                            // them to be defective). If you get one of those, set this define to 1 and you'll get debug info sent
                            // to the serial monitor to help you figure out what values will work. After that you can
                            // modify the values in the decode_button() function below to suite your DPAD button module.

#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <SoftwareSerial.h>

//////////////////////////////////////////
// Gamepad layout, using variable names used in this code:
//
//                       W
// M0  M1  M2  M3
// M4  M5  M6  M7        F
// M8  M9  M10 M11     L   R
// M12 M13 M14 M15       B
//
//  The buttons on the left side are a 4x4 matrix, connected
// to digital io ports 6-9 (columns) and 2-5 (rows)
// the right side buttons in Dpad configuration are a set of
// resistive buttons and are on A1. Only one of those can be
// detected at a time.
//
// The buttons on the right are used to control motion. The buttons on
// the left signal the "mode" you are in. After pressing one button on the
// left, you stay in that mode until a different mode button is pressed. You
// don't have to keep holding them.
//
// A "long tap" on a mode button uses the scratch recording saved to that button,
// if there is one. If there isn't one, the long tap just does the default action
// (same as short tap).
//
// The rows are marked on the gamepad: W, D, F, R for "walk", "dance", "fight", "record"
// respectively.  See wiki documentation for the W, D, F modes.
//
// Fourth row (M13-M16) are for recording motions. These Buttons leave you
// in the same movement mode, they are independent of movement mode.
// M13: Record/Stop
// M14: Play/Pause
// M15: Rewind to start
// M16: Erase. This has to be held for 3 seconds, after which an extremely annoying
//             high pitched beep indicates the erase has occured.
//
// While recording the internal beeper makes a high pitched short chirp
// every second to remind you. Because this is recording to an SD card,
// and it only takes 130 bytes for 1 second of recording, you have
// tons of recording time. We internally limit it to about 1 hour
// just so people don't forget they have record on and fill up their card.
//
// While playing back the internal beeper will make a high pitched beep
// every second.  When that stops happening the recording is finished.
//
// If you record and stop, then record again, you will be adding on to the
// prior recording unless you rewind first.


// Each movement mode (rows W, D, F) transmits the mode characters, for example W1 or D3, followed by the
// DPAD characters: l, r, f, b, w, s.  These stand for left, right, forward, backward, weapon, and stop
// and they correspond to dpad directions (weapon is the extra dpad button, stop is the lack of any button being pressed).


SoftwareSerial BlueTooth(A5,A4);  // connect bluetooth module Tx=A5=Yellow wire Rx=A4=Green Wire
                                  // although not intuitive, the analog ports function just fine as digital ports for this purpose
                                  // and in order to keep the wiring simple for the 4x4 button matrix as well as the SPI lines needed
                                  // for the SD card, it was necessary to use analog ports for the bluetooth module.



// Matrix button definitions
// Top row
#define WALK_1 0
#define WALK_2 1
#define WALK_3 2
#define WALK_4 3
// Second row
#define DANCE_1 4
#define DANCE_2 5
#define DANCE_3 6
#define DANCE_4 7
// Third Row
#define FIGHT_1 8
#define FIGHT_2 9
#define FIGHT_3 10
#define FIGHT_4 11
// Bottom Row
#define REC_RECORD 12
#define REC_PLAY 13
#define REC_REWIND 14
#define REC_ERASE 15

// Pin definitions

#define DpadPin A1
#define VCCA1 A2    // we play a trick to power the dpad buttons, use adjacent unusued analog ports for power
#define GNDA1 A3    // Yes, you can make an analog port into a digital output!
#define SDCHIPSELECT 10   // chip select pin for the SD card reader

// definitions to decode the 4x4 button matrix

#define MATRIX_ROW_START 6
#define MATRIX_COL_START 2
#define MATRIX_NROW 4
#define MATRIX_NCOL 4

long suppressButtonsUntil = 0;      // default is not to suppress until we see serial data
int verbose = 0;
File SDGamepadRecordFile;           // REC.txt, holds the gamepad record/play file
const char SDGamepadRecordFileName[] = "REC.txt";   // never changes
char SDScratchRecordFileName[4];    // three letters like W1f for walk mode 1 forward dpad, plus one more for end of string '\0'
File SDScratchRecordFile;

char ModeChars[] = {'W', 'D', 'F', 'R'};
char SubmodeChars[] = {'1', '2', '3', '4'};

char CurCmd = ModeChars[0];  // default is Walk mode
char CurSubCmd = SubmodeChars[0]; // default is primary submode
char CurDpad = 's'; // default is stop
unsigned int BeepFreq = 0;   // frequency of next beep command, 0 means no beep, should be range 50 to 2000 otherwise
unsigned int BeepDur = 0;    // duration of next beep command, 0 means no beep, should be in range 1 to 30000 otherwise
                             // although if you go too short, like less than 30, you'll hardly hear anything

void println() {
  Serial.println("");
}

void debug(char *s) {
  if (verbose||suppressButtonsUntil >= millis()) {
    Serial.print(s);
  }
}

void debug(int s) {
  if (verbose||suppressButtonsUntil >= millis()) {
    Serial.print(s);
  }
}

void debug(long s) {
  if (verbose||suppressButtonsUntil >= millis()) {
    Serial.print(s);
  }
}

void debugln() {
  if (verbose||suppressButtonsUntil >= millis()) {
    println();
  }
}

// 4x4 Button Matrix function. For now we will just return the first button found, although it is possible to detect
// multiple simultaneous button presses. We might use that in the future to increase the
// effective number of possible controls.

int priorMatrix = -1;
long curMatrixStartTime = 0;
int longClick = 0;  // this will be set to 1 if the last matrix button pressed was held a long time
int priorLongClick = 0; // used to track whether we should beep to indicate new longclick detected.

#define LONGCLICKMILLIS 500

// find out which button is pressed, the first one found is returned
int scanmatrix() {
  // we will energize row lines then read column lines

  // first set all rows to high impedance mode
  for (int row = 0; row < MATRIX_NROW; row++) {
    pinMode(MATRIX_ROW_START+row, INPUT);
  }
  // set all columns to pullup inputs
  for (int col = 0; col < MATRIX_NCOL; col++) {
    pinMode(MATRIX_COL_START+col, INPUT_PULLUP);
  } 

  // read each row/column combo until we find one that is active
  for (int row = 0; row < MATRIX_NROW; row++) {
    // set only the row we're looking at output low
    pinMode(MATRIX_ROW_START+row, OUTPUT);
    digitalWrite(MATRIX_ROW_START+row, LOW);
    
    for (int col = 0; col < MATRIX_NCOL; col++) {
      delayMicroseconds(100);
      if (digitalRead(MATRIX_COL_START+col) == LOW) {
        // we found the first pushed button
        if (row < 3) {
          CurCmd = ModeChars[row];
          CurSubCmd = SubmodeChars[col];
        }
        int curmatrix = row*MATRIX_NROW+col;
        if (curmatrix != priorMatrix) {
          curMatrixStartTime = millis();
          priorMatrix = curmatrix;
          longClick = priorLongClick = 0;
        } else if (millis() - curMatrixStartTime > LONGCLICKMILLIS) {
          // User has been holding down the same button continuously for a long time
          longClick = 1;
        }
        return curmatrix;
      }
    }
    pinMode(MATRIX_ROW_START+row, INPUT);  // set back to high impedance
    //delay(1);
  }

  // if we get here no buttons were pushed, return -1 and
  // clear out the timers used for long click detection.
  priorMatrix = -1;
  curMatrixStartTime = 0;
  return -1;
}

// This decodes which button is pressed on the DPAD
// There are some DPAD modules that output different values
// and if you get one of those (not sold by Vorpal) you may need
// to test out your dpad to find reasonable values and change
// the values below.

char decode_button(int b) {

#if DEPADDEBUG
  Serial.print("DPAD: "); Serial.println(b);
#endif

// If your DPAD is doing the wrong things for each button, and you didn't
// purchase it from Vorpal Robotics, LLC, then you may have a defective
// DPAD module that outputs nonstandard values. Turn on DEPADDEBUG mode,
// then use the serial monitor to find out what values your buttons are
// returning. Modify the code below to return the correct values for each
// button.

  if (b < 100) {
     return 'b';  // backward (bottom button)
  } else if (b < 200) {
     return 'l';  // left 
  } else if (b < 400) {
    return 'r';   // right
  } else if (b < 600) {
    return 'f';  // forward (top of diamond)
  } else if (b < 850) {
    return 'w';  // weapon (very top button) In the documentation this button is called "Special"
                 // but a long time ago we called it "weapon" because it was used in some other
                 // robot projects that were fighting robots. The code still uses "w" since "s" means stop.
  } else {
    return 's';  // stop (no button is pressed)
  }
}

//////////////////////////////////////////////////////////
// BEEP FREQUENCIES
//////////////////////////////////////////////////////////
#define BF_ERROR 100
#define BF_RECORD_CHIRP 1500
#define BF_PLAY_CHIRP 1000
#define BF_PAUSE_CHIRP 2000
#define BF_NOTIFY  500
#define BF_ERASE   2000
#define BF_REWIND  700

//////////////////////////////////////////////////////////
// BEEP DURATIONS
//////////////////////////////////////////////////////////
#define BD_CHIRP 10
#define BD_SHORT 100
#define BD_MED   200
#define BD_LONG  500
#define BD_VERYLONG 2000

//////////////////////////////////////////////////////////
//   RECORD/PLAY FEATURES
/////////////////////////////////////////////////////////

// States for the Scratch Record/play function
// These can't be the same state variable as used for the gamepad REC/PLAY states because you can have
// a scratch recording playing from a long tap button at the same time you've got
// the gamepad REC button working.
#define SREC_STOPPED    0
#define SREC_RECORDING  1
#define SREC_PLAYING    2

// States for the Gamepad record/play function
#define REC_STOPPED     0
#define REC_RECORDING   1
#define REC_PLAYING     2
#define REC_PAUSED      3
#define REC_REWINDING   4
#define REC_ERASING     5

#define REC_MAXLEN (40000)      // we will arbitrarily limit recording time to about 1 hour (forty thousand deci-seconds)
#define REC_FRAMEMILLIS 100     // time between data frames when recording/playing
#define REC_FRAMESIZE 13        // number of bytes in a frame

int GRecState = REC_STOPPED;  // gamepad recording state
int SRecState = SREC_STOPPED; // scratch recording state
long GRecNextEventTime = 0; // next time to record or play a gamepad record/play event 
long SRecNextEventTime = 0; // next time to play a scratch recording event

long NextTransmitTime = 0;  // next time to send a command to the robot
char PlayLoopMode = 0;

void setBeep(int f, int d) {
  // schedule a beep to go out with the next transmission
  // this is not quite right as there can only be one beep per transmission
  // right now so if two different subsystems wanted to beep at the same time
  // whichever one is scheduled last would win. 
  // But because 10 transmits go out per second this seems sufficient, and it's simple
  BeepFreq = f;
  BeepDur = d;
}

// Open a scratch recording file.
// Closes any prior recording file first. This may be the same file, if so then this
// effectively just rewinds it.
// Returns 1 if the open worked, otherwise 0
//
int openScratchRecordFile(char *cmd, char *subcmd, char *dpad) {
  if (SDScratchRecordFile) {  // if one is already open, close it
    SDScratchRecordFile.close();
  }

  SDScratchRecordFileName[0] = cmd;
  SDScratchRecordFileName[1] = subcmd;
  SDScratchRecordFileName[2] = dpad;
  SDScratchRecordFileName[3] = '\0';
 
  SDScratchRecordFile = SD.open(SDScratchRecordFileName, FILE_WRITE);
  if (SDScratchRecordFile) {
    SDScratchRecordFile.seek(0);
    return 1;
  } else {
    setBeep(100, 100); // error tone
    Serial.print("#SDOF:"); Serial.println(SDScratchRecordFileName); // SDOF means "SD Open Failed"
    return 0;
  }
}


long LastRecChirp;  // keeps track of the last time we sent a "recording is happening" reminder chirp to the user

void removeScratchRecordFile(char *cmd, char *subcmd, char *dpad) {
  SDScratchRecordFile.close();  // it's safe to do this even if the file is not open

  SDScratchRecordFileName[0] = cmd;
  SDScratchRecordFileName[1] = subcmd;
  SDScratchRecordFileName[2] = dpad;
  SDScratchRecordFileName[3] = '\0';
  SD.remove(SDScratchRecordFileName);
}

void removeAllRecordFiles() {
  SDGamepadRecordFile.close();
  SDScratchRecordFile.close();
  SD.remove(SDGamepadRecordFileName);

  // Go through every combination of Scratch filenames and blow them all away
  // Truthfully this should probably be written by getting the directory structure
  // and finding all the files that actually exist then remove just the ones with
  // the right name pattern. But this seems fast enough (about 500 milliseconds for all of the
  // potential filenames) and takes a lot less codespace, which is at a premium on a nano

#if DEBUG_SD
  //Serial.print("#DS");Serial.println(millis());
#endif
  char fname[4];
  char mode[] = "WDF";
  char num[] = "1234";
  char dpad[] = "FBLRSW";
  fname[3] = '\0';
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 4; j++) {
      for (int k = 0; k < 6; k++) {
        fname[0] = mode[i];
        fname[1] = num[j];
        fname[2] = dpad[k];
        SD.remove(fname);
      }    
    }
  }
#if DEBUG_SD
  //Serial.print("#DE");Serial.println(millis());
#endif
}

int sendbeep() {
    unsigned int beepfreqhigh = highByte(BeepFreq);
    unsigned int beepfreqlow = lowByte(BeepFreq);
    BlueTooth.print("B");
    BlueTooth.write(beepfreqhigh);
    BlueTooth.write(beepfreqlow);

    unsigned int beepdurhigh = highByte(BeepDur);
    unsigned int beepdurlow = lowByte(BeepDur);
    BlueTooth.write(beepdurhigh);
    BlueTooth.write(beepdurlow);

    // return checksum info
    return 'B'+beepfreqhigh+beepfreqlow+beepdurhigh+beepdurlow;
}

void SendNextRecordedFrame(File file, char *filename, int loop) {
        int length = file.read(); // first byte is the number of bytes in the frame to send
        
#if DEBUG_SD
        Serial.print("#L"); Serial.println(length);
#endif
        if (length <= 0 || file.available() < length+1) { // we are at the end of the file
          if (loop) {
            // rewind to start of file and just keep going in this state to loop
            file.seek(0);
            length = file.read();
            if (file.available() < length+1) { // something's wrong, the file is damaged, return
              return;
            }
            Serial.println("#LOOP");
          } else {
            if (GRecState == REC_PLAYING) {
              GRecState = REC_STOPPED;
            }
            if (SRecState == SREC_PLAYING) {
              SRecState = SREC_STOPPED;
            }
            file.close();
            return;
          }
        }
        // if we get here, there is a full frame to send to the robot

        // one complication: if we're playing a scratch recording off a long click button, and also
        // we're in gamepad record mode, then we need to save the current scratch playing records to the gamepad record file
        if (SRecState == SREC_PLAYING && GRecState == REC_RECORDING) {
          // in this case we know the file we're reading from must be a scratch recording that's now playing
          // so write it's length into the gamepad recording file.
          SDGamepadRecordFile.write(length);
        }
        BlueTooth.print("V1");
        BlueTooth.write(length+5);  // include 5 more bytes for beep
#if DEBUG_SD
        Serial.print("#V1L="); Serial.println(length+5);
#endif
        {
            int checksum = length+5;  // the length byte is included in the checksum
            for (int i = 0; i < length && file.available()>0; i++) {
              int c = file.read();
              checksum += c;
              BlueTooth.write(c);
              if (SRecState == SREC_PLAYING && GRecState == REC_RECORDING) {
                // in this case we know the file we're reading from must be a scratch recording that's now playing
                // and we're also recording from the gamepad record feature, so we need to write this byte into the gamepad
                // recording file
                SDGamepadRecordFile.write(c);
#if DEBUG_SD
                Serial.print("#SC->GP@");  Serial.println(SDGamepadRecordFile.position()); // scratch data written to gamepad recording file
#endif
              }
#if DEBUG_SD
              Serial.print("#"); Serial.write(c); Serial.print("("); Serial.print(c); Serial.println(")");
#endif
            }
            checksum += sendbeep();
            setBeep(0,0); // clear the beep since we've already sent it out
            checksum = (checksum % 256);
            BlueTooth.write(checksum);
#if DEBUG_SD
            Serial.print("#CS"); Serial.println(checksum);
#endif
        }

#if DEBUG_SD
        Serial.print("#P:"); Serial.print(filename); Serial.println(file.position());
#endif
}

void RecordPlayHandler() {
  // We save records consisting of a length byte followed by the data bytes.
  // This allows easy transmission by just putting the "V1" header out followed by the
  // length and data, followed by the checksum to bluetooth.

  // first let's see if scratch is playing a command right now from a SD card button file
  if (SRecState == SREC_PLAYING) {
     if (!SDScratchRecordFile) {
      // something is wrong, the file is not open
      Serial.print("#E:"); Serial.println(SDScratchRecordFileName); // "Scratch Play Error"
      SRecState = SREC_STOPPED;
      return;
     }
     SendNextRecordedFrame(SDScratchRecordFile, SDScratchRecordFileName, 1); // always loop scratch recordings

     return;  // in this case we don't need to continue with the rest of the code because we've taken care of both
              // scratch playing and gamepad recording from a scratch playback
  }

  // If we get here, then Scratch was not recording or playing anything
  // The code below handles the gamepad manual recording functions
  
  switch (GRecState) {
    case REC_STOPPED:
      // make sure file is closed and flushed
      SDGamepadRecordFile.close();
      break;
      
    case REC_PAUSED:
      // nothing to do!
#if DEBUG_SD
      Serial.println("#PS");
#endif
      break;

    ////////////////////////////////////////////////////////////
    case REC_PLAYING: {
        if (!SDGamepadRecordFile) {
          // We don't seem to have an opened file, something is wrong
          Serial.println("#SDNOP"); // "SD Not Open"
          GRecState = REC_STOPPED;
          return;
        }
        // chirp once every second to remind user they are playing a recording
        long t = millis();
        if ((t-LastRecChirp) >= 1000) {
           setBeep(BF_RECORD_CHIRP, BD_CHIRP);
           LastRecChirp = t;
        }
        
        if (millis() < GRecNextEventTime) {
          // it's not time to send the next record yet
          return;
        }

        // If we get here, it's time to transmit the next record over bluetooth
        GRecNextEventTime = millis() + REC_FRAMEMILLIS;
#if DEBUG_SD
        Serial.print("#P:"); Serial.print(SDGamepadRecordFileName);Serial.print("@");Serial.print((long)SDGamepadRecordFile.position()); println();
#endif
        SendNextRecordedFrame(SDGamepadRecordFile, SDGamepadRecordFileName, PlayLoopMode);
    } 
      break;

    /////////////////////////////////////////////////////////
    case REC_RECORDING:
      if (!SDGamepadRecordFile) {
          // something is hideously wrong
          GRecState = REC_STOPPED;
          return;
      }
      // chirp once every second to remind user they are recording
      if ((millis()-LastRecChirp) >= 1000) {
         setBeep(BF_RECORD_CHIRP, BD_CHIRP);
         LastRecChirp = millis();
      }

      if (millis() < GRecNextEventTime) {
        // it's not time to record a frame yet
        return;
      }
#if DEBUG_SD
      Serial.print("REC@"); Serial.println(SDGamepadRecordFile.position());
#endif
      GRecNextEventTime = millis() + REC_FRAMEMILLIS;
      { // local variables require a scope
        int three = 3;  // yeah this is weird but trust me
        SDGamepadRecordFile.write(three);
        SDGamepadRecordFile.write(CurCmd);
        SDGamepadRecordFile.write(CurSubCmd);
        SDGamepadRecordFile.write(CurDpad);
      }
      // we don't record headers or checksums. The SD card is considered a reliable device.
#if  DEBUG_SD
      Serial.print("#L="); Serial.println(SDGamepadRecordFile.size());
#endif
      
      break;

    ////////////////////////////////////////////////////////////
    case REC_REWINDING:
      GRecNextEventTime = 0;
      GRecState = REC_STOPPED;
      SDGamepadRecordFile.flush();  // it's safe to do all these operations without checking if the file is open
      SDGamepadRecordFile.seek(0);       

#if DEBUG_SD
      Serial.println("#RW");
#endif
      break;

    //////////////////////////////////////////////////////////
    default:
      Serial.println("#E2:");      // "Record State Error"
      Serial.print(GRecState); println();
      break;
  }
}

void setup() {
  // make a characteristic flashing pattern to indicate the gamepad code is loaded.
  pinMode(13, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(13, !digitalRead(13));
    delay(400);
  }
  // after this point you can't flash the led on pin 13 because we're using it for SD card

  BlueTooth.begin(38400);
  Serial.begin(9600);
  Serial.println(Version);

  pinMode(A0, OUTPUT);  // extra ground for additional FTDI port if needed
  digitalWrite(A0, LOW);
  pinMode(VCCA1, OUTPUT);
  pinMode(GNDA1, OUTPUT);
  digitalWrite(GNDA1, LOW);
  digitalWrite(VCCA1, HIGH);
  pinMode(SDCHIPSELECT, OUTPUT);
  digitalWrite(SDCHIPSELECT, HIGH); // chip select for SD card
  
  if (!SD.begin(SDCHIPSELECT)) {
    Serial.println("#SDBF");    // SD Begin Failed
    return;
  }
}

int priormatrix = -1;
long curmatrixstarttime = 0;  // used to detect long tap for play button and erase button

  // Scratch integration: If anything appears on the serial input,
  // it's probably coming from a scratch program, so simply send it
  // out to the robot, unless it appears to be a command to start
  // recording onto a gamepad button

// The following are states for the scratch state machine

#define SCR_WAITING_FOR_HEADER  0
#define SCR_WAITING_FOR_HEX_1   1
#define SCR_WAITING_FOR_REC_1   2
#define SCR_WAITING_FOR_LENGTH  3
#define SCR_HEX_TRANSFER        4
#define SCR_REC_COMMAND         5

#define SCR_MAX_COMMAND_LENGTH  80    // we never expect scratch to send more than 16 commands at a time and max command len is 5 bytes.
                                      // 16 commands is enough to send a move individually to every servo, plus a beep, plus a sensor
                                      // request, plus a couple more besides that. At 80 bytes the packet would require about 20 millisec
                                      // to send over bluetooth, which allows plenty of time for the hexapod to send back the sensor
                                      // data before the next transmission. Although the bluetooth hardware is full duplex, the
                                      // softwareserial library currently is not, so we can't be both reading and writing the bluetooth
                                      // module at the same time.

// this little function prints out info to help debug state machine errors
void scratcherror(char *state, char c) {
    Serial.print("#SER:"); Serial.print(state); Serial.print(":"); 
    Serial.print(c); Serial.print("("); Serial.write(c); Serial.println(")");
}

int ScratchState = SCR_WAITING_FOR_HEADER;
int ScratchXmitBytes = 0; // how many packet bytes did we get so far
int ScratchLength = 0;    // length received in packet
char ScratchSDFileName[3]; // the name of a scratch recording file which is a matrix/dpad combo like: W1b for "walk mode 1 backward dpad"

int handleSerialInput() {

  int dataread = 0;
  
  while (Serial.available()>0) {
    int c = Serial.read();
    //Serial.print("C="); Serial.write(c); Serial.print("/"); Serial.print(c); println();
    dataread++;
    
    switch (ScratchState) {
      case SCR_WAITING_FOR_HEADER:
        if (c == 'V') {  // a command to be sent directly to the robot
          ScratchState = SCR_WAITING_FOR_HEX_1;
        } else if (c == 'R') { // a scratch recording command
          ScratchState = SCR_WAITING_FOR_REC_1;
#if DEBUG_SD
          Serial.println("#R");
#endif
        } else {
          scratcherror("H", c);
        }
        break;
      case SCR_WAITING_FOR_HEX_1:  // both the R and V paths expect the next character to be a 1 meaning "version 1 protocol"
      case SCR_WAITING_FOR_REC_1:
        if (c != '1') {
          scratcherror("1", c);
          if (c == 'V') {
            ScratchState = SCR_WAITING_FOR_HEX_1;  // this little hack gets us back to the right state faster in some cases
          } else if (c == 'R') {
            ScratchState = SCR_WAITING_FOR_REC_1;  // ditto.
          } else {
            ScratchState = SCR_WAITING_FOR_HEADER;
          }
        } else if (ScratchState == SCR_WAITING_FOR_HEX_1) {
          ScratchState = SCR_WAITING_FOR_LENGTH;
        } else {  // state should be SCR_WAITING_FOR_REC_1 here
          ScratchState = SCR_REC_COMMAND;
          ScratchXmitBytes = 0;   // we expect 3 more bytes to tell us what button to record to
                                  // this will either be a command spec like: W1f or F2s, or it
                                  // will be special: SSS means stop recording, or DDD means delete all recordings
#if DEBUG_SD
          Serial.println("#TX");  // "Scratch Record Waiting for Transmit bytes"
#endif
        }
        break;
      case SCR_WAITING_FOR_LENGTH:
          // This is a hexapod command not a REC command
          ScratchXmitBytes = 0;  // how many have we gotten
          ScratchLength = c;    // how many do we expect
          if (ScratchLength > SCR_MAX_COMMAND_LENGTH) {
            // command is longer than ever expected from scratch so something is wrong
            ScratchState = SCR_WAITING_FOR_HEADER;  // punt back to waiting for a new record
            Serial.print("#E3"); Serial.println(c);  // exceeded max len error message
          } else {
            BlueTooth.print("V1");  // version 1 header
            BlueTooth.write(c);     // length of packet
            if (SRecState = SREC_RECORDING) {
              // we're also recording to a gamepad button so save the length byte
              SDScratchRecordFile.write(c);  // if this file is not open this won't crash anything, I checked the SD card library code
            }
            ScratchState = SCR_HEX_TRANSFER;
          }
          break;
      case SCR_HEX_TRANSFER:
          ScratchXmitBytes++;
          BlueTooth.write(c);  // the next byte in the transmission
          if (SRecState == SREC_RECORDING && ScratchXmitBytes <= ScratchLength) {
            // In this case we are also in scratch record mode so save the data to the recording file.
            // We use <= ScratchLength in the above condition because we don't want to put the 
            // checksum byte on there because that's not normally stored in
            // recording files (similar to the way the V1 header is not stored). The SD card is a reliable device
            // so we don't waste space and writing time by storing the checksum or headers
            SDScratchRecordFile.write(c);
          }
          if (ScratchXmitBytes > ScratchLength) {
              // everything got sent, including the checksum byte
              ScratchState = SCR_WAITING_FOR_HEADER;
          }
        break;
      case SCR_REC_COMMAND:
        if (ScratchXmitBytes < 3) {  // we didn't get the whole command yet
#if DEBUG_SD
          Serial.print("#R["); Serial.print(ScratchXmitBytes); Serial.print("]="); Serial.write(c); println();
#endif
          ScratchSDFileName[ScratchXmitBytes++] = c;
        }
        if (ScratchXmitBytes == 3) { // we got everything we need to process command
          if (c == 'S') { // stop recording if final letter of record transmission is a capital S
#if DEBUG_SD
            Serial.print("#Lx");
            Serial.println(SDGamepadRecordFile.position());  // if the file isn't open position will return -1
#endif
            SDScratchRecordFile.close();  // we're done
            SRecState = SREC_STOPPED;
            SDScratchRecordFileName[0] = 0;
            ScratchState = SCR_WAITING_FOR_HEADER;
          } else if (c == 'D') {
            // delete all record files if final character is D
#if DEBUG_SD
            Serial.println("#ERASED"); // leave this message even not in debug mode to confirm erase
#endif
            removeAllRecordFiles();
            SRecState = SREC_STOPPED;
            ScratchState = SCR_WAITING_FOR_HEADER;
          } else { // start recording
            Serial.println("#REC");
            SRecState = SREC_RECORDING; // scratch is officially recording all traffic going to the gamepad
            GRecState = REC_STOPPED;  // if recording was previously happening from the gamepad this stops it
                                      // although this really shouldn't happen since the gamepad shouldn't be responding to both
                                      // buttons and scratch at the same time.
            // if we're already recording to the same file, and that file is indeed open, don't do anything
            if (SDScratchRecordFileName[0] == ScratchSDFileName[0] &&
                SDScratchRecordFileName[1] == ScratchSDFileName[1] &&
                SDScratchRecordFileName[2] == ScratchSDFileName[2] &&
                SDScratchRecordFile) {

#if DEBUG_SD
                  Serial.println("#OPEN");
#endif
                  return dataread;
            }
            
            // If we get here, we're either not scratch recording, or recording to
            // the wrong file.
            
            // erase any prior recording on this same button sequence
            removeScratchRecordFile(ScratchSDFileName[0], ScratchSDFileName[1], ScratchSDFileName[2]);
            // open up the correct file
            openScratchRecordFile(ScratchSDFileName[0], ScratchSDFileName[1], ScratchSDFileName[2]);
            ScratchState = SCR_WAITING_FOR_HEADER;
#if DEBUG_SD
            Serial.print("#F="); Serial.println(SDScratchRecordFileName);
#endif
          }
       } // end of "if scratchxmitbytes == 3"
     } // end of switch
  } // end of main while
  return dataread;
}

void loop() {
  int matrix = scanmatrix();
  if (debugmode && matrix != -1) {
    Serial.print("#MA:");Serial.println(matrix);
  }

  if (priormatrix != matrix) {  // the matrix button pressed has changed from the prior loop iteration
    curmatrixstarttime = millis();  // used to detect long tap
    if (matrix != -1) {  // -1 means nothing was pressed
      // short beep for button press feedback
      setBeep(200,50);
    }
  }

  if (longClick && !priorLongClick) {  // we have detected a long click with no prior long click
    if (matrix < REC_RECORD) {  // don't beep if it's a record button
      setBeep(2000,100); // high pitch beep tells user they are in long click mode
#if DEBUG_BUTTONS
      Serial.println("#LONGCLICK");
#endif
    }
    priorLongClick = longClick;  // keep track of whether we are already long clicking
  }

  int serialinput = handleSerialInput();  // this would be commands coming from scratch over the hardware serial port
                                          // the return code is how many bytes of serial input were handled
#if DEBUG_SD
  if (serialinput && debugmode) {
    Serial.print("#SERINP="); Serial.println(serialinput);
  }
#endif

  // if the robot sends something back to us, print it to the serial port
  // this is likely to be sensor data for scratch or debugging info
  
  while (BlueTooth.available()) {
    Serial.write(BlueTooth.read());
  }

  // if we got commands from the serial port recently, we need to suppress commands coming from
  // the button pad for a moment, and also disable debug printing to the serial port
  
  if (serialinput > 0) {
    suppressButtonsUntil = millis() + 1000;
    // we also want to forcibly stop gamepad record/play mode if scratch is talking to the gamepad
    // because that would result in weirdness. Like, what does that even mean? If you want to record
    // Scratch commands there's a clean scratch block feature for that.
    GRecState = REC_STOPPED;
    SDGamepadRecordFile.close();
  }

  if (millis() < suppressButtonsUntil) {
    return; // we've given control to the serial port (Scratch) for the next one second
  }

  // if we get here we can finally handle the incoming button presses

  // The record/play buttons are treated specially by this switch statement.
  
  switch (matrix) {
     case REC_RECORD:
      // because this button takes on different meanings to avoid bouncing
      // we check to ensure nothing was previously pushed
      longClick = 0;
      if (priormatrix == -1) {
        if (GRecState == REC_RECORDING) {
          // if we were already recording, the record button causes a stop
          GRecState = REC_STOPPED;
          setBeep(BF_NOTIFY, BD_MED);
        } else { // GRecState was not REC_RECORDING so let's get set up to record
          if (!SDGamepadRecordFile) {
            // if it wasn't already open, open it for writing at the end
            SDGamepadRecordFile = SD.open(SDGamepadRecordFileName, FILE_WRITE);
          }
          if (SDGamepadRecordFile) {
            GRecState = REC_RECORDING;
            setBeep(BF_NOTIFY, BD_MED);
#if DEBUG_SD
            Serial.print("#OP:"); Serial.println(SDGamepadRecordFileName);
#endif
          } else {
            setBeep(BF_ERROR, BD_LONG); // error tone
          }

        }
      }    // end of "if (priormatrix == -1)
      break;

     case REC_PLAY:
      longClick = 0;
      if (priormatrix == -1) {  // again, this button takes on different meanings so debounce
        PlayLoopMode = 0;
        if (GRecState == REC_STOPPED) { // if the recording was stopped, interpret the button to mean "PLAY"
          // if it wasn't already opened, open it
          if (!SDGamepadRecordFile) {
            SDGamepadRecordFile = SD.open(SDGamepadRecordFileName, FILE_WRITE);
            SDGamepadRecordFile.seek(0);  // rewind to start of file
          }
          if (SDGamepadRecordFile) {
            GRecState = REC_PLAYING;
            setBeep(BF_NOTIFY, BD_MED);
          } else {
            setBeep(BF_ERROR, BD_LONG);
          }
        } else if (GRecState == REC_PLAYING) {
          // if the state was already playing, the hitting the play button pauses the current playback
          GRecState = REC_PAUSED;
          setBeep(BF_NOTIFY, BD_MED);
#if DEBUG_SD
          debug("#PS"); debugln();
#endif
        } else {
          GRecState = REC_PLAYING;
          setBeep(BF_NOTIFY, BD_MED);
        }
      } else if (millis() - curmatrixstarttime > 1000) { // long tap
        // if user is still holding the play button 1 second later we
        // enter loop mode
        PlayLoopMode = 1; 
      }
      break;

     case REC_REWIND:
      GRecState = REC_REWINDING;
      setBeep(BF_REWIND, BD_MED);
      break;

     case REC_ERASE:
      longClick = 0;
      if (millis() - curmatrixstarttime < 50) {
        setBeep(BF_ERASE, BD_SHORT);
      } else if (millis() - curmatrixstarttime > 2000) { // very long tap required to erase for safety
        SDGamepadRecordFile.close();
        SD.remove(SDGamepadRecordFileName);
#if DEBUG_SD
        Serial.println("#ERS");
#endif       
        setBeep(BF_ERASE, BD_LONG);
      }
      break;

      default:  // -1 or any W, D, F mode there is nothing to do
        break;
  }

  priormatrix = matrix;
  
  CurDpad = decode_button(analogRead(DpadPin));

  if (debugmode && CurDpad != 's') {
    Serial.print("#DPAD="); Serial.println(CurDpad);
  }

  //
  // The following code handles the Scratch recording to a mode button feature
  //

  if (longClick && (CurCmd == 'W' || CurCmd == 'D' || CurCmd == 'F')) {
    // see if a special command is already in progress
    if (SRecState == SREC_PLAYING && 
        SDScratchRecordFileName[0] == CurCmd && 
        SDScratchRecordFileName[1] == CurSubCmd && 
        SDScratchRecordFileName[2] == CurDpad) {
          // if all those conditions are met then we're in the middle of playing
          // a recorded mode button action and everything is as it should be
#if DEBUG_SD
          // PL = playing current button recording already 
          Serial.print("#PLC:");Serial.print(CurCmd);Serial.print(CurSubCmd);Serial.println(CurDpad);
#endif
    } else {
          // if we get here, we're supposed to be playing a special command, however
          // either none is currently playing or the wrong one is playing
          SDScratchRecordFile.close();
          SRecState = SREC_STOPPED;
#if DEBUG_SD
          Serial.print("#PL:");Serial.print(CurCmd);Serial.print(CurSubCmd);Serial.println(CurDpad);Serial.println(SDGamepadRecordFileName);
#endif
          // see if there exists a file for the current button sequence
          char cmdfile[4];
          cmdfile[0] = CurCmd; cmdfile[1] = CurSubCmd; cmdfile[2] = CurDpad; cmdfile[3] = 0;
          if (SD.exists(cmdfile)) {
            // it does exist so it should be opened and we should go into play mode
            openScratchRecordFile(CurCmd, CurSubCmd, CurDpad);

            SRecState = SREC_PLAYING;
#if DEBUG_SD
            Serial.print("#@");Serial.println(SDGamepadRecordFileName);
#endif
          } else {
#if DEBUG_SD
            Serial.print("#NOPL:");Serial.print(cmdfile);Serial.print("/");Serial.print(CurCmd);Serial.print(CurSubCmd);Serial.println(CurDpad);
#endif
            // There is no recording for the current button sequence so we should drop out of play mode
            SRecState = SREC_STOPPED;
          }
        }
  }  // END OF LONGCLICK HANDLER

  RecordPlayHandler();  // handle the record/play mode

  if (millis() > NextTransmitTime  && GRecState != REC_PLAYING & SRecState != SREC_PLAYING ) { // don't transmit joystick controls during replay mode!

    // Packet consists of:
    // Byte 0: The letter "V" is used as a header. (Vorpal)
    // Byte 1: The letter "1" which is the version number of the protocol. In the future there may be different gamepad and
    //         robot protocols and this would allow some interoperability between robots and gamepads of different version.
    // Byte 2: L, the length of the data payload in the packet. Right now it is always 8. This number is
    //          only the payload bytes, it does not include the "V", the length, or the checksum
    // Bytes 3 through 3+L-1  The actual data.
    // Byte 3+L The base 256 checksum. The sum of all the payload bytes plus the L byte modulo 256.
    //
    // Right now the 
    //

    if (debugmode) {
      Serial.print("#S="); Serial.print(CurCmd); Serial.print(CurSubCmd); Serial.println(CurDpad);
    }
    BlueTooth.print("V1"); // Vorpal hexapod radio protocol header version 1
    int eight=8;
    BlueTooth.write(eight);
    BlueTooth.write(CurCmd);
    BlueTooth.write(CurSubCmd);
    BlueTooth.write(CurDpad);

    unsigned int checksum = sendbeep();

    checksum += eight+CurCmd+CurSubCmd+CurDpad;
    checksum = (checksum % 256);
    BlueTooth.write(checksum);
    //BlueTooth.flush();
    
    setBeep(0,0); // clear the current beep because it's been sent now
    
    NextTransmitTime = millis() + 100;
  }
}
