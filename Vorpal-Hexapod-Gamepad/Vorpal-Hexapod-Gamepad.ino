// Copyright (C) 2017 Vorpal Robotics, LLC.

//
// This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
// Attribution for derivations of this work should be made to: Vorpal Robotics, LLC
//
// You may use this work for noncommercial purposes without cost.
// For information on licensing this work for commercial purposes, please send email to support@vorpalrobotics.com
//

// This version fixes some issues with Scratch Record/Play and adds "trim mode" support for fine adjustment of servos.

// Trim mode: hold down W1 and W2 at the same time power is turned on and you enter Trim mode.
// Trim mode functions:
// DPAD Special button: advance trim to next leg. The leg twitches to indicate which leg is being modified.
// DPAD Forward: move knee up 1 microsecond
// DPAD Backward: move knee down 1 microsecond
// DPAD LEFT: move hip left 1 microsecond
// DPAD RIGHT: move hip right 1 microsecond
// R4: (Erase button) Hold down for 3 seconds to erase all trim settings. Robot will return to 0 trim on all servos.
// R1: (Record button) Save all trim settings to robot EEPROM. Robot will beep to confirm it worked.
// Maximum trim is + or - 120 microseconds. This is quite a lot of trim capability, nearly 11 degrees either way, which is
// more than enough to compensate for the + or - 8 degree accuracy of the servo splines on an MG90

const char *Version = "#V1r8j";

int debugmode = 0;  // Set to 1 to get more debug messages. Warning: this may make Scratch unstable so don't leave it on.

#define DEBUGSD  0   // Set this to 1 if you want debugging info about the SD card and record/play functions.
                     // This is very verbose and may affect scratch performance so only do it if you have to.
                     // It also takes up a lot of memory and you might get warnings from Arduino about too little
                     // RAM being left.

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

// definitions to decode the 4x4 button matrix

#define MATRIX_ROW_START 6
#define MATRIX_COL_START 2
#define NROW 4
#define NCOL 4

long suppressButtonsUntil = 0;   // default is not to suppress until we see serial data
int verbose = 0;
File SDGamepadRecordFile;        // REC.txt, holds the gamepad record/play file
char SDGamepadRecordFileName[7];

char ModeChars[] = {'W', 'D', 'F', 'R'};
char SubmodeChars[] = {'1', '2', '3', '4'};

char CurCmd = ModeChars[0];  // default is tripod gait
char CurSubCmd = SubmodeChars[0]; // default is primary submode
char CurDpad = 's'; // default is stop
unsigned int BeepFreq = 0;   // frequency of next beep command, 0 means no beep, should be range 50 to 2000 otherwise
unsigned int BeepDur = 0;    // duration of next beep command, 0 means no beep, should be in range 1 to 30000 otherwise

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
    Serial.println("");
  }
}

// for now we will just return the first button found

int priorMatrix = -1;
long curMatrixStartTime = 0;
int longClick = 0;  // this will be set to 1 if the last matrix button pressed was held a long time
int priorLongClick = 0; // used to track whether we should beep to indicate new longclick detected.

#define LONGCLICKMILLIS 500

int scanmatrix() {
  // we will energize row lines then read column lines

  // first set all rows to high impedance mode
  for (int row = 0; row < NROW; row++) {
    pinMode(MATRIX_ROW_START+row, INPUT);
  }
  // set all columns to pullup inputs
  for (int col = 0; col < NCOL; col++) {
    pinMode(MATRIX_COL_START+col, INPUT_PULLUP);
  } 

  // read each row/column combo until we find one that is active
  for (int row = 0; row < NROW; row++) {
    // set only the row we're looking at output low
    pinMode(MATRIX_ROW_START+row, OUTPUT);
    digitalWrite(MATRIX_ROW_START+row, LOW);
    
    for (int col = 0; col < NCOL; col++) {
      delayMicroseconds(100);
      if (digitalRead(MATRIX_COL_START+col) == LOW) {
        // we found the first pushed button
        if (row < 3) {
          CurCmd = ModeChars[row];
          CurSubCmd = SubmodeChars[col];
        }
        int curmatrix = row*NROW+col;
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


char decode_button(int b) {

  //Serial.print("DPAD: "); Serial.println(b);
  
  if (b < 100) {
     return 'b'; //*
  } else if (b < 200) {
     return 'l'; //*
  } else if (b < 400) {
    return 'r'; //* 
  } else if (b < 600) {
    return 'f'; //*
  } else if (b < 850) {
    return 'w';
  } else {
    return 's';
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
//   RECORD/PLAY FEATURE
/////////////////////////////////////////////////////////

// States for the record/play function
#define REC_STOPPED 0
#define REC_RECORDING 1
#define REC_PLAYING 2
#define REC_PAUSED 3
#define REC_REWINDING 4
#define REC_ERASING 5

#define REC_MAXLEN (40000)  // we will arbitrarily limit recording time to about 1 hour
#define REC_FRAMEMILLIS 100   // time between data frames when recording/playing
#define REC_FRAMESIZE 13       // number of bytes in a frame

int RecState = REC_STOPPED;
long RecNextEventTime = 0; // next time to record or play an event if using record/play mode

#define RECORDFILEISOPEN (SDGamepadRecordFile != NULL && SDGamepadRecordFileName[0] != '\0')

int count = 0;  // used to limit debug output




long NextTransmitTime = 0;  // next time to send a command to the robot
char PlayLoopMode = 0;
int BTFrameSize = REC_FRAMESIZE;
int Zero = 0;

void setBeep(int f, int d) {
  // schedule a beep to go out with the next transmission
  // this is not quite right as there can only be one beep per transmission
  // right now so if two different subsystems wanted to beep at the same time
  // whichever one is scheduled last would win. 
  // But because 10 transmits go out per second this seems sufficient, and its simple
  BeepFreq = f;
  BeepDur = d;
}

// Open a recording file.
// Closes any prior recording file first. This may be the same file, if so then this
// effectively just rewinds it.
// Returns 1 if the open worked, otherwise 0
//
int openRecordFile(char *cmd, char *subcmd, char *dpad) {
  closeRecordFile();  // close whatever one is currently in progress, if any.

  SDGamepadRecordFileName[0] = cmd;
  SDGamepadRecordFileName[1] = subcmd;
  SDGamepadRecordFileName[2] = dpad;
 
  SDGamepadRecordFile = SD.open(SDGamepadRecordFileName, FILE_WRITE);
  if (SDGamepadRecordFile) {
    SDGamepadRecordFile.seek(0);
    return 1;
  } else {
    setBeep(100, 100); // error tone
    SDGamepadRecordFileName[0] = 0;
    Serial.println("#SDOF");
    return 0;
  }
}

int openRecIfNeeded() {
  if (RECORDFILEISOPEN && SDGamepadRecordFileName[0] == 'R') {
    return;
  }
  return openRecordFile('R', 'E', 'C');
}

void closeRecordFile() {
  if (RECORDFILEISOPEN) {
    SDGamepadRecordFile.flush();
    SDGamepadRecordFile.close();
  }
  SDGamepadRecordFileName[0] = 0; // marker for closed file

}

long LastRecChirp;

void removeRecordFile(char *cmd, char *subcmd, char *dpad) {
  closeRecordFile();

  SDGamepadRecordFileName[0] = cmd;
  SDGamepadRecordFileName[1] = subcmd;
  SDGamepadRecordFileName[2] = dpad;
  SD.remove(SDGamepadRecordFileName);
  SDGamepadRecordFileName[0] = 0;
}

void removeAllRecordFiles() {
  char fname[8];
  char mode[] = "WDF";
  char num[] = "1234";
  char dpad[] = "FBLRSW";
  strcpy(&fname[3], ".txt");
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
}

void eraseRecording() {
  removeRecordFile('R', 'E', 'C');
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

void RecordPlayHandler() {
  // We save records consisting of a fixed length (currently 13 bytes).
  // This allows easy transmission by just putting the "V1" header out followed by the
  // length and data, followed by the checksum to bluetooth.

  switch (RecState) {
    case REC_STOPPED:
#if DEBUGSD
      Serial.println("#RECSTOP");
#endif
      // make sure file is flushed
      closeRecordFile();
      break;
      
    case REC_PAUSED:
      // nothing to do!

      break;

    ////////////////////////////////////////////////////////////
    case REC_PLAYING: {
        if (!RECORDFILEISOPEN) {
          return;
        }
        // chirp once every second to remind user they are playing a recording
        long t = millis();
        if ((t-LastRecChirp) >= 1000) {
           setBeep(BF_RECORD_CHIRP, BD_CHIRP);
           LastRecChirp = t;
        }
        
        if (millis() < RecNextEventTime) {
          // it's not time to send the next record yet
          return;
        }
#if DEBUGSD
        Serial.println("#RECPLAY");
#endif
        RecNextEventTime = millis() + REC_FRAMEMILLIS;
#if DEBUGSD
        Serial.print("#PLAY:"); Serial.print(SDGamepadRecordFileName);Serial.print("@");Serial.print((long)SDGamepadRecordFile.position()); Serial.println("");
#endif
        int length = SDGamepadRecordFile.read();
#if DEBUGSD
        Serial.print("#RECLEN="); Serial.println(length);
#endif
        if (length <= 0 || SDGamepadRecordFile.available() < length+1) { // we are at the end of the file
          if (PlayLoopMode) {
            // rewind to start of file and just keep going in this state to loop
            SDGamepadRecordFile.seek(0);
            length = SDGamepadRecordFile.read();
            Serial.println("#SDLOOP");
          } else {
            RecState = REC_STOPPED;
            return;
          }
        }
        // if we get here, there is a full frame to send to the robot
        BlueTooth.print("V1");
        BlueTooth.write(length+5);  // include 5 more bytes for beep
#if DEBUGSD
        Serial.print("#SNDV1:Len="); Serial.println(length+5);
#endif
        {
            int checksum = length+5;  // the length byte is included in the checksum
            for (int i = 0; i < length && SDGamepadRecordFile.available()>0; i++) {
              int c = SDGamepadRecordFile.read();
              checksum += c;
              BlueTooth.write(c);
#if DEBUGSD
              Serial.print("#");Serial.write(c);Serial.print("(");Serial.print(c);Serial.print(")");
#endif
            }
            checksum += sendbeep();
            setBeep(0,0); // clear the beep
            checksum = (checksum % 256);
            BlueTooth.write(checksum);
#if DEBUGSD
            Serial.print("#SNTCHKSUM:"); Serial.println(checksum);
#endif
        }

#if DEBUGSD
        Serial.print("#SDPLAY@"); Serial.print(SDGamepadRecordFileName); Serial.println(SDGamepadRecordFile.position());
#endif
      }
      break;

    /////////////////////////////////////////////////////////
    case REC_RECORDING:
      openRecIfNeeded();
      // chirp once every second to remind user they are recording
      if ((millis()-LastRecChirp) >= 1000) {
         setBeep(BF_RECORD_CHIRP, BD_CHIRP);
         LastRecChirp = millis();
      }

      if (millis() < RecNextEventTime) {
        // it's not time to record a frame yet
        return;
      }
#if DEBUGSD
      Serial.print("REC@"); Serial.println(SDGamepadRecordFile.position());
#endif
      RecNextEventTime = millis() + REC_FRAMEMILLIS;
      { // local variables require a scope
        int three = 3;  // yeah this is weird but trust me
        SDGamepadRecordFile.write(three);
        SDGamepadRecordFile.write(CurCmd);
        SDGamepadRecordFile.write(CurSubCmd);
        SDGamepadRecordFile.write(CurDpad);
      }
      // we don't record headers or checksums. the SD card is considered a reliable device
#if  DEBUGSD
      Serial.print("#SDRECSZ="); Serial.println(SDGamepadRecordFile.size());
#endif
      
      break;

    ////////////////////////////////////////////////////////////
    case REC_ERASING:
      eraseRecording();
      break;
      
    case REC_REWINDING:
      RecNextEventTime = 0;
      RecState = REC_STOPPED;
      if (SDGamepadRecordFile) {
        SDGamepadRecordFile.flush();
        SDGamepadRecordFile.seek(0);       
      }

      Serial.println("#SDRW");

      break;

    //////////////////////////////////////////////////////////
    default:
      Serial.println("#ERR_STATE: "); 
      Serial.print(RecState); Serial.println("");
      break;
  }
}

void setup() {
  // make a characteristic flashing pattern to indicate the gamepad code is loaded.
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(200);
  digitalWrite(13, LOW);
  delay(200);
  digitalWrite(13, HIGH);
  delay(400);
  digitalWrite(13,LOW);
  // after this point you can't flash the led on pin 13 because we're using it for SD card

  BlueTooth.begin(38400);
  Serial.begin(9600);
  if (debugmode) {
    Serial.println(Version);
  }
  pinMode(A0, OUTPUT);  // extra ground for additional FTDI port if needed
  digitalWrite(A0, LOW);
  pinMode(VCCA1, OUTPUT);
  pinMode(GNDA1, OUTPUT);
  digitalWrite(GNDA1, LOW);
  digitalWrite(VCCA1, HIGH);
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH); // chip select for SD card
  
  if (!SD.begin(10)) {
    Serial.println("#SDBF");    // SD Begin Failed
    return;
  }
  strcpy(SDGamepadRecordFileName, "REC.txt");   // record/play filename
}

int priormatrix = -1;
long curmatrixstarttime = 0;  // used to detect long tap for play button and erase button

  // Scratch integration: If anything appears on the serial input,
  // it's probably coming from a scratch program, so simply send it
  // out to the robot, unless it appears to be a command to start
  // recording on the gamepad

#define SCR_WAITING_FOR_HEADER  0
#define SCR_WAITING_FOR_HEX_1   1
#define SCR_WAITING_FOR_REC_1   2
#define SCR_WAITING_FOR_LENGTH  3
#define SCR_HEX_TRANSFER        4
#define SCR_REC_COMMAND         5

#define SCR_MAX_COMMAND_LENGTH  80    // we never expect scratch to send more than 16 commands at a time and max command len is 5 bytes
                                      // 16 commands is enough to send a move individually to every servo, plus a beep, plus a sensor
                                      // request, plus a couple more besides that. At 80 bytes the packet would require about 20 millisec
                                      // to send over bluetooth, which allows plenty of time for the hexapod to send back the sensor
                                      // data

int ScratchState = SCR_WAITING_FOR_HEADER;
int ScratchXmitBytes = 0; // how many packet bytes did we get so far
int ScratchLength = 0;    // length received in packet
char ScratchSDFileName[3];

int handleSerialInput() {

  int dataread = 0;
  
  while (Serial.available()>0) {
    int c = Serial.read();
    //Serial.print("C="); Serial.write(c); Serial.print("/"); Serial.print(c); Serial.println("");
    dataread++;
    
    switch (ScratchState) {
      case SCR_WAITING_FOR_HEADER:
        if (c == 'V') {
          ScratchState = SCR_WAITING_FOR_HEX_1;
        } else if (c == 'R') {
          ScratchState = SCR_WAITING_FOR_REC_1;
          Serial.println("#GOT_R_");
        } else {
          Serial.print("#SCRERR:H:"); Serial.print(c); Serial.print("("); Serial.write(c); Serial.println(")");
        }
        break;
      case SCR_WAITING_FOR_HEX_1:
      case SCR_WAITING_FOR_REC_1:
        if (c != '1') {
          Serial.print("#SCRERR:1:"); Serial.print(c); Serial.print("("); Serial.write(c); Serial.println(")");
          if (c == 'V') {
            ScratchState = SCR_WAITING_FOR_HEX_1;
          } else if (c == 'R') {
            ScratchState = SCR_WAITING_FOR_REC_1;
          } else {
            ScratchState = SCR_WAITING_FOR_HEADER;
          }
        } else if (ScratchState == SCR_WAITING_FOR_HEX_1) {
          ScratchState = SCR_WAITING_FOR_LENGTH;
        } else {  // state should be SCR_WAITING_FOR_REC_1 here
          ScratchState = SCR_REC_COMMAND;
          ScratchXmitBytes = 0;   // we expect 3 more bytes to tell us what button to record to
                                  // this will either be a command spec like: W1f or F2s, or it
                                  // will be SSS to mean stop recording, or DDD to mean delete all recordings
#if DEBUGSD
          Serial.println("#SCRRECWFXM");
#endif
        }
        break;
      case SCR_WAITING_FOR_LENGTH:
          // This is a hexapod command not a REC command
          ScratchXmitBytes = 0;  // how many have we gotten
          ScratchLength = c;    // how many do we expect
          if (ScratchLength > SCR_MAX_COMMAND_LENGTH) {
            // command is longer than ever expected from scratch so something is wrong
            ScratchState = SCR_WAITING_FOR_HEADER;  // punt
            Serial.print("#SCRERR:LenMax:"); Serial.println(c);  
          } else {
            BlueTooth.print("V1");
            BlueTooth.write(c);
            if (SDGamepadRecordFileName[0] != 'R' && SDGamepadRecordFileName[0] != 0 
                && SDGamepadRecordFile != NULL) {
              // we're also recording so save the length byte
              SDGamepadRecordFile.write(c);
            }
            ScratchState = SCR_HEX_TRANSFER;
          }
          break;
      case SCR_HEX_TRANSFER:
          ScratchXmitBytes++;
          BlueTooth.write(c);
          if (SDGamepadRecordFile && SDGamepadRecordFileName[0] != 'R' && SDGamepadRecordFileName != 0
                  && ScratchXmitBytes <= ScratchLength) {
            // In this case we are also in record mode so save the data to the recording file.
            // We use <= ScratchLength in the above condition because we don't want to put the 
            // checksum byte on there because that's not normally stored in
            // recording files (similar to the way the V1 header is not stored).
            SDGamepadRecordFile.write(c);
          }
          if (ScratchXmitBytes > ScratchLength) {
              // everything got sent, including the checksum byte
              ScratchState = SCR_WAITING_FOR_HEADER;
          }
        break;
      case SCR_REC_COMMAND:
        if (ScratchXmitBytes < 3) {
#if DEBUGSD
          Serial.print("#RECCMD["); Serial.print(ScratchXmitBytes); Serial.print("]="); Serial.write(c); Serial.println("");
#endif
          ScratchSDFileName[ScratchXmitBytes++] = c;
        }
        if (ScratchXmitBytes == 3) { // we got everything we need to process command
          if (c == 'S') { // stop recording if final letter of record transmission is a capital S
#if DEBUGSD
            Serial.println("#SCRECSTP");
            Serial.print("#RECLEN=");
            if (SDGamepadRecordFile) {
              Serial.println(SDGamepadRecordFile.position());
            } else {
              Serial.println("ERR");
            }
#endif
            closeRecordFile();
            RecState = REC_STOPPED;
            ScratchState = SCR_WAITING_FOR_HEADER;
          } else if (c == 'D') {
            // delete all record files if final character is D
            Serial.println("#Recordings Erased"); // leave this message even not in debug mode to confirm erase
            removeAllRecordFiles();
            RecState = REC_STOPPED;
            ScratchState = SCR_WAITING_FOR_HEADER;
          } else { // start recording
            Serial.println("#SCRRECSTRT");
            // if we're already recording to the same file, don't do anything
            if (SDGamepadRecordFileName[0] == ScratchSDFileName[0] &&
                SDGamepadRecordFileName[1] == ScratchSDFileName[1] &&
                SDGamepadRecordFileName[2] == ScratchSDFileName[2] &&
                SDGamepadRecordFile != NULL) {

#if DEBUGSD
                  Serial.println("#SCRFILEOPEN");
#endif
                  return dataread;
            }
            // If we get here, we're either not recording, or recording to
            // the wrong file.
            // erase any prior recording on this same button sequence
            removeRecordFile(ScratchSDFileName[0], ScratchSDFileName[1], ScratchSDFileName[2]);
            // open it up again
            openRecordFile(ScratchSDFileName[0], ScratchSDFileName[1], ScratchSDFileName[2]);
            ScratchState = SCR_WAITING_FOR_HEADER;
#if DEBUGSD
            Serial.print("#FILE="); Serial.println(SDGamepadRecordFileName);
#endif
            RecState = REC_STOPPED;  // if recording was previously happening from the gamepad this stops it
                                     // Scratch recording does not use the same code as gamepad record feature
          }
       } // end of "if scratchxmitbytes == 3"
     } // end of switch
  } // end of main while
  return dataread;
}

void loop() {
  if (debugmode) {delay(50);} // slow things down in debug mode.
  int matrix = scanmatrix();
  if (debugmode && matrix != -1) {
    Serial.print("#MA:");Serial.println(matrix);
  }

  if (priormatrix != matrix) {
    curmatrixstarttime = millis();  // used to detect long tap
    if (matrix != -1) {
      // short beep for button press feedback
      setBeep(200,50);
    }
  }

  if (longClick && !priorLongClick) {
    setBeep(2000,100); // click tells user they are in long click mode
    Serial.println("#LCBEEP");
    priorLongClick = longClick;
  }

  int serialinput = handleSerialInput();  // this would be commands coming from scratch over the hardware serial port

  if (serialinput && debugmode) {
    Serial.print("#SERINP="); Serial.println(serialinput);
  }
  // if the robot sends something back to us, print it to the serial port
  // this is likely to be sensor data for scratch or debugging info
  
  while (BlueTooth.available()) {
    Serial.write(BlueTooth.read());
  }

  // if we got commands from the serial port recently, we need to suppress commands coming from
  // the button pad for a moment, and also disable debug printing to the serial port
  if (serialinput > 0) {
    suppressButtonsUntil = millis() + 1000;
    if (debugmode) {
      Serial.println("#SUP");
    }
  }

  if (millis() < suppressButtonsUntil) {
    return; // we've given control to the serial port (Scratch) for the next one second
  }

  switch (matrix) {
     case REC_RECORD:
      // because this button takes on different meanings to avoid bouncing
      // we check to ensure nothing was previously pushed
      if (priormatrix == -1) {
        if (RecState == REC_RECORDING) {
          // if we were already recording, the record button causes a stop
          RecState = REC_STOPPED;
          setBeep(BF_NOTIFY, BD_MED);
        } else {
          RecState = REC_RECORDING;
          setBeep(BF_NOTIFY, BD_MED);
          openRecIfNeeded();
        }
      }

      break;

     case REC_PLAY:
      if (priormatrix == -1) {  // again, this button takes on different meanings so debounce
        PlayLoopMode = 0;
        if (RecState == REC_STOPPED) {
          RecState = REC_PLAYING;
          setBeep(BF_NOTIFY, BD_MED);
          openRecIfNeeded();
        } else if (RecState == REC_PLAYING) {
          RecState = REC_PAUSED;
          setBeep(BF_NOTIFY, BD_MED);
          //debug("REC PAUSED"); debugln();
        } else {
          RecState = REC_PLAYING;
          setBeep(BF_NOTIFY, BD_MED);
        }
      } else if (millis() - curmatrixstarttime > 1000) { // long tap
        // if user is still holding the play button 1 second later we
        // enter loop mode
        PlayLoopMode = 1; 
      }
      break;

     case REC_REWIND:
      RecState = REC_REWINDING;
      setBeep(BF_REWIND, BD_MED);
      break;

     case REC_ERASE:
      if (millis() - curmatrixstarttime < 50) {
        setBeep(BF_ERASE, BD_SHORT);
      } else if (millis() - curmatrixstarttime > 2000) { // long tap required to erase for safety
        eraseRecording();
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
    if (RecState == REC_PLAYING && 
        SDGamepadRecordFileName[0] == CurCmd && 
        SDGamepadRecordFileName[1] == CurSubCmd && 
        SDGamepadRecordFileName[2] == CurDpad) {
          // if all those conditions are met then we're in the middle of playing
          // a recorded mode button action and everything is as it should be
#if DEBUGSD
          Serial.print("#PLMCUR:");Serial.print(CurCmd);Serial.print(CurSubCmd);Serial.println(CurDpad);
#endif
    } else {
          // if we get here, we're supposed to be playing a special command, however
          // either none is currently playing or the wrong one is playing
          closeRecordFile(); // close whatever one is playing
          RecState = REC_STOPPED;
#if DEBUGSD
          Serial.print("#PLM:Setup:");Serial.print(CurCmd);Serial.print(CurSubCmd);Serial.println(CurDpad);Serial.println(SDGamepadRecordFileName);
#endif
          // see if there exists a file for the current button sequence
          char cmdfile[8];
          cmdfile[0] = CurCmd; cmdfile[1] = CurSubCmd; cmdfile[2] = CurDpad; cmdfile[3] = 0;
          strcat(cmdfile, ".txt");
          if (SD.exists(cmdfile)) {
            // it does exist so it should be opened and we should go into play mode
            openRecordFile(CurCmd, CurSubCmd, CurDpad);

            RecState = REC_PLAYING;
            PlayLoopMode = 1;  // playing from a button causes looping
#if DEBUGSD
            Serial.print("#PLMPLAY@");Serial.println(SDGamepadRecordFileName);
#endif
          } else {
#if DEBUGSD
            Serial.print("#NOPLM:");Serial.print(cmdfile);Serial.print("/");Serial.print(CurCmd);Serial.print(CurSubCmd);Serial.println(CurDpad);
#endif
            // There is no recording for the current button sequence so we should drop out of play mode
            RecState = REC_STOPPED;
            PlayLoopMode = 0;
          }
        }
  }  // END OF LONGCLICK HANDLER

  RecordPlayHandler();  // handle the record/play mode

  if (millis() > NextTransmitTime  && RecState != REC_PLAYING  ) { // don't transmit joystick controls during replay mode!

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
