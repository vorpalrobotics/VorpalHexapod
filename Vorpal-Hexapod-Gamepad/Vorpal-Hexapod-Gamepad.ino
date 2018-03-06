// Copyright (C) 2017 Vorpal Robotics, LLC.

//
// This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/.
// Attribution for derivations of this work should be made to: Vorpal Robotics, LLC
//
// You may use this work for noncommercial purposes without cost.
// For information on licensing this work for commercial purposes, please send email to support@vorpalrobotics.com
//

char *Version = "#V1r8d";

int debugmode = 0;   // make this 1 for debug mode. NOTE: debug mode may make Scratch unstable, don't leave it on!

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
// M16: Erase. This has to be held for 3 seconds.
//
// While recording the internal beeper makes a high pitched short chirp
// every couple of seconds. Because this is recording to an SD card,
// and it only takes 130 bytes for 1 second or recording, you have
// tons of recording time. We internally limit it to about 1 hour
// just so people don't forget they have record on and fill up their card.
//
// While playing back the internal beeper will make a low pitched beep
// every couple of seconds.
//
// Erase has to be held for several full seconds. A long deep tone indicates the erase
// occurred.
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
char *SDGamepadRecordFileName = "REC.txt";
File SDScratchRecordFile; // there is potentially one of these per button, but we will only use one at a time
                          // they are named for the button combination that plays the recording. 
                          // For example W1f.txt is Walk mode 1 forward button

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
          longClick = 0;
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

void eraseRecording() {
      //debug("ERASING"); debugln();
      // to erase the recording, close the file, remove the file, then re-open it again
      //Serial.println("#SDERASE");
      SDGamepadRecordFile.close();
      SD.remove(SDGamepadRecordFileName) || Serial.println("#SDRMF");  // SD Erase Failed
      SDGamepadRecordFile = SD.open(SDGamepadRecordFileName, FILE_WRITE);
      if (SDGamepadRecordFile) {
        SDGamepadRecordFile.seek(0);  // in theory it should already be at 0, but not if the remove above failed for some reason.
      } else {
        Serial.println("#SDOF"); // SD Open Failed
      }
}

void RecordPlayHandler() {
  // We save records consisting of a fixed length (currently 13 bytes).
  // This allows easy transmission by just putting the "V1" header out followed by the
  // length and data, followed by the checksum to bluetooth.

  // if the recording file failed to open (maybe SD card is not inserted or hardware problem)
  // then it's pointless to do anything in this function, just return.
  
  if (! SDGamepadRecordFile) {
    // We don't seem to have an SD card available so try to open it
    SDGamepadRecordFile = SD.open(SDGamepadRecordFileName, FILE_WRITE);
    if (SDGamepadRecordFile) {
      SDGamepadRecordFile.seek(0);
    } else {

      // it seems hopeless, we can't open the card.
      // So, exit record/play handler
      RecState = REC_STOPPED;
      longClick = 0;
      return;
    }
  }

  switch (RecState) {
    case REC_STOPPED:
      //Serial.println("REC STOPPED");
      // make sure file is flushed
      SDGamepadRecordFile.flush();
      SDGamepadRecordFile.close();
      SDGamepadRecordFile = SD.open(SDGamepadRecordFileName, FILE_WRITE);
      if (SDGamepadRecordFile) {
        SDGamepadRecordFile.seek(0);
      }
      break;
      
    case REC_PAUSED:
      // nothing to do!

      break;

    ////////////////////////////////////////////////////////////
    case REC_PLAYING: {

        // chirp once every two seconds to remind user a recording is playing
        if ((millis()%2000) > 1990) {
           setBeep(BF_PLAY_CHIRP, BD_CHIRP);
        }
        
        if (millis() < RecNextEventTime) {
          // it's not time to send the next record yet
          return;
        }
        RecNextEventTime = millis() + REC_FRAMEMILLIS;
        //debug("PLAYING AT "); debug(RecPosition); debugln();
        int length = SDGamepadRecordFile.read();
        
        if (length <= 0 || SDGamepadRecordFile.available() < length+1) { // we are at the end of the file
          if (PlayLoopMode) {
            // rewind to start of file and just keep going in this state to loop
            if (SDGamepadRecordFile) {
              SDGamepadRecordFile.seek(0);
              length = SDGamepadRecordFile.read();
            }
            //Serial.println("#SDLOOP");
          } else {
            RecState = REC_STOPPED;
            return;
          }
        }
        // if we get here, there is a full frame to send to the robot
        BlueTooth.print("V1");
        BlueTooth.write(length);
        //Serial.print("#SNDV1:Len="); Serial.println(length);
        {
            int checksum = length;  // the length byte is included in the checksum
            for (int i = 0; i < length && SDGamepadRecordFile.available()>0; i++) {
              int c = SDGamepadRecordFile.read();
              checksum += c;
              BlueTooth.write(c);
              //Serial.write(c);Serial.print("(");Serial.print(c);Serial.print(")");
            }
            checksum = (checksum % 256);
            BlueTooth.write(checksum);
            //Serial.print("#SNTCHKSUM:"); Serial.println(checksum);
        }
       
        //Serial.print("#SDPLAY@"); Serial.println(SDGamepadRecordFile.position());
      }
      break;

    /////////////////////////////////////////////////////////
    case REC_RECORDING:

      // chirp once every two seconds to remind user they are recording
      if ((millis()%2000) > 1990) {
         setBeep(BF_RECORD_CHIRP, BD_CHIRP);
      }

      if (millis() < RecNextEventTime) {
        // it's not time to record a frame yet
        return;
      }
      //Serial.print("REC@"); Serial.println(SDGamepadRecordFile.position());
      RecNextEventTime = millis() + REC_FRAMEMILLIS;
      { // local variables require a scope
        int three = 3;  // yeah this is weird but trust me
        SDGamepadRecordFile.write(three);
        SDGamepadRecordFile.write(CurCmd);
        SDGamepadRecordFile.write(CurSubCmd);
        SDGamepadRecordFile.write(CurDpad);
      }
      // we don't record headers or checksums. the SD card is considered a reliable device and
      // it has its own error detection
      //Serial.print("#SDRECSZ="); Serial.println(SDGamepadRecordFile.size());
      
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

      //Serial.println("#SDRW");

      break;

    //////////////////////////////////////////////////////////
    default:
      //debug("ERROR UNKNOWN REC/PLAY STATE: "); 
      //debug(RecState); debugln();
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
  pinMode(A0, OUTPUT);  // extra ground for additional FTDI port
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

  // open the record file for writing, it will be retained even across gamepad boots.
  SDGamepadRecordFile = SD.open(SDGamepadRecordFileName, FILE_WRITE); // we will keep this file open at all times

  if (SDGamepadRecordFile) {
    SDGamepadRecordFile.seek(0);  // rewind it if it already exists, so the PLAY button will immediately work.
    //Serial.print("#SDSZ="); Serial.println(SDGamepadRecordFile.available());
  } else {
    setBeep(BF_ERROR, BD_LONG);
    Serial.println("#SDOF");    // SD Open Failed
  }

}

int priormatrix = -1;
long curmatrixstarttime = 0;  // used to detect long tap for play button and erase button

void StopScratchRecording() {
  if (SDGamepadRecordFile) {
    SDGamepadRecordFile.close();
  }
  RecState = REC_STOPPED;
  // reset name of file to default
  SDGamepadRecordFileName[0] = 'R';
  SDGamepadRecordFileName[1] = 'E';
  SDGamepadRecordFileName[2] = 'C';
  // open it again
  SDGamepadRecordFile = SD.open(SDGamepadRecordFileName, FILE_WRITE);
  if (SDGamepadRecordFile) {
    SDGamepadRecordFile.seek(0);
  }
}

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
        } else {
          Serial.print("#SCRERR:HDR:"); Serial.print(c); Serial.print("("); Serial.write(c); Serial.println(")");
        }
        break;
      case SCR_WAITING_FOR_HEX_1:
      case SCR_WAITING_FOR_REC_1:
        if (c != '1') {
          ScratchState = SCR_WAITING_FOR_HEADER;
        } else if (ScratchState == SCR_WAITING_FOR_HEX_1) {
          ScratchState = SCR_WAITING_FOR_LENGTH;
        } else {
          ScratchState = SCR_REC_COMMAND;
          ScratchXmitBytes = 0;   // we expect 3 more bytes to tell us what button to record to
                                  // this will either be a command spec like: W1f or F2s, or it
                                  // will be SSS to mean stop recording
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
            if (SDGamepadRecordFileName[0] != 'R' && SDGamepadRecordFile) {
              // we're also recording so save the length byte
              SDGamepadRecordFile.write(c);
            }
            ScratchState = SCR_HEX_TRANSFER;
          }
          break;
      case SCR_HEX_TRANSFER:
          ScratchXmitBytes++;
          BlueTooth.write(c);
          if (SDGamepadRecordFile && SDGamepadRecordFileName[0] != 'R' && ScratchXmitBytes <= ScratchLength) {
            // In this case we are also in record mode so save the data to the recording file.
            // We use ScratchLength-1 in the above condition because we don't want to put the 
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
          ScratchSDFileName[ScratchXmitBytes++] = c;
        }
        if (ScratchXmitBytes == 3) { // we got everything we need to process command
          if (c == 'S') { // stop recording if final letter of record transmission is a capital S
            StopScratchRecording();
            ScratchState = SCR_WAITING_FOR_HEADER;
            //Serial.println("#SCRECSTP");
          } else { // start recording
            // if we're already recording to the same file, don't do anything
            if (SDGamepadRecordFileName[0] == ScratchSDFileName[0] &&
                SDGamepadRecordFileName[1] == ScratchSDFileName[1] &&
                SDGamepadRecordFileName[2] == ScratchSDFileName[2]) {
                  return dataread;
            }
            // stop any prior recording that might be happening
            StopScratchRecording();
            
            for (int i = 0; i < 3; i++) {
              SDGamepadRecordFileName[i] = ScratchSDFileName[i];
            }
            SD.remove(SDGamepadRecordFileName); // erase any prior version
            SDGamepadRecordFile = SD.open(SDGamepadRecordFileName, FILE_WRITE);
            ScratchState = SCR_WAITING_FOR_HEADER;
            //Serial.print("#SCRECFILE="); Serial.println(SDGamepadRecordFileName);
            RecState = REC_STOPPED;  // if recording was previously happening from the gamepad this stops it
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

  if (priormatrix != matrix) {
    curmatrixstarttime = millis();  // used to detect long tap
    if (matrix != -1) {
      // short beep for button press feedback
      setBeep(200,50);
    }
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

  // if we get here then scratch appears not to have control.
  // close any scratch recording in progress
  if (SDGamepadRecordFileName[0] != 'R') {   
    // yeah this is kind of a hack but memory is tight so didn't want to make a new variable
    // The SD file name will be "REC.txt" if scratch did not initiate any recording (scratch's
    // recordings always start with W, D, or F indicating a mode button). So, if we get here
    // then scratch seems to have been recording and never closed off the recording session.
    
    //StopScratchRecording();  // this seemed to be causing trouble ... for further study. if scratch user properly programmed it's not needed
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
        }
      }

      break;

     case REC_PLAY:
      if (priormatrix == -1) {  // again, this button takes on different meanings so debounce
        PlayLoopMode = 0;
        if (RecState == REC_STOPPED) {
          RecState = REC_PLAYING;
          setBeep(BF_NOTIFY, BD_MED);
          if (SDGamepadRecordFile) {
            SDGamepadRecordFile.seek(0);
          }
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
          //Serial.print("#PLM:");Serial.print(CurCmd);Serial.print(CurSubCmd);Serial.println(CurDpad);
    } else {
          // if we get here, we're supposed to be playing a special command, however
          // either none is currently playing or the wrong one is playing
          SDGamepadRecordFile.close();  // close any existing file that's playing
          //Serial.print("#PLM:Setup:");Serial.print(CurCmd);Serial.print(CurSubCmd);Serial.println(CurDpad);Serial.println(SDGamepadRecordFileName);

          // see if there exists a file for the current button sequence
          char cmdfile[8];
          cmdfile[0] = CurCmd; cmdfile[1] = CurSubCmd; cmdfile[2] = CurDpad; cmdfile[3] = 0;
          strcat(cmdfile, ".txt");
          if (SD.exists(cmdfile)) {
            // it does exist so it should be opened and we should go into play mode
            strncpy(SDGamepadRecordFileName, cmdfile, 7);
            SDGamepadRecordFile = SD.open(SDGamepadRecordFileName, FILE_WRITE);
            if (SDGamepadRecordFile) {
              SDGamepadRecordFile.seek(0);              
            }

            RecState = REC_PLAYING;
            PlayLoopMode = 1;  // playing from a button causes looping
            //Serial.print("#PLMPLAY@");Serial.println(SDGamepadRecordFileName);
          } else {
            //Serial.print("#NOPLM:");Serial.print(cmdfile);Serial.print("/");Serial.print(CurCmd);Serial.print(CurSubCmd);Serial.println(CurDpad);
            // There is no recording for the current button sequence so we should drop out of play mode
            RecState = REC_STOPPED;
            PlayLoopMode = 0;
          }
        }
  }

  RecordPlayHandler();  // handle the record/play mode
  
  if (!SDGamepadRecordFile && millis() > 12000 && millis() < 13000) {
    // do a long beep if we can't open the SD card. The wait until millis is over 10 seconds
    // is to make sure we have a bluetooth connection so the robot actually gets the beep
    setBeep(BF_ERROR, BD_VERYLONG);
  }

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
    
    unsigned int beepfreqhigh = highByte(BeepFreq);
    unsigned int beepfreqlow = lowByte(BeepFreq);
    BlueTooth.print("B");
    BlueTooth.write(beepfreqhigh);
    BlueTooth.write(beepfreqlow);

    unsigned int beepdurhigh = highByte(BeepDur);
    unsigned int beepdurlow = lowByte(BeepDur);
    BlueTooth.write(beepdurhigh);
    BlueTooth.write(beepdurlow);

    unsigned int checksum = eight+CurCmd+CurSubCmd+CurDpad+'B'+beepfreqhigh+beepfreqlow+beepdurhigh+beepdurlow;
    checksum = (checksum % 256);
    BlueTooth.write(checksum);
    //BlueTooth.flush();
    
    setBeep(0,0); // clear the current beep because it's been sent now
    
    NextTransmitTime = millis() + 100;
  }

}
