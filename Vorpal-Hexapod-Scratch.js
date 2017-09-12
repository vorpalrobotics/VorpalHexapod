// Vorpal Combat Hexapod Control Program For Scratch X V1.0d
//
// Copyright (C) 2017 Vorpal Robotics, LLC.
//
// This work is licensed under the Creative Commons Attribution-ShareAlike 4.0 International License.
// To view a copy of this license, visit http://creativecommons.org/licenses/by-sa/4.0/.
// Attribution for derivations of this work should be made to: Vorpal Robotics, LLC
//

(function(ext) {
    var device = null;
    var rawData = null;
    var trace = 0;	// make this nonzero to trace data packet reception

    //
    // finite state machine for input processing
    //
    const ST_START 	= 0;	// waiting for start of a new packet
    const ST_COM 	= 1;	// stripping comments until next newline
    const ST_WAIT_1 	= 2;	// waiting for a "1" version number
    const ST_WAIT_LEN	= 3;	// waiting for the payload length byte
    const ST_PAYLOAD	= 4;	// reading the payload
    const ST_WAIT_CS	= 5;	// waiting for the checksum

    var InputState = ST_START;
    var packetLength = 0;
    var packetReceived = 0;
    var packetData = null;
    var curComment = null;	// the current partial comment
    // the current value of sensors
    // these are not changed until a complete packet is
    // received
    var SensorA3 = 0;
    var SensorA6 = 0;
    var SensorA7 = 0;
    var SensorDistance = 0;

    // cmucam5 support
    var SensorPixySignature = -1;
    var SensorPixyX = -1;
    var SensorPixyY = -1;
    var SensorPixyWidth = -1;
    var SensorPixyHeight = -1;

    ext.resetAll = function(){};

    function appendBuffer( buffer1, buffer2 ) {
	if (buffer1 == null) {
		return new Uint8Array(buffer2);
	}
        var tmp = new Uint8Array( buffer1.byteLength + buffer2.byteLength );
        tmp.set( new Uint8Array( buffer1 ), 0 );
        tmp.set( new Uint8Array( buffer2 ), buffer1.byteLength );
        return tmp.buffer;
    }

    // Extension API interactions
    var potentialDevices = [];
    ext._deviceConnected = function(dev) {
	console.log("DEV CONNECTED:" + dev.id);
        potentialDevices.push(dev);

        if (!device) {
            tryNextDevice();
        }
    };

    var poller = null;
    var watchdog = null;
    var curCmd = null;
    var curCmdRec = null;


    function setSimpleCommand(cmdbuf) {
	var cb = new Uint8Array(cmdbuf);
	// add four bytes for V1 header, length, and checksum
	// and add one more byte for "S" sensor request at end
	var c = new Uint8Array(5+cb.length);

	c[0] = "V".charCodeAt();
	c[1] = "1".charCodeAt();
	c[2] = cb.length+1;		// going to add a sensor request "S" at the end
	var checksum = cb.length+1;
	for (var i = 0; i < cb.length; i++) {
		c[3+i] = cb[i];
		checksum += cb[i];
	}
	c[cb.length+3] = "S".charCodeAt();	// sensor read request
	checksum += c[cb.length+3];

	checksum = checksum%256;

	c[cb.length+4] = checksum;
	curCmd = c;
	console.log("setsimple:" + curCmd.length);
    }


    function bin2String(array) {
     var result = "";
     for (var i = 0; i < array.length; i++) {
       result += String.fromCharCode(array[i]);
     }
     return result;
    }


    var deviceOpenedNotify = null;

    function deviceOpenedCallback(dev) {
        if (!dev) {
            device = null;
            tryNextDevice();
            return;
        }
        device.set_receive_handler(function(data) {
            if (trace) console.log("RCV:BYTELEN=" + data.byteLength);
	    rawData = appendBuffer(rawData, data);
            processData();

        });
	if (deviceOpenedNotify != null) {
		deviceOpenedNotify();
		deviceOpenedNotify = null;
	}

    console.log("SET RCV DATA HANDLER");
    //
    // send commands once every 100 ms, same as gamepad
    //

    if (poller == null) {
        poller = setInterval(function() {
                if (device != null) {
		if (curCmdRec != null) {
			// we have a record command, send that first
			device.send(curCmdRec.buffer);
			// these only get sent once
			console.log("sent REC command len=" + curCmdRec.length);
			curCmdRec = null;
		}
                if (curCmd != null) {
                    device.send(curCmd.buffer);
		    console.log("sent cmd len=" + curCmd.length);
		    // if it's a beep command, we will not retransmit
		    if (curCmd.length > 3 && curCmd[3] == "B".charCodeAt()) {
			    curCmd = null;
			    console.log("************Cleared BEEP command");
		    }
                }
            }
		//console.log("sent ping");
	}, 100);  // for debugging slow it down from 100 ms to 2000 ms
    }

    if (0) {  // for now we're just going to take the lowest com port and hope for the best
        watchdog = setTimeout(function() {
            // This device didn't get good data in time, so give up on it. Clean up and then move on.
            // If we get good data then we'll terminate this watchdog.
            clearInterval(poller);
            console.log("watchdog!");
            poller = null;
            //device.set_receive_handler(null);
            device.close();
            device = null;
            tryNextDevice();
        }, 5000);
    }
    }

    function tryNextDevice() {
        // If potentialDevices is empty, device will be undefined.
        // That will get us back here next time a device is connected.
        device = potentialDevices.shift();
        if (!device) return;

        device.open({ stopBits: 0, bitRate: 9600, ctsFlowControl: 0 }, deviceOpenedCallback);

    	console.log("TRYING DEV: " + device.id);


    }

    ext.startserial = function() {
        console.log("STARTSERIAL");
    };


    function processData() {


	    while (rawData != null && rawData.length > 0) {
		    var b = rawData[0];
		    rawData = rawData.subarray(1,rawData.length);
		    if (rawData.length == 0) {
			    rawData = null;
		    }

		    switch (InputState) {
		    case ST_START:
			    if (b == "V".charCodeAt()) {
				    if (trace) console.log("RCV:V");
				    InputState = ST_WAIT_1;
			    } else if (b == "#".charCodeAt()) {
				    if (trace) console.log("RCV:#");
				    InputState = ST_COM;
			    } else {
				    console.log("RCV:ERR:ST:"+b);
			    }
			    break;
		    case ST_WAIT_1:
			    if (b == "1".charCodeAt()) {
				    InputState = ST_WAIT_LEN;
				    if (trace) console.log("RCV:1");
			    } else {
				    console.log("RCV:ERR:W1:"+b);
				    // error so return to start state
				    InputState = ST_START;
			    }
			    break;
		    case ST_COM:
			    // Comment coming from the gamepad, 
			    // so flush everything up to a newline
			    if (b == 10 || b == 13) { 	// newline ascii code is 10, CR is 13
				    InputState = ST_START;
				    if (curComment != null) {
					    if (trace) console.log("#" + bin2String(curComment));
					    curComment = null;
				    }
			    } else {
				    // append the character to the current comment
				    // and we'll console.log it when its all received
				    var newbyte = new Uint8Array(1);
				    newbyte[0] = b;
				    curComment = appendBuffer(curComment, newbyte);
			    }
			    break;

		    case ST_WAIT_LEN:

			    packetLength = b;
			    packetReceived = 0;
			    packetData = new Uint8Array(b+0);
			    InputState = ST_PAYLOAD;
			    if (trace) console.log("RCV:Len="+b);
			    break;

		    case ST_PAYLOAD:
			    packetData[packetReceived++] = b;
			    if (trace) console.log("RCV:Payload byte" + packetReceived + " of " + packetLength + " val=" + b);
			    if (packetReceived == packetLength) {
				    // we got it all!
				    InputState = ST_WAIT_CS;
				    //console.log("RCV:incoming packet is complete");
			    }
			    break;

		    case ST_WAIT_CS:
			    // checksum must match packet received
			    var checksum = packetLength; // sum includes length and packet bytes
			    for (var i = 0; i < packetLength; i++) {
				    checksum += packetData[i];
			    }
			    checksum %= 256;	// mod 256 checksum

			    if (checksum != b) {
				    console.log("RCV:CHECKSUM FAIL GOT:" + b + " EXPECTED:" + checksum);
				    InputState = ST_START; // back to the drawing board
			    } else {
				    // we got a complete sensor data packet and it
				    // passes checksum! Cool beans!
				    // Now set the sensor values from the packet data
				    //console.log("RCV:Checksum is good");
				    if (packetLength == 8) {
					    // prototype hard codes 8 bytes
					    SensorA3 = packetData[0]*256+packetData[1];
					    SensorA6 = packetData[2]*256+packetData[3];
					    SensorA7 = packetData[4]*256+packetData[5];
					    SensorDistance = packetData[6]*256+packetData[7];

					    console.log("RCV:SENSORVALS:"+SensorA3+":"+SensorA6+":"+SensorA7+":"+SensorDistance);

					    if (packetLength >= 18) {
						    // next 10 bytes are cmucam5 pixy largest object detected
						    SensorPixySignature = packetData[8]*256+packetData[9];
						    SensorPixyX = packetData[10]*256+packetData[11];
						    SensorPixyY = packetData[12]*256+packetData[13];
						    SensorPixyWidth = packetData[14]*256+packetData[15];
						    SensorPixyHeight = packetData[16]*256+packetData[17];
					    } else {
						    SensorPixySignature = -1;
						    SensorPixyX = -1;
						    SensorPixyY = -1;
						    SensorPixyWidth = -1;
						    SensorPixyHeight = -1;
					    }
				    } else {
					    console.log("RCV:Expected 8 byte payload, got "+packetLength);
				    }
				    InputState = ST_START;	// ready for next packet
			    }
			    break;

		    default:
			    console.log("RCV:ERROR:Unknown input state: "+InputState);
			    InputState = ST_START;
			    break;
		    }
	    }

	    //console.log("RCV: Ran out of bytes to process");

    /*******************
        if (watchdog && (bytes[0] == "V".charCodeAt())) {
            // Seems to be a valid Gamepad.
            clearTimeout(watchdog);
            watchdog = null;
        console.log("V Received!");
        } else {
        console.log("Hmmm. Not V but rather " + bytes[0]);
    }
    **********************/
    }


    ext._deviceRemoved = function(dev) {
	console.log("Device removed");
        if(device != dev) return;
        if(poller) poller = clearInterval(poller);
        device = null;
    };

    ext._shutdown = function() {
        if(device) device.close();
        if(poller) poller = clearInterval(poller);
        device = null;
        console.log("shutdown");
    };

    ext._stop = function() {
        console.log("stop command received");
	curCmd = null;
	curCmdRec = null;
	//curRecCmd = new Uint8Array(5);
	//curRecCmd[0] = "R".charCodeAt();
	//curRecCmd[1] = "1".charCodeAt();
	//curRecCmd[2] = curRecCmd[3] = curRecCmd[4] = "S".charCodeAt();
    };

    ext._getStatus = function() {
        if(!device) return {status: 1, msg: "Vorpal Gamepad disconnected"};
        return {status: 2, msg: "Vorpal Gamepad connected"};
	//add this back later:
        //if(watchdog) return {status: 1, msg: "Vorpal Searching for Gamepad"};
    };

    ext.walk = function(ingait, indir, wtime, callback) {

        var cmd = new Uint8Array(3);

        cmd[0] = "W".charCodeAt();    

        // gait: ["normal", "high knees", "small steps", "scamper"],
        switch (ingait) {
            default:
            case "normal":
                cmd[1] = "1".charCodeAt();
                break;

            case "high knees":
                cmd[1] = "2".charCodeAt();
                break;

            case "small steps":
                cmd[1] = "3".charCodeAt();
                break;

            case "scamper":
                cmd[1] = "4".charCodeAt();
                break;
        }

        switch (indir) {
            default:
            case "forward":
                cmd[2] = "f".charCodeAt();
                break;

	    case "backward":
                cmd[2] = "b".charCodeAt();
                break;

            case "turn left":
                cmd[2] = "l".charCodeAt();
                break;

	    case "turn right":
                cmd[2] = "r".charCodeAt();
		break;

            case "stomp/honk":
                cmd[2] = "w".charCodeAt();
		break;

            case "stop":
                cmd[2] = "s".charCodeAt();
                break;

        }

	window.setTimeout(function() {
            callback();
        }, wtime*1000);

        //curCmd = new Uint8Array(cmd.buffer);
	setSimpleCommand(cmd.buffer);

        console.log("sent walk " + ingait + " " + indir );
    };

    ext.start = function() {
        console.log("start");
    };

    ext.standstill = function(style, wtime, callback) {
        console.log("stand");

        var cmd = new Uint8Array(3);

        // standstyle: ["normal", "tiptoes", "floor", "foldup"],
        switch (style) {
            default:
            case "normal":
		cmd[0] = "W".charCodeAt();
                cmd[1] = "1".charCodeAt();
                cmd[2] = "s".charCodeAt();
                break;
            case "tiptoes":
		cmd[0] = "D".charCodeAt();
                cmd[1] = "2".charCodeAt();
                cmd[2] = "s".charCodeAt();
		break;
            case "floor":
		cmd[0] = "S".charCodeAt();
                cmd[1] = "1".charCodeAt();
                cmd[2] = "s".charCodeAt();
		break;
	    case "foldup":
		cmd[0] = "S".charCodeAt();
                cmd[1] = "2".charCodeAt();
                cmd[2] = "s".charCodeAt();
		break;
	}

	setSimpleCommand(cmd.buffer);
        console.log("sent stand still " + style);
    };

    ext.dance = function(dancemove, wtime, callback) {
        console.log("dance");

        var cmd = new Uint8Array(3);

        cmd[0] = "D".charCodeAt();

        //dancemove: ["twist", "twist on floor", "twist legs up", "twist other legs up", "dab",
        switch (dancemove) {
            default:
            case "twist":
                cmd[1] = "1".charCodeAt();
                cmd[2] = "f".charCodeAt();
                break;
	    case "twist on floor":
                cmd[1] = "1".charCodeAt();
                cmd[2] = "b".charCodeAt();
                break;
            case "twist legs up":
                cmd[1] = "1".charCodeAt();
                cmd[2] = "l".charCodeAt();
                break;
                case "twist other legs up":
                cmd[1] = "1".charCodeAt();
                cmd[2] = "r".charCodeAt();
                break;
            case "dab":
                cmd[1] = "1".charCodeAt();
                cmd[2] = "w".charCodeAt();
                break;

        // "ballet flutter", "ballet left", "ballet right", "ballet forward", "ballet backward",

            case "ballet flutter":
                cmd[1] = "2".charCodeAt();
                cmd[2] = "w".charCodeAt();
                break;
            case "ballet left":
                cmd[1] = "2".charCodeAt();
                cmd[2] = "l".charCodeAt();
                break;
            case "ballet right":
                cmd[1] = "2".charCodeAt();
                cmd[2] = "r".charCodeAt();
                break;
            case "ballet forward":
                cmd[1] = "2".charCodeAt();
                cmd[2] = "f".charCodeAt();
                break;
            case "ballet backward":
                cmd[1] = "2".charCodeAt();
                cmd[2] = "b".charCodeAt();
                break;

        // "wave teeter", "wave totter", "wave ripple", "wave swirl left", "wave swirl right"],
            case "wave teeter":
                cmd[1] = "3".charCodeAt();
                cmd[2] = "r".charCodeAt();
                break;
            case "wave totter":
                cmd[1] = "3".charCodeAt();
                cmd[2] = "l".charCodeAt();
                break;
            case "wave ripple":
                cmd[1] = "3".charCodeAt();
                cmd[2] = "w".charCodeAt();
                break;
            case "wave swirl left":
                cmd[1] = "3".charCodeAt();
                cmd[2] = "f".charCodeAt();
                break;
            case "wave swirl right":
                cmd[1] = "3".charCodeAt();
                cmd[2] = "r".charCodeAt();
                break;
	    case "stop":
                cmd[1] = "3".charCodeAt();
                cmd[2] = "s".charCodeAt();
                break;

        }

	window.setTimeout(function() {
            callback();
        }, wtime*1000);

        //curCmd = new Uint8Array(cmd.buffer);
	setSimpleCommand(cmd.buffer);
        console.log("sent dance " + dancemove);

    };


    ext.fightadjust = function(adjuststyle,wtime,callback) {
        console.log("fightadjust");

        var cmd = new Uint8Array(3);

        cmd[0] = "F".charCodeAt();

    //fightadjust: ["square up", "thrust forward", "thrust backward", "lean left", "lean right", "lean forward", "lean back upward", "twist hips right", "twist hips left", "freeze in place"],
        switch (adjuststyle) {
        default:
        case "square up":
            cmd[1] = "3".charCodeAt();
            cmd[2] = "w".charCodeAt();
            break;
        case "thrust forward":
            cmd[1] = "3".charCodeAt();
            cmd[2] = "f".charCodeAt();
            break;
        case "thrust backward":
            cmd[1] = "3".charCodeAt();
            cmd[2] = "b".charCodeAt();
            break;
        case "lean left":
            cmd[1] = "4".charCodeAt();
            cmd[2] = "l".charCodeAt();
            break;
        case "lean right":
            cmd[1] = "4".charCodeAt();
            cmd[2] = "r".charCodeAt();
            break;
        case "lean forward":
            cmd[1] = "4".charCodeAt();
            cmd[2] = "f".charCodeAt();
            break;
        case "lean backward":
            cmd[1] = "4".charCodeAt();
            cmd[2] = "b".charCodeAt();
            break;
        case "twist hips right":
            cmd[1] = "3".charCodeAt();
            cmd[2] = "r".charCodeAt();
            break;
        case "twist hips left":
            cmd[1] = "3".charCodeAt();
            cmd[2] = "l".charCodeAt();
            break;
        case "freeze in place":
            cmd[1] = "4".charCodeAt();
            cmd[2] = "s".charCodeAt();
            break;
        }

        //curCmd = new Uint8Array(cmd.buffer);
	setSimpleCommand(cmd.buffer);
	window.setTimeout(function() {
            callback();
        }, wtime*1000);
        console.log("sent fight adjust " + adjuststyle );
    };

    ext.fightarms = function(fightstyle,fightmove,wtime,callback) {
        console.log("fightarms");

        var cmd = new Uint8Array(3);

        cmd[0] = "F".charCodeAt();

        // armfightstyle: ["single arms", "unison arms"],
        switch (fightstyle) {
        default:
        case "single arms":
            cmd[1] = "1".charCodeAt();
            break;
        case "unison arms":
            cmd[1] = "2".charCodeAt();
            break;
        }

        // armfightmove: ["defend", "lefthook", "righthook", "uppercut", "downsweep", "auto ninja"],
        switch (fightmove) {
        case "righthook":
            cmd[2] = "r".charCodeAt();
            break;
        case "lefthook":
            cmd[2] = "l".charCodeAt();
            break;
        case "uppercut":
            cmd[2] = "f".charCodeAt();
            break;
        case "downsweep":
            cmd[2] = "d".charCodeAt();
            break;
        case "auto ninja":
            cmd[2] = "w".charCodeAt();
            break;

          default:
        case "defend":
            cmd[2] = "s".charCodeAt();
            break;
        }

	window.setTimeout(function() {
            callback();
        }, wtime*1000);

        //curCmd = new Uint8Array(cmd.buffer);
	setSimpleCommand(cmd.buffer);
        console.log("sent fight " + fightstyle + " " + fightmove);
    };


    ext.beep = function(freq,duration,callback) {
        console.log("beep");

        var cmd = new Uint8Array(5);

        cmd[0] = "B".charCodeAt();    // code for BEEP command, high bit on

        if (freq < 50) {
            freq = 50;
        } else if (freq > 2000) {
            freq = 2000;
        }

	freq = freq + 0;  // convert to integer
        cmd[1] = freq / 256;
        cmd[2] = freq % 256;

        if (duration > 30) {
            duration = 30;    // maximum time for tone in seconds
        }
        // conver to milliseconds
        duration = (duration + 0);  // convert to integer
        duration *= 1000;

        // output high byte first, then low
        cmd[3] = duration / 256;
        cmd[4] = duration % 256;

	setSimpleCommand(cmd.buffer);

        console.log("sent beep " + freq + " " + duration);
	window.setTimeout(function() {
	    callback();
	}, duration);
    };

    ext.recordend = function(callback) {
        console.log("recordend");

        var cmd = new Uint8Array(5);

        cmd[0] = "R".charCodeAt();
	cmd[1] = "1".charCodeAt();
	cmd[2] = "S".charCodeAt();
	cmd[3] = "S".charCodeAt();
	cmd[4] = "S".charCodeAt();
        console.log("sending RECSTOP");
        curCmdRec = new Uint8Array(cmd.buffer);
    };


    ext.recordstart = function(matrix,dpad,callback) {
        console.log("recordstart");

        var cmd = new Uint8Array(5);

        cmd[0] = "R".charCodeAt();
	cmd[1] = "1".charCodeAt();

	switch (matrix) {
	default:
	case "Walk 1":
		cmd[2] = "W".charCodeAt();
		cmd[3] = "1".charCodeAt();
		break;
	case "Walk 2":
		cmd[2] = "W".charCodeAt();
		cmd[3] = "2".charCodeAt();
		break;
	case "Walk 3":
		cmd[2] = "W".charCodeAt();
		cmd[3] = "3".charCodeAt();
		break;
	case "Walk 4":
		cmd[2] = "W".charCodeAt();
		cmd[3] = "4".charCodeAt();
		break;
	case "Dance 1":
		cmd[2] = "D".charCodeAt();
		cmd[3] = "1".charCodeAt();
		break;
	case "Dance 2":
		cmd[2] = "D".charCodeAt();
		cmd[3] = "2".charCodeAt();
		break;
	case "Dance 3":
		cmd[2] = "D".charCodeAt();
		cmd[3] = "3".charCodeAt();
		break;
	case "Dance 4":
		cmd[2] = "D".charCodeAt();
		cmd[3] = "4".charCodeAt();
		break;
	}

	switch (dpad) {
	default:
	case "nothing pressed":
		cmd[4] = "s".charCodeAt();
		break;
	case "forward":
		cmd[4] = "f".charCodeAt();
		break;
	case "backward":
		cmd[4] = "b".charCodeAt();
		break;
	case "left":
		cmd[4] = "l".charCodeAt();
		break;
	case "right":
		cmd[4] = "r".charCodeAt();
		break;
	case "special":
		cmd[4] = "w".charCodeAt();
		break;
	}
        console.log("sending REC");
        curCmdRec = new Uint8Array(cmd.buffer);
    };

    ext.waitforconnection = function(callback) {
        console.log("RECONNECT");
	if (device) {
		console.log("Stopping and restarting device:" + device.id);
		device.close();

		// when the device actually completes opening we'll trigger
		// the Wait block callback
		deviceOpenedNotify = callback;
		device.open({ stopBits: 0, bitRate: 9600, ctsFlowControl: 0 }, deviceOpenedCallback);
	} else {
		tryNextDevice();
	}

    };

    ext.sethips = function(legs,hippos,opts,wtime,callback) {
        console.log("setleg");

        var cmd = new Uint8Array(5);

        cmd[0] = "L".charCodeAt();    // code for SETLEG command, high bit on

        console.log("1");

        // cmd[1] specifies a bitmask of legs
        switch (legs) {
            case "all":
                cmd[1] = Number(0b10111111);    // bitmask, LSB is leg 0, etc.
                break;
            case "left":
                cmd[1] = Number(0b10000111);    // legs 0, 1, 2
                break;
            case "right":
                cmd[1] = Number(0b10111000);    // legs 3, 4, 5
                break;
            case "front":
                cmd[1] = Number(0b10100001);    // legs 0, 5
                break;
            case "middle":
                cmd[1] = Number(0b10010010);    // legs 1, 4
                break;
            case "back":
                cmd[1] = Number(0b10001100);    // legs 2, 3
                break;
            case "tripod1":
                cmd[1] = Number(0b10010101);    // legs 0, 2, 4
                break;
            case "tripod2":
                cmd[1] = Number(0b10101010);    // legs 1, 3, 5
                break;
            case "0":
                cmd[1] = Number(0b10000001);    // legs 0
                break;
            case "1":
                cmd[1] = Number(0b10000010);    // legs 1
                break;
            case "2":
                cmd[1] = Number(0b10000100);    // legs 2
                break;
            case "3":
                cmd[1] = Number(0b10001000);    // legs 3
                break;
            case "4":
                cmd[1] = Number(0b10010000);    // legs 4
                break;
            case "5":
                cmd[1] = Number(0b10100000);    // legs 5
                break;
        }

        console.log("2");

        if (hippos > 180) {
            hippos = 180;
        } else if (hippos < 0) {
            hippos = 1;
        }

        cmd[2] = hippos;
        cmd[3] = 255;	// 255 is a special value, here means don't change knee positions

        //legopts: ["mirror hips", "raw hips"]

        if (opts == "mirror hips") {
            cmd[4] = 0;
        } else {
            cmd[4] = 1;
        }

	console.log("3");

	window.setTimeout(function() {
            callback();
        }, wtime*1000);

	setSimpleCommand(cmd.buffer);

        console.log("sethips " + legs + " " + hippos + " " + opts);

    };

    ext.setknees = function(legs,kneepos,wtime,callback) {
        console.log("setknees");

        var cmd = new Uint8Array(5);

        cmd[0] = "L".charCodeAt();    // code for SETLEG command, high bit on

        console.log("1");

        // cmd[1] specifies a bitmask of legs
        switch (legs) {
            case "all":
                cmd[1] = Number(0b10111111);    // bitmask, LSB is leg 0, etc.
                break;
            case "left":
                cmd[1] = Number(0b10000111);    // legs 0, 1, 2
                break;
            case "right":
                cmd[1] = Number(0b10111000);    // legs 3, 4, 5
                break;
            case "front":
                cmd[1] = Number(0b10100001);    // legs 0, 5
                break;
            case "middle":
                cmd[1] = Number(0b10010010);    // legs 1, 4
                break;
            case "back":
                cmd[1] = Number(0b10001100);    // legs 2, 3
                break;
            case "tripod1":
                cmd[1] = Number(0b10010101);    // legs 0, 2, 4
                break;
            case "tripod2":
                cmd[1] = Number(0b10101010);    // legs 1, 3, 5
                break;
            case "0":
                cmd[1] = Number(0b10000001);    // legs 0
                break;
            case "1":
                cmd[1] = Number(0b10000010);    // legs 1
                break;
            case "2":
                cmd[1] = Number(0b10000100);    // legs 2
                break;
            case "3":
                cmd[1] = Number(0b10001000);    // legs 3
                break;
            case "4":
                cmd[1] = Number(0b10010000);    // legs 4
                break;
            case "5":
                cmd[1] = Number(0b10100000);    // legs 5
                break;
        }

        console.log("2");

        if (kneepos > 180) {
            kneepos = 180;
        } else if (kneepos < 1) {
            kneepos = 1;
        }

        cmd[2] = 255;	// 255 is a special value, here means don't change hip positions;
	cmd[3] = kneepos;

	console.log("3");

	window.setTimeout(function() {
            callback();
        }, wtime*1000);

	setSimpleCommand(cmd.buffer);

        console.log("setknees " + legs + " " + kneepos);

    };

    ext.setleg = function(legs,hippos,kneepos,opts,wtime,callback) {
        console.log("setleg");

        var cmd = new Uint8Array(5);

        cmd[0] = "L".charCodeAt();    // code for SETLEG command, high bit on

        console.log("1");

        //legs: ["all", "left", "right", "front", "middle", "back", "tripod1", "tripod2", "0", "1", "2", "3", "4", "5"],
        // cmd[1] specifies a bitmask of legs
        switch (legs) {
            case "all":
                cmd[1] = Number(0b10111111);    // bitmask, LSB is leg 0, etc.
                break;
            case "left":
                cmd[1] = Number(0b10000111);    // legs 0, 1, 2
                break;
            case "right":
                cmd[1] = Number(0b10111000);    // legs 3, 4, 5
                break;
            case "front":
                cmd[1] = Number(0b10100001);    // legs 0, 5
                break;
            case "middle":
                cmd[1] = Number(0b10010010);    // legs 1, 4
                break;
            case "back":
                cmd[1] = Number(0b10001100);    // legs 2, 3
                break;
            case "tripod1":
                cmd[1] = Number(0b10010101);    // legs 0, 2, 4
                break;
            case "tripod2":
                cmd[1] = Number(0b10101010);    // legs 1, 3, 5
                break;
            case "0":
                cmd[1] = Number(0b10000001);    // legs 0
                break;
            case "1":
                cmd[1] = Number(0b10000010);    // legs 1
                break;
            case "2":
                cmd[1] = Number(0b10000100);    // legs 2
                break;
            case "3":
                cmd[1] = Number(0b10001000);    // legs 3
                break;
            case "4":
                cmd[1] = Number(0b10010000);    // legs 4
                break;
            case "5":
                cmd[1] = Number(0b10100000);    // legs 5
                break;
        }

        console.log("2");

        if (hippos > 180) {
            hippos = 180;
        } else if (hippos < 0) {
            hippos = 1;
        }
        if (kneepos > 180) {
            kneepos = 180;
        } else if (kneepos < 0) {
            kneepos = 0;
        }

        cmd[2] = hippos;
        cmd[3] = kneepos;

        //legopts: ["mirror hips", "raw hips"]
        //

        if (opts == "mirror hips") {
            cmd[4] = 0;
        } else {
            cmd[4] = 1;
        }

	console.log("3");

	window.setTimeout(function() {
            callback();
        }, wtime*1000);

        //curCmd = new Uint8Array(cmd.buffer);
	setSimpleCommand(cmd.buffer);

        console.log("setleg " + legs + " " + hippos + " " + kneepos + " " + opts);

    };

    ext.readsensor = function(sensor) {
	    console.log("readsensor");
	    switch (sensor) {
	    default:
	    case "Analog 3":
		    return SensorA3;

	    case "Analog 6":
		    return SensorA6;

	    case "Analog 7": 
		    return SensorA7;

	    case "Ultrasonic distance": 
		    return SensorDistance;
	    }
    };

    var descriptor = {
        blocks: [
            ["w", "Walk %m.gait %m.direction %n seconds", "walk", "normal", "forward", 1],
            ["w", "Dance %m.dancemove %n seconds", "dance", "twist", 1],
            ["w", "Fight with arms %m.armfightstyle %m.armfightmove %n seconds", "fightarms", "single arms", "defend", 0.2],
            ["w", "fight adjust %m.fightadjust %n seconds", "fightadjust", "square up", 0.2],
            ["w", "Set Legs: %m.legs hips: %n knees: %n options: %m.legopts %n seconds", "setleg", "all", "90", "90", "mirror hips", 0.2],
            ["w", "Set Hips: %m.legs %n options: %m.legopts %n seconds", "sethips", "all", "90", "mirror hips", 0.2],
            ["w", "Set Knees: %m.legs %n %n seconds", "setknees", "all", "90", 0.2],
            ["w", "Stand Still: %m.standstyle %n seconds", "standstill", "normal", 1],
            ["w", "Beep frequency: %n seconds: %n", "beep", "300", "0.3"],
            ["r", "Sensor: %m.sensors", "readsensor", "Analog 3"],
            ["r", "CMUcam5: %m.cmucam5vals", "readcmucam5", "x"],
            [" ", "Record Start: %m.matrix %m.dpad", "recordstart", "Walk 1", "forward"],
            [" ", "Record End", "recordend"],
            ["w", "Wait for Connection", "waitforconnection"],

        ],

    menus: {
        direction: ["forward", "backward", "turn left", "turn right", "stomp/honk", "stop"],

        yesno: ["yes", "no"],

        gait: ["normal", "high knees", "small steps", "scamper"],

        standstyle: ["normal", "tiptoes", "floor", "foldup"],

        dancemove: ["twist", "twist on floor", "twist legs up", "twist other legs up", "dab", "ballet flutter", "ballet left", "ballet right", "ballet forward", "ballet backward", "wave teeter", "wave totter", "wave ripple", "wave swirl left", "wave swirl right", "stop"],

        armfightstyle: ["single arms", "unison arms"],

        armfightmove: ["defend", "lefthook", "righthook", "uppercut", "downsweep", "auto ninja"],

        fightadjust: ["square up", "thrust forward", "thrust backward", "lean left", "lean right", "lean forward", "lean back upward", "twist hips right", "twist hips left", "freeze in place"],

        legs: ["all", "left", "right", "front", "middle", "back", "tripod1", "tripod2", "0", "1", "2", "3", "4", "5"],

        legopts: ["mirror hips", "raw hips"],

        sensors: ["Ultrasonic distance", "Analog 3", "Analog 6", "Analog 7"],

        cmucam5vals: ["x", "y", "width", "height", "angle", "object id"],

        matrix: ["Walk 1", "Walk 2", "Walk 3", "Walk 4", "Dance 1", "Dance 2", "Dance 3", "Dance 4", "Fight 1", "Fight 2", "Fight 3", "Fight 4"],

        dpad: ["nothing pressed", "forward", "backward", "left", "right", "special"],
      },
      url: "http://www.vorpalrobotics.com/wiki/index.php?title=Vorpal_Combat_Hexapod_Scratch_Programming"
    };

    ScratchExtensions.register("VorpalCombatHexapod", descriptor, ext, {type: "serial"});

})({});
