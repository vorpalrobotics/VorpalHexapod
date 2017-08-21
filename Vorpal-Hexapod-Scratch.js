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

    ext.resetAll = function(){};


    function appendBuffer( buffer1, buffer2 ) {
        var tmp = new Uint8Array( buffer1.byteLength + buffer2.byteLength );
        tmp.set( new Uint8Array( buffer1 ), 0 );
        tmp.set( new Uint8Array( buffer2 ), buffer1.byteLength );
        return tmp.buffer;
    }

    // Extension API interactions
    var potentialDevices = [];
    ext._deviceConnected = function(dev) {
	console.log("Device connected " + dev.id);
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
	var c = new Uint8Array(4+cb.length);

	c[0] = "V".charCodeAt();
	c[1] = "1".charCodeAt();
	c[2] = cb.length;
	var checksum = cb.length;
	for (var i = 0; i < cb.length; i++) {
		c[3+i] = cb[i];
		checksum += cb[i];
	}
	checksum = checksum%256;
	c[cb.length+3] = checksum;
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


    function deviceOpenedCallback(dev) {
        if (!dev) {
            device = null;
            tryNextDevice();
            return;
        }
        device.set_receive_handler(function(data) {
            var bytes = new Uint8Array(data);
            console.log("RCV:" + bin2String(bytes));
            //var inputData = new Uint8Array(data);
            //processData(inputData);
        });

    console.log("set receive data handler");
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
		    // should really set it to null here
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

    	console.log("Trying next device " + device.id);


    }

    ext.startserial = function() {
        console.log("startserial");
    };

    function processData(data) {
        // var bytes = new Uint8Array(data);

    console.log("processing data");

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

    //fightadjust: ["square up", "thrust forward", "thrust backward", "lean left", "lean right", "lean forward", "lean back upward", "twist hips right", "twist hips left"],
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
            cmd[1] = "3".charCodeAt();
            cmd[2] = "l".charCodeAt();
            break;
        case "lean right":
            cmd[1] = "3".charCodeAt();
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
            cmd[1] = "4".charCodeAt();
            cmd[2] = "r".charCodeAt();
            break;
        case "twist hips left":
            cmd[1] = "4".charCodeAt();
            cmd[2] = "l".charCodeAt();
            break;
        }

	window.setTimeout(function() {
            callback();
        }, wtime*1000);
        //curCmd = new Uint8Array(cmd.buffer);
	setSimpleCommand(cmd.buffer);
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


    ext.beep = function(freq,duration,wtime,callback) {
        console.log("beep");

        var cmd = new Uint8Array(5);

        cmd[0] = Number(0b10000010);    // code for BEEP command, high bit on

        console.log("1");        window.setTimeout(function() {
            callback();
        }, wtime*1000);

        if (freq < 50) {
            freq = 50;
        } else if (freq > 2000) {
            freq = 2000;
        }

	freq = freq + 0;  // convert to integer
        cmd[1] = freq / 256;
        cmd[2] = freq % 256;

        if (duration > 30) {
            duration = 30;    // maximum time for tone
        }
        // conver to milliseconds
        duration *= 1000;
        duration = (duration + 0);  // convert to integer

        // output high byte first, then low
        cmd[3] = duration / 256;
        cmd[4] = duration % 256;

        //curCmd = new Uint8Array(cmd.buffer);
	setSimpleCommand(cmd.buffer);
        console.log("sent beep " + freq + " " + duration);
	window.setTimeout(function() {
            callback();
        }, wtime*1000);


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

    var descriptor = {
        blocks: [
            ["w", "Walk %m.gait %m.direction %n seconds", "walk", "normal", "forward", 1],
            ["w", "Dance %m.dancemove %n seconds", "dance", "twist", 1],
            ["w", "Fight with arms %m.armfightstyle %m.armfightmove %n seconds", "fightarms", "single arms", "defend", 0.2],
            ["w", "fight adjust %m.fightadjust %n seconds", "fightadjust", "square up", 0.2],
            ["w", "Set Legs: %m.legs hips: %n knees: %n options: %m.legopts %n seconds", "setleg", "all", "90", "90", "mirror hips", 0.2],
            //["w", "Set Hips: %m.legs hips: %n options: %m.legopts", "sethip", "all", "90", "mirror hips",0.2],
            //["w", "Set Knees: %m.legs knees: %n", "setknees", "all", "90", 0.2],
            //["w", "Stand Still: %m.standstyle %n seconds", "standstill", "normal", 1],
            //["w", "Beep frequency: %n seconds: %n ", "beep", "300", "0.3"],
            ["r", "ReadSensor: %m.sensors", "readsensor", "Generic Analog (A6)"],
            [" ", "Record Start: %m.matrix %m.dpad", "recordstart", "Walk 1", "forward"],
            [" ", "Record End", "recordend"],

        ],

    menus: {
        direction: ["forward", "backward", "turn left", "turn right", "stop"],

        gait: ["normal", "high knees", "small steps", "scamper"],

        standstyle: ["normal", "tiptoes", "floor", "foldup"],

        dancemove: ["twist", "twist on floor", "twist legs up", "twist other legs up", "dab", "ballet flutter", "ballet left", "ballet right", "ballet forward", "ballet backward", "wave teeter", "wave totter", "wave ripple", "wave swirl left", "wave swirl right"],

        armfightstyle: ["single arms", "unison arms"],

        armfightmove: ["defend", "lefthook", "righthook", "uppercut", "downsweep", "auto ninja"],

        fightadjust: ["square up", "thrust forward", "thrust backward", "lean left", "lean right", "lean forward", "lean back upward", "twist hips right", "twist hips left"],

        legs: ["all", "left", "right", "front", "middle", "back", "tripod1", "tripod2", "0", "1", "2", "3", "4", "5"],

        legopts: ["mirror hips", "raw hips"],

        sensors: ["Ultrasonic Rangefinder (D6/D7)", "Light Sensor (A3)", "Generic Analog (A6)", "CmuCam5 Pixie (SPI)"],

        matrix: ["Walk 1", "Walk 2", "Walk 3", "Walk 4", "Dance 1", "Dance 2", "Dance 3", "Dance 4", "Fight 1", "Fight 2", "Fight 3", "Fight 4"],

        dpad: ["nothing pressed", "forward", "backward", "left", "right", "special"],
      },
      url: "http://www.vorpalrobotics.com/wiki/index.php?title=Vorpal_Combat_Hexapod_Scratch_Programming"
    };

    ScratchExtensions.register("VorpalCombatHexapod", descriptor, ext, {type: "serial"});

})({});
