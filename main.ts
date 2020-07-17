/*
 startbit package
*/
//% weight=10 icon="\ue312" color=#2896ff block="Kulbot_demo1"
namespace Kulbot_demo1 {

    export enum startbit_Colors {
        //% block="Red"
        Red = 0x01,
        //% block="Green"
        Green = 0x02,
        //% block="Blue"
        Blue = 0x03,
        //% block="Black"
        Black = 0x04,
        //% block="White"
        White = 0x05,
        //% block="None"
        None = 0x06
    }

    export enum startbit_lineFollower {
        //% blockId="S1_OUT_S2_OUT" block="Sensor1 and sensor2 are out black line"
        S1_OUT_S2_OUT = 0x00,
        //% blockId="S1_OUT_S2_IN" block="Sensor2 in black line but sensor1 not"
        S1_OUT_S2_IN = 0x01,
        //% blockId="S1_IN_S2_OUT" block="Sensor1 in black line but sensor2 not"
        S1_IN_S2_OUT = 0x02,
        //% blockId="S1_IN_S2_IN" block="Sensor1 and sensor2 are in black line "
        S1_IN_S2_IN = 0x03
    }

    export enum startbit_colorSensorPort {
        //% block="Port 4"
        port4 = 0x04
    }
    export enum startbit_LCDPort {
        //% block="Port 4"
        port4 = 0x04
    }
    export enum startbit_Servos {
        //% block="servo 1"
        Servo1 = 0x01,
        //% block="servo 2"
        Servo2 = 0x02
    }

    export enum startbit_ultrasonicPort {
        //% block="Port 1"
        port1 = 0x01,
        //% block="Port 2"
        port2 = 0x02
    }

    export enum startbit_touchKeyPort {
        //% block="Port 1"
        port1 = 0x01,
        //% block="Port 2"
        port2 = 0x02,
        //% block="Port 3"
        port3 = 0x03
    }

    export enum startbit_lineFollowPort {
        //% block="Port 1"
        port1 = 0x01
    }


    export enum startbit_PinIOStatus {
        //% block="Low"
        Low = 0x00,
        //% block="High"
        Hight = 0x01
    }

    export enum startbit_LineFollowerSensor {
        //% block="Sensor 1"
        LFSensor_1 = 0x00,
        //% block="Sensor 2"
        LFSensor_2 = 0x01
    }

    export enum startbit_busServoPort {
        //% block="Port 6"
        port6 = 0x06
    }

    export enum startbit_knobPort {
        //% block="Port 1"
        port1 = 0x01
    }
	
    export enum startbit_photosensitivePort {
        //% block="Port 1"
        port1 = 0x01	    
    }
	
    export enum startbit_PhotosensitiveSensor {
        //% block="Port 1"
        port1 = 0x00	    
    }

    export enum startbit_fanPort {
        //% block="Port 1"
        port1,
        //% block="Port 2"
        port2
    }

    export enum startbit_servorange {
        //% block="180"
        range1 = 180,
        //% block="270"
        range2 =270
    }
	
    export enum startbit_digitaltubePort {
        //% block="Port 1"
        port1 = 0x01,
        //% block="Port 2"
        port2 = 0x02
    }
	
    export enum startbit_CmdType {
        //% block="Invalid command"
        NO_COMMAND = 0,
        //% block="car run"
        CAR_RUN = 1,
	    //% block="robot run"   
	    ROBOT_RUN = 1,
        //% block="Servo"
        SERVO = 2,
        //% block="Ultrasonic distance"
        ULTRASONIC = 3,
        //% block="Temperature"
        TEMPERATURE = 4,
        //% block="Sound"
        SOUND = 5,
        //% block="Light"
        LIGHT = 6,
        //% block="Voltage"
        BAT = 7,
        //% block="Rgb light"
        RGB_LIGHT = 8,
        //% block="Honk horn"
        DIDI = 9,
        //% block="Read firmware version"
        VERSION = 10,
        //% block="Read angle"
        READ_ANGLE = 11,
        //% block="Light belt"        
        RGB_BELT = 12,
        //% block="WIFI mode"
        WIFI_MODE = 13,
        //% block="Get mac"
        GET_MAC = 14,
        //% block="Get hand cmd"
        GET_HAND_CMD = 15
     }

    export enum startbit_CarRunCmdType {
        //% block="Stop"
        STOP = 0,
        //% block="Go ahead"
        GO_AHEAD,
        //% block="Back"
        GO_BACK,
        //% block="Turn left"
        TURN_LEFT,
        //% block="Turn right"
        TURN_RIGHT,
        //% block="Go ahead slowly"
        GO_AHEAD_SLOW,
        //% block="Turn left slowly"
        TURN_LEFT_SLOW,
        //% block="Turn right slowly"
        TURN_RIGHT_SLOW,
        //% block="Invalid command"
        COMMAND_ERRO
    }

    export enum I2C_ADDR {
        //% block="0x27"
        addr1 = 0x27,
        //% block="0x3f"
        addr2 = 0x3f,
        //% block="0x20"
        addr3 = 0x20,
        //% block="0x62"
        addr4 = 0x62,
        //% block="0x3e"
        addr5 = 0x3e
    }
    export enum on_off {
        //% block="on"
        on = 1,
        //% block="off"
        off = 0
    }

    export enum visibled {
        //% block="visibled"
        visible = 1,
        //% block="invisibled"
        invisible = 0
    }


    /**
     * Startbit initialization, please execute at boot time
    */
    //% weight=100 blockId=startbit_Init block="Initialize Startbit" group="0. Set up."
    export function startbit_Init() {
        startbit_initRGBLight();
        
        basic.pause(2000);
    }


    let handleCmd: string = "";
    let currentVoltage: number = 0;
    let volume: number = 0;
    let lhRGBLight: StartbitRGBLight.LHstartbitRGBLight;
    let lhRGBLightBelt: StartbitRGBLight.LHstartbitRGBLight;

    let P14_ad = 0;
    let MESSAGE_MAC = 0xff;
    let MESSAGE_ANGLE = 0x100;

    let i2cPortValid: boolean = true;
    let connectStatus: boolean = false;

    let servo1Angle: number = 0xfff;
    let servo2Angle: number = 0xfff;

    let macStr: string = "";
    let actiongroup_finished = true;
    
    let Digitaltube:startbit_TM1640LEDs
    let TM1640_CMD1 = 0x40;
    let TM1640_CMD2 = 0xC0;
    let TM1640_CMD3 = 0x80;
    let _SEGMENTS = [0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71];
	
    



    

    /**
    * Set the angle of servo 1 to 6, range of 0~270 degree
    */
    //% weight=99 blockId=setServo block="Set pwm servo range|range %range|index %index|angle %angle|duration %duration" group="2. Servo-Motor."
    //% angle.min=0 angle.max=270
    //% inlineInputMode=inline
    export function setServo(range:startbit_servorange, index: number = 1, angle: number, duration: number = 300) {
	    
        let position = mapRGB(angle, 0, range, 500, 2500);

        let buf = pins.createBuffer(10);
        buf[0] = 0x55;
        buf[1] = 0x55;
        buf[2] = 0x08;
        buf[3] = 0x03;//cmd type
        buf[4] = 0x01;
        buf[5] = duration & 0xff;
        buf[6] = (duration >> 8) & 0xff;
        buf[7] = index;
        buf[8] = position & 0xff;
        buf[9] = (position >> 8) & 0xff;
        serial.writeBuffer(buf);
    }

	
    /**
    * Set the servo controller to run a actiongroup
    * @param times Running times. eg: 1
    */
    //% weight=96 blockId=startbit_runActionGroup block="Run ActionGroup|index %index|times %times" group="2. Servo-Motor." advanced=true
    export function startbit_runActionGroup(index: number, times: number = 1) {

        let buf = pins.createBuffer(7);
        buf[0] = 0x55;
        buf[1] = 0x55;
        buf[2] = 0x05;
        buf[3] = 0x06;//cmd type CMD_ACTION_GROUP_RUN
        buf[4] = index & 0xff;
        buf[5] = times & 0xff;
        buf[6] = (times >> 8) & 0xff;

        actiongroup_finished = false;
        serial.writeBuffer(buf);
    }
	
    /**
    * Stop running actiongroup
    */
    //% weight=95 blockId=startbit_stopnActionGroup block="Stop ActionGroup" group="2. Servo-Motor." advanced=true
    export function startbit_stopActionGroup() {

        let buf = pins.createBuffer(7);
        buf[0] = 0x55;
        buf[1] = 0x55;
        buf[2] = 0x02;
        buf[3] = 0x07;//cmd type CMD_ACTION_GROUP_STOP
	    
	actiongroup_finished = false;
        serial.writeBuffer(buf);
    }
	
    /**
     * Wait for Actiongroup Finishing
     */
    //% weight=94 blockId=startbit_actionRunover block="Action run over" group="2. Servo-Motor." advanced=true
    export function startbit_actionRunover(): boolean {
        // let ret = false;
        if (actiongroup_finished == true) {
            // ret = true;
            actiongroup_finished = true;
        }
        else {
            actiongroup_finished = false;
        }
        return actiongroup_finished;
    }

	
    /**
     * Send read startbit servos angle command
     */
    //% weight=92 blockId=startbit_readAngle block="Send |%servo|angle command " group="2. Servo-Motor." advanced=true
    export function startbit_readAngle(servo: startbit_Servos) {
        let buf = pins.createBuffer(6);
        buf[0] = 0x55;
        buf[1] = 0x55;
        buf[2] = 0x04;
        buf[3] = 0x3E;//cmd type
        buf[4] = 0x05;
        buf[5] = servo;
        serial.writeBuffer(buf);
    }


    /**
     * Do someting when Startbit receive angle
     * @param body code to run when event is raised
     */
    //% weight=90 blockId=onStartbit_getAngle block="on Startbit|%servo|get angle" group="2. Servo-Motor." advanced=true
    export function onStartbit_getAngle(servo: startbit_Servos, body: Action) {
        control.onEvent(MESSAGE_ANGLE, servo, body);
    }


    /**
     *  Get servos angle
     */
    //% weight=88 blockId=getServosAngle blockGap=50 block="Get|%servo|angle(-120~120)" group="2. Servo-Motor." advanced=true
    export function getServosAngle(servo: startbit_Servos): number {
        if (servo == startbit_Servos.Servo1) {
            return servo1Angle;
        }
        else if (servo == startbit_Servos.Servo2) {
            return servo2Angle;
        }
        else
            return 0xFFF;
    }
    
    /**
     *  Send robot attitude to the servo controller
     *  @param pitch eg: 0
     *  @param roll eg: 0
     */
    //% weight=91 blockId=startbit_sendAttitude block="Send pitch|%pitch|and roll|%roll"
   /*
    export function startbit_sendAttitude(pitch: number, roll: number) {
        pitch < -90 ? -90 : pitch;
        pitch > 90 ? 90 : pitch;
        roll < -90 ? -90 : roll;
        roll > 90 ? 90 : roll;

        let buf = pins.createBuffer(6);
        buf[0] = 0x55;
        buf[1] = 0x55;
        buf[2] = 0x04;
        buf[3] = 0x5A;
        buf[4] = pitch;
        buf[5] = roll;
        serial.writeBuffer(buf);
    }
    */
	
    /**
    *	Set the speed of the number 1 motor and number 2 motor, range of -100~100, that can control the tank to go advance or turn of.
    */
    //% weight=86 blockId=startbit_setMotorSpeed  block="Set motor1 speed(-100~100)|%speed1|and motor2|speed %speed2" group="2. Servo-Motor."
    //% speed1.min=-100 speed1.max=100
    //% speed2.min=-100 speed2.max=100
    export function startbit_setMotorSpeed(speed1: number, speed2: number) {
        if (speed1 > 100 || speed1 < -100 || speed2 > 100 || speed2 < -100) {
            return;
        }
        speed1 = speed1 * -1;
        speed2 = speed2 * -1;
        let buf = pins.createBuffer(6);
        buf[0] = 0x55;
        buf[1] = 0x55;
        buf[2] = 0x04;
        buf[3] = 0x32;//cmd type
        buf[4] = speed1;
        buf[5] = speed2;
        serial.writeBuffer(buf);
    }

    /**
    *	Set the speed of the fan, range of -100~100. 
    */
    //% weight=84 blockId=startbit_setFanSpeed  block="Set |%port fan speed(-100~100)|%speed1" group="2. Servo-Motor." advanced=true
    //% speed1.min=-100 speed1.max=100
    export function startbit_setFanSpeed(port: startbit_fanPort, speed1: number) {
        if (speed1 > 100 || speed1 < -100) {
            return;
        }
        let pin1 = AnalogPin.P1;
        let pin2 = AnalogPin.P2;

        if (port == startbit_fanPort.port2) {
            pin1 = AnalogPin.P13;
            pin2 = AnalogPin.P14;
        }
        if (speed1 < 0) {
            pins.analogWritePin(pin2, 0);
            pins.analogWritePin(pin1, pins.map(-speed1, 0, 100, 0, 1023));
        }
        else if (speed1 > 0) {
            pins.analogWritePin(pin1, 0);
            pins.analogWritePin(pin2, pins.map(speed1, 0, 100, 0, 1023));
        }
        else {
            pins.analogWritePin(pin2, 0);
            pins.analogWritePin(pin1, 0);
        }

    }

	
   

    const APDS9960_I2C_ADDR = 0x39;
    const APDS9960_ID_1 = 0xA8;
    const APDS9960_ID_2 = 0x9C;
    /* APDS-9960 register addresses */
    const APDS9960_ENABLE = 0x80;
    const APDS9960_ATIME = 0x81;
    const APDS9960_WTIME = 0x83;
    const APDS9960_AILTL = 0x84;
    const APDS9960_AILTH = 0x85;
    const APDS9960_AIHTL = 0x86;
    const APDS9960_AIHTH = 0x87;
    const APDS9960_PERS = 0x8C;
    const APDS9960_CONFIG1 = 0x8D;
    const APDS9960_PPULSE = 0x8E;
    const APDS9960_CONTROL = 0x8F;
    const APDS9960_CONFIG2 = 0x90;
    const APDS9960_ID = 0x92;
    const APDS9960_STATUS = 0x93;
    const APDS9960_CDATAL = 0x94;
    const APDS9960_CDATAH = 0x95;
    const APDS9960_RDATAL = 0x96;
    const APDS9960_RDATAH = 0x97;
    const APDS9960_GDATAL = 0x98;
    const APDS9960_GDATAH = 0x99;
    const APDS9960_BDATAL = 0x9A;
    const APDS9960_BDATAH = 0x9B;
    const APDS9960_POFFSET_UR = 0x9D;
    const APDS9960_POFFSET_DL = 0x9E;
    const APDS9960_CONFIG3 = 0x9F;
    const APDS9960_GCONF4 = 0xAB;
    const APDS9960_AICLEAR = 0xE7;


    /* LED Drive values */
    const LED_DRIVE_100MA = 0;

    /* ALS Gain (AGAIN) values */
    const AGAIN_4X = 1;

    /* Default values */
    const DEFAULT_ATIME = 219;    // 103ms
    const DEFAULT_WTIME = 246;    // 27ms
    const DEFAULT_PROX_PPULSE = 0x87;    // 16us, 8 pulses
    const DEFAULT_POFFSET_UR = 0;       // 0 offset
    const DEFAULT_POFFSET_DL = 0;       // 0 offset      
    const DEFAULT_CONFIG1 = 0x60;    // No 12x wait (WTIME) factor
    const DEFAULT_AILT = 0xFFFF;  // Force interrupt for calibration
    const DEFAULT_AIHT = 0;
    const DEFAULT_PERS = 0x11;    // 2 consecutive prox or ALS for int.
    const DEFAULT_CONFIG2 = 0x01;    // No saturation interrupts or LED boost  
    const DEFAULT_CONFIG3 = 0;       // Enable all photodiodes, no SAI
    const DEFAULT_LDRIVE = LED_DRIVE_100MA;
    const DEFAULT_AGAIN = AGAIN_4X;

    const OFF = 0;
    const POWER = 0;
    const AMBIENT_LIGHT = 1;
    const ALL = 7;

    const red_wb = 2130;
    const green_wb = 3500;
    const blue_wb = 4620;

    function i2cwrite(reg: number, value: number) {
        let buf = pins.createBuffer(2);
        buf[0] = reg;
        buf[1] = value;
        pins.i2cWriteBuffer(APDS9960_I2C_ADDR, buf);
    }

    function i2cread(reg: number): number {
        pins.i2cWriteNumber(APDS9960_I2C_ADDR, reg, NumberFormat.UInt8BE);
        let val = pins.i2cReadNumber(APDS9960_I2C_ADDR, NumberFormat.UInt8BE);
        return val;
    }


    function InitColor(): boolean {
        let id = i2cread(APDS9960_ID);
        //  serial.writeLine("id:")
        //  serial.writeNumber(id); 
        // if (!(id == APDS9960_ID_1 || id == APDS9960_ID_2)) {
        //     return false;
        // }
        //  serial.writeLine("set mode:")
        setMode(ALL, OFF);
        i2cwrite(APDS9960_ATIME, DEFAULT_ATIME);
        i2cwrite(APDS9960_WTIME, DEFAULT_WTIME);
        i2cwrite(APDS9960_PPULSE, DEFAULT_PROX_PPULSE);
        i2cwrite(APDS9960_POFFSET_UR, DEFAULT_POFFSET_UR);
        i2cwrite(APDS9960_POFFSET_DL, DEFAULT_POFFSET_DL);
        i2cwrite(APDS9960_CONFIG1, DEFAULT_CONFIG1);
        setLEDDrive(DEFAULT_LDRIVE);
        setAmbientLightGain(DEFAULT_AGAIN);
        setLightIntLowThreshold(DEFAULT_AILT);
        setLightIntHighThreshold(DEFAULT_AIHT);
        i2cwrite(APDS9960_PERS, DEFAULT_PERS);
        i2cwrite(APDS9960_CONFIG2, DEFAULT_CONFIG2);
        i2cwrite(APDS9960_CONFIG3, DEFAULT_CONFIG3);
        return true;
    }

    function setLEDDrive(drive: number) {
        let val = i2cread(APDS9960_CONTROL);
        /* Set bits in register to given value */
        drive &= 0b00000011;
        drive = drive << 6;
        val &= 0b00111111;
        val |= drive;
        i2cwrite(APDS9960_CONTROL, val);
    }

    function setLightIntLowThreshold(threshold: number) {
        let val_low = threshold & 0x00FF;
        let val_high = (threshold & 0xFF00) >> 8;
        i2cwrite(APDS9960_AILTL, val_low);
        i2cwrite(APDS9960_AILTH, val_high);
    }

    function setLightIntHighThreshold(threshold: number) {
        let val_low = threshold & 0x00FF;
        let val_high = (threshold & 0xFF00) >> 8;
        i2cwrite(APDS9960_AIHTL, val_low);
        i2cwrite(APDS9960_AIHTH, val_high);
    }


    function rgb2hue(r: number, g: number, b: number): number {
        let max = Math.max(r, Math.max(g, b))
        let min = Math.min(r, Math.min(g, b))
        let c = max - min;
        let hue = 0;
        let segment = 0;
        let shift = 0;
        if (c == 0)
            return 0;
        if ((r > g) && (r > b)) {
            segment = (60.0 * (g - b)) / c;
            if (segment < 0)
                hue = segment + 360;
        }
        else if ((g > b) && (g > r)) {
            segment = (60.0 * (b - r)) / c;
            hue = segment + 120;
        }
        else if ((b > g) && (b > r)) {
            segment = (60.0 * (r - g)) / c;
            hue = segment + 240;
        }
        return hue;
    }

    function setMode(mode: number, enable: number) {
        let reg_val = getMode();
        /* Change bit(s) in ENABLE register */
        enable = enable & 0x01;
        if (mode >= 0 && mode <= 6) {
            if (enable > 0) {
                reg_val |= (1 << mode);
            }
            else {
                //reg_val &= ~(1 << mode);
                reg_val &= (0xff - (1 << mode));
            }
        }
        else if (mode == ALL) {
            if (enable > 0) {
                reg_val = 0x7F;
            }
            else {
                reg_val = 0x00;
            }
        }
        i2cwrite(APDS9960_ENABLE, reg_val);
    }

    function getMode(): number {
        let enable_value = i2cread(APDS9960_ENABLE);
        return enable_value;
    }

    function enableLightSensor(interrupts: boolean) {
        setAmbientLightGain(DEFAULT_AGAIN);
        if (interrupts) {
            setAmbientLightIntEnable(1);
        }
        else {
            setAmbientLightIntEnable(0);
        }
        enablePower();
        setMode(AMBIENT_LIGHT, 1);
    }

    function setAmbientLightGain(drive: number) {
        let val = i2cread(APDS9960_CONTROL);
        /* Set bits in register to given value */
        drive &= 0b00000011;
        val &= 0b11111100;
        val |= drive;
        i2cwrite(APDS9960_CONTROL, val);
    }

    function getAmbientLightGain(): number {
        let val = i2cread(APDS9960_CONTROL);
        val &= 0b00000011;
        return val;
    }

    function enablePower() {
        setMode(POWER, 1);
    }

    function setAmbientLightIntEnable(enable: number) {
        let val = i2cread(APDS9960_ENABLE);
        /* Set bits in register to given value */
        enable &= 0b00000001;
        enable = enable << 4;
        val &= 0b11101111;
        val |= enable;
        i2cwrite(APDS9960_ENABLE, val);
    }

    function readAmbientLight(): number {
        let val = i2cread(APDS9960_CDATAL);
        let val_byte = i2cread(APDS9960_CDATAH);
        val = val + val_byte * 256;
        return val;
    }


    /**
     * Initialize the color sensor,please execute at boot time
     */
    //% weight=87 blockId=startbit_init_colorSensor block="Initialize color sensor port at %port" group="1. Declaration."
    export function startbit_init_colorSensor(port: startbit_colorSensorPort) {
        InitColor();
        enableLightSensor(true);
        control.waitMicros(100);
    }

    /**
	 *  Color sensor return the color.
	 */
    //% weight=86 blockId=startbit_checkCurrentColor block="Current color %color" group="3. Sensor." 
    export function startbit_checkCurrentColor(color: startbit_Colors): boolean {
        let c = i2cread(APDS9960_CDATAL) + i2cread(APDS9960_CDATAH) * 256;
        let r = i2cread(APDS9960_RDATAL) + i2cread(APDS9960_RDATAH) * 256;
        let g = i2cread(APDS9960_GDATAL) + i2cread(APDS9960_GDATAH) * 256;
        let b = i2cread(APDS9960_BDATAL) + i2cread(APDS9960_BDATAH) * 256;

        // serial.writeNumber(c);
        // serial.writeLine("->ccc");
        // serial.writeNumber(r);
        // serial.writeLine("->red");
        // serial.writeNumber(g);
        // serial.writeLine("->green");
        // serial.writeNumber(b);
        // serial.writeLine("->blue");

        if (r > red_wb)
            r = red_wb;
        if (g > green_wb)
            g = green_wb;
        if (b > blue_wb)
            b = blue_wb;

        r = Math.round(mapRGB(r, 0, red_wb, 0, 255));
        g = Math.round(mapRGB(g, 0, green_wb, 0, 255));
        b = Math.round(mapRGB(b, 0, blue_wb, 0, 255));
        // serial.writeNumber(r);
        // serial.writeLine("->rred");
        // serial.writeNumber(g);
        // serial.writeLine("->ggreen");
        // serial.writeNumber(b);
        // serial.writeLine("->bblue");
         let hsv = rgb2hue(r, g, b);
        // serial.writeNumber(hsv);
        // serial.writeLine("->hsv");
        let t = startbit_Colors.None;
        if (c > 2200 && r > 65 && g > 65 && b > 65) {
            t = startbit_Colors.White;
        }
        else if (c > 800) {
            if (hsv < 8 || hsv > 350)
                t = startbit_Colors.Red;
            else if (hsv > 60 && hsv < 170) {
                t = startbit_Colors.Green;
            }
            else if (hsv > 210 && hsv < 230) {
                t = startbit_Colors.Blue;
            }
        }
        else if (c > 200 && r > 10 && g > 7 && b > 7 && r < 16.5 && g < 15 && b < 14) {
            t = startbit_Colors.Black;
        }
        return (color == t);
    }
    /**
    * Get the volume level detected by the sound sensor, range 0 to 255
    */
    //% weight=82 blockId=startbit_getSoundVolume block="Sound volume" group="3. Sensor."
    export function startbit_getSoundVolume(): number {
        return volume;
    }

    /**
    * Get the obstacle avoidance sensor status,1 detect obstacle,0 no detect obstacle
    * Pin 1 with Port 1
    * Pin 13 with Port 2
    * Pin ADC with Port 3
    */
    //% weight=85 blockId=startbit_avoidSensor block="Obstacle avoidance sensor|port %port|detect obstacle" group="3. Sensor."
    export function startbit_avoidSensor(port: startbit_touchKeyPort): boolean {
        let status = 0;
        let flag: boolean = false;
        switch (port) {
            case startbit_touchKeyPort.port1:
                pins.setPull(DigitalPin.P1, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P1);
                break;
            case startbit_touchKeyPort.port2:
                pins.setPull(DigitalPin.P13, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P13);
                break;
            case startbit_touchKeyPort.port3:
                if (P14_ad > 0xA)
                    status = 1
                else
                    status = 0;
                break;
        }
        if (status == 1)
            flag = false;
        else
            flag = true;
        return flag;
    }
    /**
    * Get the gas sensor status,0 detect gas,1 no detect gas
    * Pin 1 with Port 1
    * Pin 13 with Port 2
    * Pin ADC with Port 3
    */
    //% weight=85 blockId=startbit_gasSensor block="gas sensor|port %port|detect Gas" group="3. Sensor."  advanced=true
    export function startbit_gasSensor(port: startbit_touchKeyPort): boolean {
        let status = 0;
        let flag: boolean = false;
        switch (port) {
            case startbit_touchKeyPort.port1:
                pins.setPull(DigitalPin.P1, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P1);
                break;
            case startbit_touchKeyPort.port2:
                pins.setPull(DigitalPin.P13, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P13);
                break;
            case startbit_touchKeyPort.port3:
                if (P14_ad > 0xA)
                    status = 1
                else
                    status = 0;
                break;
        }
        if (status == 1)
            flag = false;
        else
            flag = true;
        return flag;
    }

    /**
    * Get the condition of the line follower sensor
    */
    //% weight=84 blockId=startbit_readLineFollowerStatus block="Line follower status|port %port|%status" group="3. Sensor." 
    export function startbit_readLineFollowerStatus(port: startbit_lineFollowPort, status: startbit_lineFollower): boolean {
        let s1 = 0;
        let s2 = 0;
        switch (port) {
            case startbit_lineFollowPort.port1:
                s1 = pins.analogReadPin(AnalogPin.P1);
                s2 = pins.analogReadPin(AnalogPin.P2);
                s1 = s1 * 255 / 1023;
                s2 = s2 * 255 / 1023;
                if (s1 < 100)
                    s1 = 0;
                else
                    s1 = 1;
                if (s2 < 100)
                    s2 = 0;
                else
                    s2 = 1;
                break;
        }
        let s = ((1 & s1) << 1) | s2;
        if (s == status) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * Get the line follower sensor port ad value
     */
    //% weight=83 blockId=startbit_lineSensorValue block="Get line follower sensor|port %port|%sensor|ad value" group="3. Sensor."  advanced=true
    export function startbit_lineSensorValue(port: startbit_lineFollowPort, sensor: startbit_LineFollowerSensor): number {
        let s1 = 0;
        let s2 = 0;
        switch (port) {
            case startbit_lineFollowPort.port1:
                s1 = pins.analogReadPin(AnalogPin.P1);
                s2 = pins.analogReadPin(AnalogPin.P2);
                s1 = s1 * 255 / 1023;
                s2 = s2 * 255 / 1023;
                break;
        }
        if (sensor == startbit_LineFollowerSensor.LFSensor_1) {
            return 255 - s1;
        }
        else {
            return 255 - s2;
        }

    }


    let distanceBak = 0;
    /**
     * Get the distance of ultrasonic detection to the obstacle 
     * Port 1: echo-P2, tri-P1
     * Port 2: echo-P14, tri-P13
     */
    //% weight=81 blockId=startbit_ultrasonic  block="Ultrasonic|port %port|distance(cm)" group="3. Sensor." 
    export function startbit_ultrasonic(port: startbit_ultrasonicPort): number {
        let echoPin: DigitalPin;
        let trigPin: DigitalPin;
        switch (port) {
            case startbit_ultrasonicPort.port1:
                echoPin = DigitalPin.P2;
                trigPin = DigitalPin.P1;
                break;
            case startbit_ultrasonicPort.port2:
                echoPin = DigitalPin.P14;
                trigPin = DigitalPin.P13;
                break;
        }
        pins.setPull(echoPin, PinPullMode.PullNone);
        pins.setPull(trigPin, PinPullMode.PullNone);

        pins.digitalWritePin(trigPin, 0);
        control.waitMicros(2);
        pins.digitalWritePin(trigPin, 1);
        control.waitMicros(10);
        pins.digitalWritePin(trigPin, 0);
        control.waitMicros(5);
        let d = pins.pulseIn(echoPin, PulseValue.High, 25000);
        let distance = d;
        // filter timeout spikes
        if (distance == 0 && distanceBak != 0) {
            distance = distanceBak;
        }
        distanceBak = d;
        return Math.round(distance * 10 / 6 / 58);
    }


	
    /**
    * Get the ad value of the photosensitive moudule
    * Port 1: P1 --- analog 
    */
    //% weight=79 blockId=startbit_getphotosensitiveValue  block="Get Photosensitive|port %port|value(0~255)"  group="3. Sensor." 
    export function startbit_getphotosensitiveValue(port: startbit_photosensitivePort): number {
        let adValue = pins.analogReadPin(AnalogPin.P1);
        adValue = adValue * 255 / 1023;
        return 255 - adValue;
    }

    /**
    * Get the Photosensitive sensor status,1 detect bright,0 no detect bright
    * Port 1: P2 --- digital
    */
    //% weight=78 blockId=startbit_photosensitiveSensor blockGap=50 block="Photosensitive sensor|port %port|detect bright" group="3. Sensor."  advanced=true
    export function startbit_photosensitiveSensor(port: startbit_PhotosensitiveSensor): boolean {
        let status = 0;
        let flag: boolean = false;
        switch (port) {
            case startbit_PhotosensitiveSensor.port1:
                pins.setPull(DigitalPin.P2, PinPullMode.PullUp);
                status = pins.digitalReadPin(DigitalPin.P2);
                break;
        }
        if (status == 1)
            flag = false;
        else
            flag = true;
        return flag;
    }




    /**
	 * Initialize RGB
	 */
    function startbit_initRGBLight() {
        if (!lhRGBLight) {
            lhRGBLight = StartbitRGBLight.create(DigitalPin.P15, 6, StartbitRGBPixelMode.RGB);
        }
        startbit_clearLight();
    }

    /**
         * Set the brightness of the strip. This flag only applies to future operation.
         * @param brightness a measure of LED brightness in 0-255. eg: 255
    */
    //% blockId="startbit_setBrightness" block="set brightness %brightness" group="1. Declaration."
    //% weight=68
    export function startbit_setBrightness(brightness: number): void {
        lhRGBLight.setBrightness(brightness);
    }

    /**
     * Set the color of the colored lights, after finished the setting please perform  the display of colored lights.
     */
    //% weight=67 blockId=startbit_setPixelRGB block="Set|%lightoffset|color to %rgb" group="4. The Light." 
    export function startbit_setPixelRGB(lightoffset: StartbitLights, rgb: StartbitRGBColors) {
        lhRGBLight.setPixelColor(lightoffset, rgb);
    }


    /**
     * Set RGB Color argument
     */
    //% weight=66 blockId=startbit_setPixelRGBArgs block="Set|%lightoffset|color to %rgb" group="4. The Light."
    export function startbit_setPixelRGBArgs(lightoffset: StartbitLights, rgb: number) {
        lhRGBLight.setPixelColor(lightoffset, rgb);
    }


    /**
     * Display the colored lights, and set the color of the colored lights to match the use. After setting the color of the colored lights, the color of the lights must be displayed.
     */
    //% weight=65 blockId=startbit_showLight block="Show light" group="4. The Light." 
    export function startbit_showLight() {
        lhRGBLight.show();
    }

    /**
     * Clear the color of the colored lights and turn off the lights.
     */
    //% weight=64 blockGap=50 blockId=startbit_clearLight block="Clear light" group="4. The Light."
    export function startbit_clearLight() {
        lhRGBLight.clear();
    }

    /**
	 * Initialize Light belt
	 */
    //% weight=63 blockId=startbit_belt_initRGBLight block="Initialize light belt at port %port" group="1. Declaration."  advanced=true
    export function startbit_belt_initRGBLight(port: startbit_ultrasonicPort) {
        switch (port) {
            case startbit_ultrasonicPort.port1:
                if (!lhRGBLightBelt) {
                    lhRGBLightBelt = StartbitRGBLight.create(DigitalPin.P1, 30, StartbitRGBPixelMode.RGB);
                }
                break;
            case startbit_ultrasonicPort.port2:
                if (!lhRGBLightBelt) {
                    lhRGBLightBelt = StartbitRGBLight.create(DigitalPin.P13, 30, StartbitRGBPixelMode.RGB);
                }
                break;
        }

        startbit_clearLight();
    }

    /**
     * Set the color of the colored lights, after finished the setting please perform  the display of colored lights.
     */
    //% weight=62 blockId=startbit_belt_setPixelRGB block="Set light belt|%lightoffset|color to %rgb" group="4. The Light."  advanced=true
    export function startbit_belt_setPixelRGB(lightoffset: StartbitLightsBelt, rgb: StartbitRGBColors) {
        lhRGBLightBelt.setBeltPixelColor(lightoffset, rgb);
    }

    /**
     * Display the colored lights, and set the color of the colored lights to match the use. After setting the color of the colored lights, the color of the lights must be displayed.
     */
    //% weight=61 blockId=startbit_belt_showLight block="Show light belt"group="4. The Light."  advanced=true
    export function startbit_belt_showLight() {
        lhRGBLightBelt.show();
    }

    /**
     * Clear the color of the colored lights and turn off the lights.
     */
    //% weight=60 blockGap=50 blockId=startbit_belt_clearLight block="Clear light belt" group="4. The Light."  advanced=true
    export function startbit_belt_clearLight() {
        lhRGBLightBelt.clear();
    }

    function mapRGB(x: number, in_min: number, in_max: number, out_min: number, out_max: number): number {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
    * Get the condition of the touch button,press return 1,or return 0
    * Port 1: P1
    * Port 2: P13
    * port 3: 
    */
    //% weight=82 blockId=startbit_touchButton block="Touch button|port %port|is pressed"    group="5. Mechaincs." 
    export function startbit_touchButton(port: startbit_touchKeyPort): boolean {
        let status: boolean = false;
        switch (port) {
            case startbit_touchKeyPort.port1:
                pins.setPull(DigitalPin.P1, PinPullMode.PullUp);
                status = !pins.digitalReadPin(DigitalPin.P1);
                break;
            case startbit_touchKeyPort.port2:
                pins.setPull(DigitalPin.P13, PinPullMode.PullUp);
                status = !pins.digitalReadPin(DigitalPin.P13);
                break;
            case startbit_touchKeyPort.port3:
                if (P14_ad > 0xA)
                    status = false;
                else
                    status = true;
                break;
        }
        return status;
    }
    /**
    * Get the ad value of the knob moudule
    */
    //% weight=80 blockId=startbit_getKnobValue  block="Get knob|port %port|value(0~255)" group="5. Mechaincs." advanced=true
    export function startbit_getKnobValue(port: startbit_knobPort): number {
        let adValue = pins.analogReadPin(AnalogPin.P1);
        adValue = adValue * 255 / 1023;
        return adValue;
    }

    
    /**
     *  LCD
     */
    export let LCD_I2C_ADDR = 0x3f
    let buf = 0x00
    let BK = 0x08
    let RS = 0x00
    let E = 0x04

    function setReg(dat: number): void {
        pins.i2cWriteNumber(LCD_I2C_ADDR, dat, NumberFormat.UInt8BE, false)
        basic.pause(1)
    }

    function send(dat: number): void {
        let d = dat & 0xF0
        d |= BK
        d |= RS
        setReg(d)
        setReg(d | 0x04)
        setReg(d)
    }

    function setcmd(cmd: number): void {
        RS = 0
        send(cmd)
        send(cmd << 4)
    }

    function setdat(dat: number): void {
        RS = 1
        send(dat)
        send(dat << 4)
    }

    function setI2CAddress(): void {
        setcmd(0x33)
        basic.pause(5)
        send(0x30)
        basic.pause(5)
        send(0x20)
        basic.pause(5)
        setcmd(0x28)
        setcmd(0x0C)
        setcmd(0x06)
        setcmd(0x01)
    } 

    /**
     * Initialize the I2C address
     */
    //% blockId="LCD_setAddress" block=" Initialize LCD1602 I2C |%port address |%myAddr" group="1. Declaration."
    //% weight=51 blockExternalInputs=true
    export function setAddress(port: startbit_LCDPort, myAddr: I2C_ADDR): void {
        LCD_I2C_ADDR = myAddr
        setI2CAddress()
    }

    /**
     * Initialize the I2C address (number)
     */
   // //% blockId="LCD_setAddress2" block="LCD1602 I2C address %myAddr" group="8. Display" 
    //% weight=50 blockExternalInputs=true
    //export function setAddress2(myAddr: number): void {
    //    LCD_I2C_ADDR = myAddr
    //    setI2CAddress()
    //}
    /*
    function AutoAddr() {
        let k = true
        let addr = 0x20
        let d1 = 0, d2 = 0
        while (k && (addr < 0x28)) {
            pins.i2cWriteNumber(addr, -1, NumberFormat.Int32LE)
            d1 = pins.i2cReadNumber(addr, NumberFormat.Int8LE) % 16
            pins.i2cWriteNumber(addr, 0, NumberFormat.Int16LE)
            d2 = pins.i2cReadNumber(addr, NumberFormat.Int8LE)
            if ((d1 == 7) && (d2 == 0)) k = false
            else addr += 1
        }
        if (!k) return addr
         addr = 0x38
        while (k && (addr < 0x40)) {
            pins.i2cWriteNumber(addr, -1, NumberFormat.Int32LE)
            d1 = pins.i2cReadNumber(addr, NumberFormat.Int8LE) % 16
            pins.i2cWriteNumber(addr, 0, NumberFormat.Int16LE)
            d2 = pins.i2cReadNumber(addr, NumberFormat.Int8LE)
            if ((d1 == 7) && (d2 == 0)) k = false
            else addr += 1
        }
        if (!k) return addr
        else return 0
    }
    */

    /**
     * Automatically initialize the I2C address
     */
    // //% blockId="LCD_setAddress3" block="Auto set LCD1602 I2C address" group="8. Display" 
    //% weight=50
    //export function setAddress3(): void {
     //   LCD_I2C_ADDR = AutoAddr()
     //   setI2CAddress()
    //}  

    /**
     * LCD clear 
     */
    //% blockId="LCD_clear" block="LCD clear" group="8. Display" 
    //% weight=45
    export function clear(): void {
        setcmd(0x01)
    }
   
    /**
     * Setting backlight
     */
    //% blockId="LCD_backlight" block="set LCD backlight %on" group="8. Display"  advanced=true
    //% weight=46
    export function set_backlight(on: on_off): void {
        if (on == 1)
            BK = 0x08
        else
            BK = 0x00
        setcmd(0x00)
    }

    /**
     * Show string.
     */
    //% blockId="LCD_Show" block="set string %show" group="8. Display" 
    //% weight=47
    export function set_LCD_Show(show: visibled): void {
        if (show == 1)
            setcmd(0x0C)
        else
            setcmd(0x08)
    }

    function printChar(ch: number, x: number, y: number): void {
        if (x >= 0) {
            let a = 0x80
            if (y == 1)
                a = 0xC0
	    if (y == 2)
                a = 0x80 + 0x14
            if (y == 3)
                a = 0xC0 + 0x14
            a += x
            setcmd(a)
        }
        setdat(ch)
    }

    /**
     * Show x, y string 
     */
    //% blockId="LCD_putString" block="LCD show string %s|on x:%x|y:%y" group="8. Display"   advanced=true
    //% weight=49 blockExternalInputs=true x.min=0 x.max=15 y.min=0 y.max=1
    export function putString(s: string, x: number, y: number): void {
        if (s.length > 0) {
            let breakPoint = -1
            printChar(s.charCodeAt(0), x, y)
            if (y == 0)
                breakPoint = 16 - x
            for (let i = 1; i < s.length; i++) {
                if (i == breakPoint)
                    printChar(s.charCodeAt(i), 0, 1)
                else
                    printChar(s.charCodeAt(i), -1, 0)
            }
        }
    }
	
    /**
     * Show number
     */
    //% blockId="LCD_putNumber" block="LCD show number %n|on x:%x|y:%y" group="8. Display"  
    //% weight=48 blockExternalInputs=true x.min=0 x.max=15 y.min=0 y.max=1
    export function putNumber(n: number, x: number, y: number): void {
        putString(n.toString(),x,y)
    }

    /**
     * Shift Left
     */
    //% blockId="LCD_shl" block="Shift Left" group="8. Display"   advanced=true
    //% weight=43
    export function shl(): void {
        setcmd(0x18)
    }

    /**
     * Shift Right
     */
    //% blockId="LCD_shr" block="Shift Right" group="8. Display"   advanced=true
    //% weight=42
    export function shr(): void {
        setcmd(0x1C)
    }

        /**
        * TM1640 LED display
        */
       export class startbit_TM1640LEDs {
        buf: Buffer;
        clk: DigitalPin;
        dio: DigitalPin;
        _ON: number;
        brightness: number;
        count: number;  // number of LEDs

        /**
         * initial TM1640
         */
        init(): void {
            pins.digitalWritePin(this.clk, 0);
            pins.digitalWritePin(this.dio, 0);
            this._ON = 8;
            this.buf = pins.createBuffer(this.count);
            this.clear();
        }

        /**
         * Start 
         */
        _start() {
            pins.digitalWritePin(this.dio, 0);
            pins.digitalWritePin(this.clk, 0);
        }

        /**
         * Stop
         */
        _stop() {
            pins.digitalWritePin(this.dio, 0);
            pins.digitalWritePin(this.clk, 1);
            pins.digitalWritePin(this.dio, 1);
        }

        /**
         * send command1
         */
        _write_data_cmd() {
            this._start();
            this._write_byte(TM1640_CMD1);
            this._stop();
        }

        /**
         * send command3
         */
        _write_dsp_ctrl() {
            this._start();
            this._write_byte(TM1640_CMD3 | this._ON | this.brightness);
            this._stop();
        }

        /**
         * send a byte to 2-wire interface
         */
        _write_byte(b: number) {
            for (let i = 0; i < 8; i++) {
                pins.digitalWritePin(this.clk, 0);
                pins.digitalWritePin(this.dio, (b >> i) & 1);
                pins.digitalWritePin(this.clk, 1);

            }
            pins.digitalWritePin(this.clk, 1);
            pins.digitalWritePin(this.clk, 0);
        }

        intensity(val: number = 7) {
            if (val < 1) {
                this.off();
                return;
            }
            if (val > 8) val = 8;
            this._ON = 8;
            this.brightness = val - 1;
            this._write_data_cmd();
            this._write_dsp_ctrl();
        }

        /**
         * set data to TM1640, with given bit
         */
        _dat(bit: number, dat: number) {
            this._write_data_cmd();
            this._start();
            this._write_byte(TM1640_CMD2 | (bit % this.count))
            this._write_byte(dat);
            this._stop();
            this._write_dsp_ctrl();
        }


        showbit(num: number = 5, bit: number = 0) {
            this.buf[bit % this.count] = _SEGMENTS[num % 16]
            this._dat(bit, _SEGMENTS[num % 16])
        }

        showNumber(num: number) {
            if (num < 0) {
                this._dat(0, 0x40) // '-'
                num = -num
            }
            else
                this.showbit(Math.idiv(num, 1000) % 10)
            this.showbit(num % 10, 3)
            this.showbit(Math.idiv(num, 10) % 10, 2)
            this.showbit(Math.idiv(num, 100) % 10, 1)
        }

        showHex(num: number) {
            if (num < 0) {
                this._dat(0, 0x40) // '-'
                num = -num
            }
            else
                this.showbit((num >> 12) % 16)
            this.showbit(num % 16, 3)
            this.showbit((num >> 4) % 16, 2)
            this.showbit((num >> 8) % 16, 1)
        }


        showDP(bit: number = 1, show: boolean = true) {
            bit = bit % this.count
            if (show) this._dat(bit, this.buf[bit] | 0x80)
            else this._dat(bit, this.buf[bit] & 0x7F)
        }

        clear() {
            for (let i = 0; i < this.count; i++) {
                this._dat(i, 0)
                this.buf[i] = 0
            }
        }

        on() {
            this._ON = 8;
            this._write_data_cmd();
            this._write_dsp_ctrl();
        }

        off() {
            this._ON = 0;
            this._write_data_cmd();
            this._write_dsp_ctrl();
        }
    }
    /**
     * TM 1640 Module.
     * @param clk the CLK pin for TM1640, eg: DigitalPin.P1, P13
     * @param dio the DIO pin for TM1640, eg: DigitalPin.P2, P14
     * @param intensity the brightness of the LED, eg: 7
     * @param count the count of the LED, eg: 4
     */
    function startbit_TM1640create(port: startbit_digitaltubePort, intensity: number, count: number): startbit_TM1640LEDs {
        let digitaltube = new startbit_TM1640LEDs();
        switch (port) {
            case startbit_digitaltubePort.port1:
                digitaltube.clk = DigitalPin.P2;
                digitaltube.dio = DigitalPin.P1;
                break;
            case startbit_digitaltubePort.port2:
                digitaltube.clk = DigitalPin.P14;
                digitaltube.dio = DigitalPin.P13;
                break;
        }

        if ((count < 1) || (count > 5)) count = 4;
        digitaltube.count = count;
        digitaltube.brightness = intensity;
        digitaltube.init();
        return digitaltube;
    }

  /**
     * MT1640: Port 1: clk-P1, dio-P2 ; Port 2: clk-P13, dio-P14 ; 
     * @param clk the CLK pin for TM1640, eg: DigitalPin.P1
     * @param dio the DIO pin for TM1640, eg: DigitalPin.P2
     * @param intensity the brightness of the LED, eg: 7
     * @param count the count of the LED, eg: 4
     */
    //% weight=77 blockId=startbit_digitaltube block="digitaltube|%port|intensity %intensity|LED count %count" group="1. Declaration."  advanced=true 
    export function startbit_digitaltube(port: startbit_digitaltubePort, intensity: number, count: number) { 
        Digitaltube = startbit_TM1640create(port, intensity, count);
    }

    /**
     * show a number. 
     * @param num is a number, eg: 0
     */
    //% weight=76 blockId=startbit_showNumber block="digitaltube show number| %num"  group="8. Display"   advanced=true
    export function startbit_showNumber(num: number)  {
        Digitaltube.showNumber(num);
    }

    /**
     * show a number in given position. 
     * @param num number will show, eg: 5
     * @param bit the position of the LED, eg: 0
     */
    //% weight=75 blockId=startbit_showbit block="digitaltube show digit| %num|at %bit"  group="8. Display"   advanced=true
    export function startbit_showbit(num: number = 5, bit: number = 0) {
        Digitaltube.showbit(num, bit);
    }

    /**
     * show a hex number. 
     * @param num is a hex number, eg: 0
     */
    //% weight=74 blockId=startbit_showhex block="digitaltube show hex number| %num"  group="8. Display"   advanced=true
    export function startbit_showhex(num: number) {
        Digitaltube.showHex(num);
    }

    /**
     * show or hide dot point. 
     * @param bit is the position, eg: 1
     * @param show is show/hide dp, eg: true
     */
    //% weight=73 blockId=startbit_showDP block="digitaltube DotPoint at| %bit|show %show"  group="8. Display"   advanced=true
    export function startbit_showDP(bit: number = 1, show: boolean = true) {
        Digitaltube.showDP(bit, show);
    } 

    /**
     * set TM1640 intensity, range is [0-8], 0 is off.
     * @param val the brightness of the TM1640, eg: 7
     */
    //% weight=72 blockId=startbit_intensity block=" digitaltube set intensity %val"  group="8. Display"   advanced=true
    export function startbit_intensity(val: number = 7) {
        Digitaltube.intensity(val);
    } 

    /**
     * turn off LED. 
     */
    //% weight=71 blockId=startbit_off block="turn off digitaltube"  group="8. Display"   advanced=true
    export function startbit_off() {
        Digitaltube.off();
    }

    /**
     * turn on LED. 
     */
    //% weight=70 blockId=startbit_on block="turn on digitaltube"  group="8. Display"   advanced=true
    export function startbit_on() {
        Digitaltube.on();
    }

    /**
     * clear LED. 
     */
    //%weight=69 blockId=startbit_clear blockGap=50 block="clear digitaltube"  group="8. Display"   advanced=true
    export function startbit_clear() {
        Digitaltube.clear();
    }  
}
