# Usb connection md 49
# April 23, 2024. Universidade Federal do Espirito Santo.
# Elio Triana.

# Import Libraries of binary orders #
from ntarobot_pkg.untils_md import *
# Serial Communication libraries from python native #
from serial import Serial, PARITY_NONE, STOPBITS_ONE, EIGHTBITS
from serial.tools import list_ports # TODO: Make better validation of connection in md
# Untils Libraries from python #
from os import access, W_OK
from struct import pack, unpack
# Optionals #
from time import sleep

# Object for Serial communication #
# FATHER CLASS #
class ConnPort:
    # Instance variables 
    errConn: str = "Process without errors"
    stateConn: bool = True
    def __init__(
            self,
            namePort: str = "",
            baudRate: int = 9600,
            #baudRate: int = 38400,
            timeOut: float = 1.0
        ) -> None:
        # Inicialization of vars
        self.namePort = namePort
        self.baudRate = baudRate
        self.timeOut = timeOut
        self._mode = 0
        # Inicialization of port
        if not self._initPort():
            self.portSerial = None
    
    # Module functions of connection object
    # Inicialization port Function
    def _initPort(self) -> bool:
        valPort = False
        # Check Port State Connect
        try:
            self.portSerial = Serial(
                port = self.namePort,
                baudrate = self.baudRate,
                parity = PARITY_NONE,
                bytesize = EIGHTBITS,
                stopbits = STOPBITS_ONE,
                timeout = self.timeOut,
            )
            valPort = True
            return valPort
        except Exception as e:
            self.stateConn = False
            self.errConn = f"Connection Error. Number of Error: {e}"
            return False
        # 
    
    # Function for update state of connection
    def _updateState(self) -> None:
        self.stateConn = access(self.namePort, W_OK)
        if not self.stateConn and self.portSerial is not None:
            self._closePort()

    # Function for close serial port        
    def _closePort(self) -> None:
        try:
            self.portSerial.close()
            print("Closed port successfully")
        except Exception as e:
            print(f"![ERROR]!.Can't close Port. Error Number {e}")
        self.portSerial = None

# Object for md49 driver #
# SON CLASS #
class driverMD49(ConnPort):
    # FUNCTIONS TO EXTRACT SUPPLY GENERAL PARAMETERS
    # Voltage Read Function
    def GetVolts(self):
        if self.ordDataBytes(VALGETVOLTS):
            ret = self.readGenValues(1)
            if ret is not None:
                return unpack('B', ret)[0]
        return None

    # Read Function Current motor 1
    def GetCurrent1(self) :
        if self.ordDataBytes(VALGETCURRENT1) :
            ret = self.readGenValues(1)
            if ret is not None :
                return unpack('B', ret)[0] / 10
        return None
    
    # Read Function Current motor 2
    def GetCurrent2(self) :
        if self.ordDataBytes(VALGETCURRENT2) :
            ret = self.readGenValues(1)
            if ret is not None :
                return unpack('B', ret)[0] #/ 10
        return None
    
    # Extract information from params of software
    # Read Function of software version
    def GetVersion(self) :
        if self.ordDataBytes(VALGETVERSION) :
            ret = self.readGenValues(1)
            if ret is not None :
                return unpack('B', ret)[0]
        return None
    # Read Function to get actual aceleration
    def GetAcceleration(self) :
        if self.ordDataBytes(VALGETACELERATION) :
            ret = self.readGenValues(1)
            if ret is not None :
                return unpack('B', ret)[0]
        return None
    # Get function for actual mode
    def GetMode(self) :
        if self.ordDataBytes(VALGETMODE) :
            ret = self.readGenValues(1)
            if ret is not None :
                return unpack('B', ret)[0]
        return None
    
    ## SECTION FOR GET SPEED AND COUNTS FROM ENCODERS ##
    # Read function for speed from rigth motor
    def GetSpeed1(self) :
        if self.ordDataBytes(VALGETSPEED1) :
            ret = self.readGenValues(1)
            if ret is not None :
                typeByte = 'B' if self._mode == 0 or self._mode == 2 else 'b'
                return unpack(typeByte, ret)[0]
        return None
    # Read function for speed from left motor
    def GetSpeed2(self) :
        if self.ordDataBytes(VALGETSPEED2) :
            ret = self.readGenValues(1)
            if ret is not None :
                typeByte = 'B' if self._mode == 0 or self._mode == 2 else 'b'
                return unpack(typeByte, ret)[0]
        return None

    # Read function to get encoder value rigth
    def GetEncoder1(self) :
        if self.ordDataBytes(VALGETENCODER1):
            ret = self.readGenValues(4)
            if ret is not None :
                return unpack('>i', ret)[0]
        return None

    # Read function to get encoder value left
    def GetEncoder2(self) :
        if self.ordDataBytes(VALGETENCODER2) :
            ret = self.readGenValues(4)
            if ret is not None :
                return unpack('>i', ret)[0]
        return None
    
    # SECCION DE FUNCIONES PARA MODIFICACION DE VELOCIDAD
    # Funcion de variacion de velocidad de motor 1
    def setSpeed1(self, valSpeed: int = 128):
        if self._mode == 0 or self._mode == 2 :
            typeByte = 'B'
            if valSpeed < 0 or valSpeed > 255 :
                return False
        else :
            typeByte = 'b'
            if valSpeed < -128 or valSpeed > 127 :
                return False
        return self.ordDataBytes(
            VALSETSPEED1,
            pack(typeByte, valSpeed)
        )
    
    # Funcion de variacion de velocidad de motor 1
    def setSpeed2(self, valSpeed: int = 128):
        if self._mode == 0 or self._mode == 2 :
            typeByte = 'B'
            if valSpeed < 0 or valSpeed > 255 :
                return False
        else :
            typeByte = 'b'
            if valSpeed < -128 or valSpeed > 127 :
                return False
        return self.ordDataBytes(
            VALSETSPEED2,
            pack(typeByte, valSpeed)
        )
    
    # Seccion de funciones de validacion de modo de funcionamiento del robot
    def ResetEncoders(self) :
        return self.ordDataBytes(RESETENCODERS)

    # Funcion para deshabiliitar el regulador de velocidad
    def DisableRegulator(self) :
        return self.ordDataBytes(DISABLEREGULATOR)

    # Funcion para deshabiliitar el regulador de velocidad
    def EnableRegulator(self) :
        return self.ordDataBytes(ENABLEREGULATOR)

    # Funcion para deshabiliitar el timeout de la comunicacion
    def DisableTimeout(self) :
        return self.ordDataBytes(DISABLETIMEOUT)

    # Funcion para habilitar el timeout de la comunicacion
    def EnableTimeout(self) :
        return self.ordDataBytes(ENABLETIMEOUT)

    # Section for untils get and set functions #
    # Function for make specific order #
    def ordDataBytes(self, commandByte: bytes, extraValue = None) -> bool:
        # Validation length of order data. if len == 2 is a get function, if len == 3 is a set function
        if extraValue is None:
            order = VALSYNC + commandByte
        else:
            order = VALSYNC + commandByte + extraValue
        # Loop to get smallest value from buffer
        for i in range(100):
            val_res = self.portSerial.write(order)
            if val_res is not None and val_res == len(order):
                return True
            sleep(0.01)
        return False
    
    # Read Function - Make verification of buffer of received data
    def readGenValues(self, lenValues: int = 0):
        retVal = b''
        val_OK = False
        for i in range(100) :
            buf = self.portSerial.read_all()
            if buf is not None :
                retVal += buf
                if len(retVal) == lenValues:
                    val_OK = True
                    break
            sleep(0.01)
        return retVal if val_OK else None
    
    # Function for byte correction
    def byteCorrection(self, posByte, byte):
        # Validation of byte possition -
        # Normal possition between 1 - 8
        if posByte >= 1 and posByte <= 8:
            # Make byte mask for final response validation
            mask = pack('B', 0 | 1 << 8 - posByte)[0]
            # Mask comparation. Input byte value validation
            result = byte[0] and mask
            # Return result of validation
            return result > 0
        # Return byte value
        return byte

if __name__ == "__main__":
    print("Paquete ejecutado correctamente")
