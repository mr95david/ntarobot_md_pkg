# File for all constans orders for MD49 driver
# For docs: http://www.robot-electronics.co.uk/htm/md49tech.htm

## General Order Constants for Get values in MD49
VALSYNC             =       b"\x00" # Order for Sync
# VALOR REAL
VALGETSPEED1        =       b"\x21" # Obtencion de valor de velocidad 1
VALGETSPEED2        =       b"\x22" # Obtencion de valor de velocidad 2
VALGETENCODER1      =       b"\x23" # Obtencion de valor de encoder 1
VALGETENCODER2      =       b"\x24" # Obtencion de valor de encoder 2
# VALOR INVERTIDO
# VALGETSPEED2        =       b"\x21" # Obtencion de valor de velocidad 1
# VALGETSPEED1       =        b"\x22" # Obtencion de valor de velocidad 2
# VALGETENCODER2      =       b"\x23" # Obtencion de valor de encoder 1
# VALGETENCODER1     =        b"\x24" # Obtencion de valor de encoder 2

VALGETENCODERS      =       b"\x25" # Obtencion de valor de los encoders juntos
VALGETVOLTS         =       b'\x26' # Obtencion de voltios de driver
# VALOR REAL
VALGETCURRENT1      =       b"\x27" # Obtencion de valor de corriente del motor 1
VALGETCURRENT2      =       b"\x28" # Obtencion de valor de corriente del motor 2
# VALOR INVERTIDO
# VALGETCURRENT2      =       b"\x27" # Obtencion de valor de corriente del motor 1
# VALGETCURRENT1      =       b"\x28" # Obtencion de valor de corriente del motor 2
VALGETVERSION       =       b"\x29" # Obtencion de valor de version
VALGETACELERATION   =       b"\x2A" # Obtencion de valor del nivel de aceleracion
VALGETMODE          =       b'\x2B' # Obtener el valor del modo actual del driver
VALGETVI            =       b'\x2C' # Forma rapida de obtener el valor de voltaje, y corrientes
VALGETERROR         =       b'\x2D' # Obtencion de valor de Errores

# Constantes para variacion de valores
# De manera temporal, los valores de los motores deben ser invertidos
VALSETSPEED1        =       b'\x31' # Variacion de velocidad del motor 1
VALSETSPEED2        =       b'\x32' # Variacion de velocidad del motor 2
# CONDIGO DE ORDEN INVERTIDO
# VALSETSPEED2        =       b'\x31' # Variacion de velocidad del motor 1
# VALSETSPEED1        =       b'\x32' # Variacion de velocidad del motor 2

VALSETACELERATION   =       b'\x33' # Variacion de Aceleracion del sistema
VALSETMODE          =       b'\x34' # Variacion de Modo de funcionamiento del sistema
RESETENCODERS      =        b'\x35' # Funcion para resetear el valor de los encoders
DISABLEREGULATOR   =        b'\x36' # Funcion para deshabiltar el regulador de velocidad del control de motores
ENABLEREGULATOR    =        b'\x37' # Funcion para habilitar el regulador de velocidad del control de motores
DISABLETIMEOUT     =        b'\x38' # Dehabilitar el timeout
ENABLETIMEOUT      =        b'\x39' # Habilitar el time out

# General variables
DICT_STATE_PARAMS = {
        True: "Enable",
        False: "Disabled"
    }