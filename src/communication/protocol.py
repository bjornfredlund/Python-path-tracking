from enum import Enum


class Protocol(Enum):
    # error codes 
    ERR_CODE                = -1
    ERR_RADIO_LOST          = -1
    ERR_WIFI_LOST           = -1
    ERR_UNKNOWN_CODE        = -1

    # requests 
    COM_GET_STATES          = 'a'
    COM_GET_POS             = 'b'
    COM_GET_VELOCITY        = 'c'
    COM_SET_SPEED           = 'd'  #  m/s 
    COM_SET_STEERING_ANGLE  = 'e'  #  radians 
    COM_WRITEOUTPUT         = 'f'

    # replies 
    ANS_ACK                 = 'g' 
    ANS_NACK                = 'h'
    ANS_GET_STATES          = 'i'
    ANS_GET_POS             = 'j' 
    ANS_GET_VELOCITY        = 'k'
    ANS_SET_SPEED           = 'l'
    ANS_SET_STEERING_ANGLE  = 'm'
    ANS_WRITE_OUTPUT        = 'n'

    ANS_END                 = '\n'
    COM_END                 = '\n'
    DELIM                   = '$'

