# regs.py
from dataclasses import dataclass

class REG:
    CONTROL_MODE = 0x200D
    CONTROL_WORD = 0x200E
    SYNC_ASYNC   = 0x200F

    ACC_L = 0x2080; ACC_R = 0x2081
    DEC_L = 0x2082; DEC_R = 0x2083
    TARGET_VEL_L = 0x2088; TARGET_VEL_R = 0x2089
    ACT_VEL_L    = 0x20AB; ACT_VEL_R    = 0x20AC
    TPOS_H_L = 0x208A; TPOS_L_L = 0x208B
    TPOS_H_R = 0x208C; TPOS_L_R = 0x208D
    TSPD_L   = 0x208E; TSPD_R   = 0x208F
    APOS_H_L = 0x20A7; APOS_L_L = 0x20A8
    APOS_H_R = 0x20A9; APOS_L_R = 0x20AA
    TSLOPE_L = 0x2086; TSLOPE_R = 0x2087
    TTORQUE_L= 0x2090; TTORQUE_R= 0x2091
    ATORQUE_L= 0x20AD; ATORQUE_R= 0x20AE

    CLEAR_FEEDBACK_POS = 0x2005
    SET_ZERO = 0x2006

    # FW에 따라 주소가 다를 수 있음 (원본 그대로 유지)
    COM_TIMEOUT = 0x2014


@dataclass
class SerialCfg:
    port: str = ""
    baudrate: int = 115200
    slave: int = 1
