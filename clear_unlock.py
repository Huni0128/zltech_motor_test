# run_go_sto.py — ZLAC8030D (no brake): STO 대기 → 잠금해제 → 저속 구동
# pip install pymodbus==2.* pyserial
import time
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
try:
    from serial.tools import list_ports
except Exception:
    list_ports = None

CFG = {"BAUD":115200,"SLAVE":1,"RPM_L":80,"RPM_R":80,"DUR":6.0,"ACC_MS":1500,"STO_TIMEOUT_S":60}

REG = {
  "CONTROL_MODE":0x200D,"CONTROL_WORD":0x200E,"SYNC_ASYNC":0x200F,
  "STATUS_WORD":0x20A2,"BUS_VOLT":0x20A1,
  "ACC_L":0x2080,"ACC_R":0x2081,"DEC_L":0x2082,"DEC_R":0x2083,
  "TGT_VEL_L":0x2088,"TGT_VEL_R":0x2089, "ACT_VEL_L":0x20AB,"ACT_VEL_R":0x20AC,
  "POS_L_H":0x20A7,"POS_L_L":0x20A8,"POS_R_H":0x20A9,"POS_R_L":0x20AA,
  "ERR_L":0x20A5,"ERR_R":0x20A6,
  "INPUT_LEVEL":0x2016,"X0_FUNC":0x2017,"X1_FUNC":0x2018,  # E-stop 기능 끔
  "PARK":0x200C,"PWRON":0x2007,                           # 파킹/전원시 잠금
}

def autodetect_ports():
    if not list_ports: return []
    def score(p):
        t=((p.device or '')+' '+(p.description or '')+' '+(p.hwid or '')).upper()
        pri=0
        for k in ["USB","CH340","CP210","FTDI","PL2303"]:
            if k in t: pri-=10
        return (pri,p.device)
    detected = [p.device for p in sorted(list_ports.comports(), key=score)]
    if detected:
        return detected
    # Fallback to common Linux serial ports
    return ["/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyACM0", "/dev/ttyACM1"]

def open_client(baud):
    cands = autodetect_ports()
    for dev in cands:
        cli=ModbusSerialClient(method="rtu", port=dev, baudrate=baud, bytesize=8, parity='N', stopbits=1,
                               timeout=2.0, retries=3, retry_on_empty=True, strict=False)
        if cli.connect():
            print(f"🔌 연결 성공: {dev} @ {baud}bps"); return cli, dev
        try: cli.close()
        except: pass
    print("포트 없음"); return None, None

def r_u16(c,a,u):
    rr=c.read_holding_registers(a,1,u)
    if rr.isError(): raise ModbusException(rr)
    return rr.registers[0] & 0xFFFF
def r_i16(c,a,u):
    v=r_u16(c,a,u); return v-0x10000 if v & 0x8000 else v
def w_u16(c,a,v,u):
    rq=c.write_register(a, v & 0xFFFF, u)
    if rq.isError(): raise ModbusException(rq)
def r_i32(c,ah,al,u):
    hi=r_i16(c,ah,u); lo=r_i16(c,al,u)
    x=((hi<<16)&0xFFFFFFFF)|(lo&0xFFFF)
    return x-0x100000000 if x & 0x80000000 else x

def unlock_and_enable(client,u,acc):
    # E-stop 기능 제거 & 기본 극성
    w_u16(client, REG["X0_FUNC"], 0, u)
    w_u16(client, REG["X1_FUNC"], 0, u)
    w_u16(client, REG["INPUT_LEVEL"], 0x0000, u)
    # 파킹 해제 & 전원시 잠금 해제
    w_u16(client, REG["PARK"], 1, u)   # Open
    w_u16(client, REG["PWRON"], 0, u)  # Not lock
    # Fault clear → 속도모드/비동기/가감속 → Enable
    w_u16(client, REG["CONTROL_WORD"], 0x06, u)     # Clear
    w_u16(client, REG["CONTROL_MODE"], 3, u)        # Velocity
    w_u16(client, REG["SYNC_ASYNC"], 1, u)          # Async
    for r in ("ACC_L","ACC_R","DEC_L","DEC_R"):
        w_u16(client, REG[r], acc, u)
    w_u16(client, REG["CONTROL_WORD"], 0x08, u)     # Enable

def wait_sto_clear(client, u, timeout_s):
    t0 = time.time()
    last = -1
    while True:
        stat = r_u16(client, REG["STATUS_WORD"], u)
        if stat != last:
            print(f"   [STO wait] STAT=0x{stat:04X}  (0x0040 비트가 0이면 통과)")
            last = stat
        if (stat & 0x0040) == 0:
            print("   [STO wait] shaft lock 비트가 내려갔습니다.")
            return True
        if time.time() - t0 > timeout_s:
            print("   [STO wait] 시간 초과 — STO/락 해제 안 됨. 하드웨어 STO 회로/점퍼 확인 필요.")
            return False
        time.sleep(0.3)

def run_axis(client,u,label,tgt_reg,rpm,dur):
    print(f"\n[{label}] Target={rpm} rpm")
    t0=time.time()
    while time.time()-t0<dur:
        w_u16(client, tgt_reg, rpm & 0xFFFF, u)   # keepalive
        vL=r_i16(client, REG["ACT_VEL_L"], u)/10.0
        vR=r_i16(client, REG["ACT_VEL_R"], u)/10.0
        pL=r_i32(client, REG["POS_L_H"], REG["POS_L_L"], u)
        pR=r_i32(client, REG["POS_R_H"], REG["POS_R_L"], u)
        stat=r_u16(client, REG["STATUS_WORD"], u)
        print(f"   vL/vR={vL:.1f}/{vR:.1f} rpm | posL/posR={pL}/{pR} | STAT=0x{stat:04X}")
        time.sleep(0.3)
    w_u16(client, tgt_reg, 0, u)

def main():
    BAUD=CFG["BAUD"]; SLAVE=CFG["SLAVE"]; ACC=CFG["ACC_MS"]
    DUR=CFG["DUR"]; RPM_L=CFG["RPM_L"]; RPM_R=CFG["RPM_R"]; TMO=CFG["STO_TIMEOUT_S"]

    client,dev=open_client(BAUD)
    if not client: return
    try:
        bus=r_u16(client, REG["BUS_VOLT"], SLAVE)/100.0
        stat0=r_u16(client, REG["STATUS_WORD"], SLAVE)
        eL=r_u16(client, REG["ERR_L"], SLAVE); eR=r_u16(client, REG["ERR_R"], SLAVE)
        print(f"⚙️ Bus={bus:.2f}V | STAT=0x{stat0:04X} | Fault L/R=0x{eL:04X}/0x{eR:04X}")

        unlock_and_enable(client, SLAVE, ACC)
        print("🔓 Enable & unlock issued. STO/lock 비트가 내려갈 때까지 대기합니다...")
        if not wait_sto_clear(client, SLAVE, TMO):
            return  # STO 미해제시 종료

        # 축별 저속 동작
        if RPM_L: run_axis(client, SLAVE, "LEFT",  REG["TGT_VEL_L"], RPM_L, DUR)
        if RPM_R: run_axis(client, SLAVE, "RIGHT", REG["TGT_VEL_R"], RPM_R, DUR)

        statF=r_u16(client, REG["STATUS_WORD"], SLAVE)
        print(f"\nFinal STAT=0x{statF:04X}")
    except Exception as e:
        print(f"예외: {e}")
    finally:
        try:
            w_u16(client, REG["TGT_VEL_L"], 0, SLAVE)
            w_u16(client, REG["TGT_VEL_R"], 0, SLAVE)
            w_u16(client, REG["CONTROL_WORD"], 0x07, SLAVE)  # Stop
        except Exception: pass
        client.close(); print("🔚 종료")

if __name__=="__main__":
    main()
