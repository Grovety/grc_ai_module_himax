import random
import time
from datetime import datetime

plate_idx = 0

def car_reg_plate_gene(plate_seq_num = False):
    global plate_idx
    aval_lett =  ("A", "B", "E", "K", "M", "H", "O", "P", "C", "T", "Y", "X")
    aval_mask = ("cnnnccnn", "cnnnccnnn")
    leng = random.randint(len(aval_mask[0]), len(aval_mask[1]))
    mask = aval_mask[leng-8]
    reg_plate = ""
    for i in range(leng):
        if mask[i] == "c":
            reg_plate += aval_lett[random.randint(0, len(aval_lett)-1)]
        else:
            if plate_seq_num and i == 1:
                reg_plate += f"{(plate_idx % 1000):03d}"
                plate_idx += 1
            elif not plate_seq_num or i > 3:
                reg_plate += (chr(0x30 + random.randint(0, 9)))
    return reg_plate

class lprp:

    list_to_save = []

    @staticmethod
    def add():
        plate = car_reg_plate_gene(True)
        lprp.list_to_save.append(plate)
        cmd = "AT+LPADD=" + plate + "\n"
        return cmd.encode()

    @staticmethod
    def delete():
        if len(lprp.list_to_save) > 0:
            index = random.randint(0, len(lprp.list_to_save)-1)
            plate = lprp.list_to_save[index]
            lprp.list_to_save.remove(plate)
        else:
            plate = "X000TT000"
        cmd = "AT+LPDEL=" + plate + "\n"
        return cmd.encode()

    @staticmethod
    def read():
        cmd = "AT+LPREAD\n"
        return cmd.encode()

    @staticmethod
    def next():
        cmd = "AT+LPRDNEXT\n"
        return cmd.encode()

    @staticmethod
    def clear():
        lprp.list_to_save.clear()
        cmd = "AT+LPCLEAR\n"
        return cmd.encode()

class lrrp:

    list_to_save = []

    @staticmethod   # AT+LRREAD=<count>\n
    def read():
        cmd = "AT+LRREAD=7\n"
        return cmd.encode()

    @staticmethod   # AT+LRREAD=<count>,<date>\n
    def read_from_date():
        list_len = len(lrrp.list_to_save)
        if list_len:
            item_idx = random.randint(0, list_len-1)
            timestamp = lrrp.list_to_save[item_idx][1]
        else:
            timestamp = datetime.fromtimestamp(time.time() - (60*60*24))
        str_date = str(timestamp)
        str_date = str_date[:str_date.find(".")]
        print(f"Recognizing results from {str_date}")
        cmd = f"AT+LRREAD=7,{str_date}\n"
        return cmd.encode()

    @staticmethod   # AT+LRRDNEXT=<count>\n
    def next():
        cmd = "AT+LRRDNEXT=7\n"
        return cmd.encode()

    @staticmethod   # AT+LRCLEAR\n
    def clear():
        lrrp.ts = None
        cmd = "AT+LRCLEAR\n"
        return cmd.encode()

    ts = None

    @staticmethod   # AT+LRADD=<plate>,<data>,<perm>\n
    def add():
        if lrrp.ts is None:
            lrrp.ts = time.time()
        else:
            lrrp.ts += random.randint(0, 60*60*24*20)
        plate = car_reg_plate_gene(True)
        timestamp = datetime.fromtimestamp(lrrp.ts)
        str_date = str(timestamp)
        str_date = str_date[:str_date.find(".")]
        perm = ("Y","N")[random.randint(0,1)]
        lrrp.list_to_save.append((plate, timestamp, perm,))
        cmd = f"AT+LRADD={plate},{str_date},{perm}\n"
        return cmd.encode()

class dttm:

    @staticmethod # AT%TIME=<date-time>\n
    def set():
        timestamp = datetime.fromtimestamp(time.time())
        str_date = str(timestamp)
        str_date = str_date[:str_date.find(".")]
        cmd = f"AT%TIME={str_date}\n"
        return cmd.encode()

    @staticmethod # AT%TIME
    def get():
        cmd = "AT%TIME\n"
        return cmd.encode()

class himax:

    @staticmethod # AT^MODE\n
    def get_mode():
        cmd = "AT^MODE\n"
        return cmd.encode()

    @staticmethod # AT^MODE=WORK\n
    def work_mode():
        cmd = "AT^MODE=WORK\n"
        return cmd.encode()

    @staticmethod # AT^MODE=PDOWN\n
    def power_down():
        cmd = "AT^MODE=PDOWN\n"
        return cmd.encode()

    @staticmethod  # AT^MODE=DPD\n
    def deep_power_down():
        cmd = "AT^MODE=DPD\n"
        return cmd.encode()

    @staticmethod  # AT^MODE=PWROFF\n
    def power_off():
        cmd = "AT^MODE=PWROFF\n"
        return cmd.encode()

    @staticmethod  # AT^MODE=ONESHOT\n
    def one_shot():
        cmd = "AT^MODE=ONESHOT\n"
        return cmd.encode()

    @staticmethod  # AT^RCGNIMIT=1
    def rcgn_on():
        cmd = "AT^RCGNIMIT=1\n"
        return cmd.encode()

    @staticmethod  # AT^RCGNIMIT=1
    def rcgn_off():
        cmd = "AT^RCGNIMIT=0\n"
        return cmd.encode()

class tof:
    @staticmethod  # AT&RDDIST
    def read_dist():
        cmd = "AT&RDDIST\n"
        return cmd.encode()

    @staticmethod  # AT&RDZONE
    def read_zone():
        cmd = "AT&RDZONE\n"
        return cmd.encode()

    @staticmethod  # AT&SMODE
    def get_mode():
        cmd = "AT&SMODE\n"
        return cmd.encode()

    @staticmethod   # AT&SMODE=TIMER
    def timer_mode_bigw():
        cmd = "AT&SMODE=TIMER\n"
        return cmd.encode()

    @staticmethod   # AT&SMODE=TMRZ
    def timer_mode_zone():
        cmd = "AT&SMODE=TMRZ\n"
        return cmd.encode()

    @staticmethod   # AT&SMODE=INTER
    def interrupt_mode():
        cmd = "AT&SMODE=INTER\n"
        return cmd.encode()

    @staticmethod   # AT&SMODE=AUTO+
    def auto_mode_is_target():
        cmd = "AT&SMODE=AUTO+\n"
        return cmd.encode()

    @staticmethod   # AT&SMODE=AUTO-
    def auto_mode_no_target():
        cmd = "AT&SMODE=AUTO-\n"
        return cmd.encode()

cmd_table = { b"a\n": lprp.add,           # add number
              b"d\n": lprp.delete,        # remove number
              b"r\n": lprp.read,          # read first
              b"n\n": lprp.next,          # read next
              b"c\n": lprp.clear,         # clear list
              b"A\n": lrrp.add,           # add number
              b"R\n": lrrp.read,          # read first without date
              b"G\n": lrrp.read_from_date,  # read first with date
              b"N\n": lrrp.next,          # read next
              b"C\n": lrrp.clear,         # clear list
              b"t\n": dttm.get,           # get date-time
              b"T\n": dttm.set,           # set date-time
              b"!m\n" : himax.get_mode,
              b"!w\n" : himax.work_mode,
              b"!p\n" : himax.power_down,
              b"!d\n" : himax.deep_power_down,
              b"!o\n" : himax.power_off,
              b"!n\n" : himax.one_shot,
              b"~1\n" : himax.rcgn_on,
              b"~0\n": himax.rcgn_off,
              b"@d\n": tof.read_dist,
              b"@z\n": tof.read_zone,
              b"@m\n": tof.get_mode,
              b"@t\n": tof.timer_mode_bigw,
              b"@T\n": tof.timer_mode_zone,
              b"@i\n": tof.interrupt_mode,
              b"@a\n": tof.auto_mode_is_target,
              b"@A\n": tof.auto_mode_no_target, }

def command_helper(data):
    data = data.replace(b"\r\n", b"\n")
    if cmd_table.get(data) is None:
        return data
    return cmd_table[data]()
