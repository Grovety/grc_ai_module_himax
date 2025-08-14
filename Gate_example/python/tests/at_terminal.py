import time
import sys
import atexit
from pynput import keyboard # instead of keyboard due to a need being root
from pynput.keyboard import Key,Listener
import threading
import getopt
import random

from cmd_helper import *

write_proc = None
read_proc = None

def send_cmd(cmd, print_data = True):
    global write_proc
    cmd += "\n"
    cmd = command_helper(cmd.encode())
    if print_data:
        print(cmd)
    write_proc(cmd)

def read_ans(print_data = True):
    global read_proc
    answ = b""
    while True:
        char = read_proc(1)
        if char == b"":
            if print_data:
                print("Not answer!!!")
            break
        answ += char
        if char == b"\n":
            break
    if print_data:
        print(answ)
    return answ.decode("utf-8")

user_break = False

def user_break_tread():
    global user_break
    while True:
        with keyboard.Events() as events:
            for event in events:
                if event.key == keyboard.Key.space:
                    user_break = True
                    return

def user_break_init():
    global user_break
    user_break = False
    thread = threading.Thread(target=user_break_tread)
    thread.start()

def wait_proc(pause):
    global user_break
    ret = True
    inf = ("-", "/", "|", "\\")
    infn = 0
    pause *= 10
    for f in range(pause):
        if user_break:
            print("\r", end = "")
            ret = False
            break
        time.sleep(0.1)
        print(f"\r{inf[infn]}", end = " (press Space to stop)")
        infn = (infn + 1) & 3
    print("\r                        ")
    return ret

def clear_lprp_list():
    send_cmd("c")
    read_ans()

lprp_list_ref = []          # reference list,must have the same items as on device,but the order can be different
lprp_list_chk = []          # it is read from device,should be compared to reference
lprp_list_max_len = None    # the depth of the lists,it defines by reading from device and actual always

def compare_lprp_lists():
    l = (len(lprp_list_ref), len(lprp_list_chk))            # check length first
    if l[0] != l[1]:
        print(f"Bad length! {l[0]},{l[1]}")
        return False
    for i in range(l[0]):                                   # for each item it the first list
        found = False
        for j in range(l[1]):                               # find the same in the second list
            if lprp_list_ref[i] == lprp_list_chk[j]:
                found = True
                break
        if found == False:
            print(f"Items {i} are different!\n{lprp_list_ref}\n{lprp_list_chk}")
            return False
    print(f"Lists({l[0],l[1]}) are the same")
    return True

def add_item_to_lprp_list(plate):
    send_cmd(f"AT+LPADD={plate}", False)
    return read_ans(False)

def delete_item_from_lprp_list(plate):
    send_cmd(f"AT+LPDEL={plate}", False)
    return read_ans(False)

def fill_lprp_list_ref(seqn = True, size = None):
    global lprp_list_ref, lprp_list_max_len
    # clear_lprp_list()
    cnt = 0
    print("lprp list is filling,wait a little..")
    while True:
        if (size is not None) and (cnt >= size):
            break
        plate = car_reg_plate_gene(seqn)
        ret = add_item_to_lprp_list(plate)
        if ret != "OK\n":
            break
        cnt += 1
        lprp_list_ref.append(plate)
        lprp_list_max_len = cnt
    print(f"{ret.strip()}: {cnt} items has written")

def update_lprp_list_chk(show_list = False):
    global lprp_list_chk
    lprp_list_chk.clear()
    cmd = ("r", "n")
    first = 0
    while True:
        send_cmd(cmd[first], False)
        first = 1
        ret = read_ans(False)
        if ret == "\n":
            break
        ret = ret.strip().split(",")
        for r in ret:
            lprp_list_chk.append(r)
    if show_list:
        print(lprp_list_chk)

def test_lprp_list():
    global lprp_list_ref, lprp_list_chk, lprp_list_max_len
    clear_lprp_list()               # clear list
    fill_lprp_list_ref()            # fill list
    update_lprp_list_chk()          # upload list
    compare_lprp_lists()            # compare list
    user_break_init()
    while (1):
        optype = random.randint(0,1)
        if optype == 0:
            del_num = random.randint(0,lprp_list_max_len)
            for i in range(del_num):                    # if list is empty need to skip delete
                if len(lprp_list_ref) == 0:
                    break
                item_index_for_delete = random.randint(0, len(lprp_list_ref)-1)
                plate = lprp_list_ref[item_index_for_delete]
                ret = delete_item_from_lprp_list(plate) # delete on the device
                if ret != "OK\n":
                    print(f"{ret}: {plate} items can not be deleted")
                    return
                update_lprp_list_chk()                  # update data from device
                lprp_list_ref.remove(plate)             # delete from reference
                print(f"{plate} has removed")
                if compare_lprp_lists() == False:       # check for the same
                    return
        else:
            ins_num = random.randint(0, lprp_list_max_len)
            for i in range(ins_num):                    # if list is full,need to skip write
                if lprp_list_max_len == len(lprp_list_ref):
                    break
                plate = car_reg_plate_gene(True)
                ret = add_item_to_lprp_list(plate)      # insert item on the device
                if ret != "OK\n":
                    print(f"{ret}: {plate} items can not be insert")
                    return
                update_lprp_list_chk()                  # update data from device
                lprp_list_ref.append(plate)             # append to reference
                print(f"{plate} has added")
                if compare_lprp_lists() == False:       # check for the same
                    return
        if user_break:
            return

def clear_lrrp_list():
    send_cmd("C")
    read_ans()

def fill_lrrp_list(lrrp_size = 64):
    clear_lrrp_list()
    for i in range(lrrp_size):
        send_cmd("A")
        read_ans()

def set_curr_time():
    send_cmd("T")
    read_ans()

def read_lrrp_list_once():
    list = ""
    send_cmd("R", False)
    ret = read_ans(False)
    list += ret.replace(";", "\n")
    if ret != "\n":
        while True:
            send_cmd("N", False)
            ret = read_ans(False)
            list += ret.replace(";", "\n")
            if ret== "\n":
                break
    print(list)
    return True

def read_lrrp_list_loop():
    global user_break
    user_break_init()
    while True:
        if read_lrrp_list_once() == False:
            return
        if user_break:
            return
        time.sleep(0.01)

def rep_cmd(cmd_seq, start_idx = 0, end_idx = 0):

    def send(cmd):
        if isinstance(cmd, tuple):          # tuple of commands
            for idx in range(len(cmd)):
                send_cmd(cmd[idx])
                read_ans()
        else:                               # single command
            send_cmd(cmd)
            read_ans()

    cmd_idx = start_idx
    send(cmd_seq[start_idx])
    user_break_init()
    while True:
        if not wait_proc(5 + random.randint(0,5)):
            break
        cmd_idx = (cmd_idx + 1) % len(cmd_seq)
        send(cmd_seq[cmd_idx])
    send(cmd_seq[end_idx])

def power_down_test(rcgn_imit):
    if rcgn_imit:
        rep_cmd((("!w", "~1",), "!p", ("!w", "~1",), "!d",))
    else:
        rep_cmd(("!w", "!p", "!w", "!d",))

def rcgn_imit_test():
    rep_cmd(("~1", "~0"), 0, 1)

def send_garbage():
    dlenmax = 244 # BT payload,for serial connection can be more
    dlen = random.randint(1,dlenmax)
    data = [random.randint(1,255) for _ in range(dlen)]
    print(f"{dlen} bytes are sending..")
    write_proc(bytearray(data))

def stress_test():
    while True:
        send_garbage()
        user_break_init()
        if not wait_proc(1):
            return
        for _ in range(2):
            send_cmd("AT")
            read_ans()

def nrf_sys_pwroff():
    send_cmd("AT^LOWPWR=0")
    read_ans()

def error_stack_read_all():
    while True:
        send_cmd("AT?ERR")
        ret = read_ans()
        if ret == "0\n" or ret == "":
            break

def prompt():
    print(f"Enter command(h,q,a,f,d,r,c,z,A,F,R,G,N,C,L,P,t,T,!m,!w,!p,!d,!o,!n,!t,!i,~1,~0,~t,@m,@d,@t,@T,@i,@a,@A,^g,^s,*o,?e or AT.. ):")

def help():
# common commands
    print("h-get help")
    print("q-exit")
# commands to manage lprp list
    print("a-add random number to the list of permitted registration plates")
    print("f-fill the list of permitted registration plates with sequential plates")
    print("d-delete random selected number from the list of permitted registration plates")
    print("r-print the content of the list of permitted registration plates")
    print("c-clear the content of the list of permitted registration plates")
    print("z-test for the list of permitted registration plates")
# commands to manage lrrp list (car's history)
    print("A-add random number to the list of recognized registration plates")
    print("F-fill the list of recognized registration plates with generated plates")
    print("R-read first seven items from the list of recognized registration plates")
    print("G-read first seven items after random chosen date from the list of recognized registration plates")
    print("N-read next seven items from the list of recognized registration plates")
    print("C-clear the content of the list of recognized registration plates")
    print("L-read whole content of list of recognized registration plates")
    print("P-read whole content of list of recognized registration plates repetively (Esc-stop)")
# date-time commands
    print("t-get internal time from device")
    print("T-set internal device time to current time")
# Himax mode control
    print("!m-get Himax current work mode")
    print("!w-put Himax to work mode")
    print("!p-put Himax to power done mode")
    print("!d-put Himax to deep down mode")
    print("!o-put Himax to power off mode (not fully implemented on the board yet)")
    print("!n-put Himax to one-shot mode from PD or DPD modes")
    print("!t-test for sequential PD and DPD modes")
    print("!i-test for sequential PD and DPD modes with recognize imitation")
    print("~0-turn off the imitation of the recognizing")
    print("~1-turn on the imitation of the recognizing")
    print("~t-test the imitation of the recognizing")
    print("@m-get TOF sensor current mode")
    print("@t-read TOF sensor by timer,big window")
    print("@T-read TOF sensor by timer,multizone mode")
    print("@i-read TOF sensor by interrupt")
    print("@a-autonomous mode of TOF sensor,interrupt happends when target has appeared")
    print("@A-autonomous mode of TOF sensor,interrupt happends when target has disappeared")
    print("@d-read distance measured by TOF sensor")
    print("^g-send garbage to device,i mean random data instead of particular command")
    print("^s-stress test-send random data then two valid commands,the second must have correct answer")
    print("*o-put nRF to system power off mode,then caugth answer,close port and return from script")
    print("?e-read error stack")

def term(first_cmd=None):
    sys.stdout.flush()
    while True:
        if first_cmd is None:
            prompt()
            cmd = input()
            cmd = cmd.lstrip()
        else:
            cmd = first_cmd
            first_cmd = None
        print(cmd)
        if cmd == "q":
            return
# verbose help information
        if cmd == 'h':
            help()
            continue
# fill lprp list
        if cmd == "f":
            fill_lprp_list_ref()
            continue
# read lprp list
        if cmd == "r":
            update_lprp_list_chk(True)
            continue
# test for lprp list
        if cmd == "z":
            test_lprp_list()
            continue
# fill lrrp list
        if cmd == "F":
            fill_lrrp_list()
            continue
# read lrrp list only once
        if cmd == "L":
            read_lrrp_list_once()
            continue
# read lrrp list until key pressed
        if cmd == "P":
            read_lrrp_list_loop()
            continue
# power modes switching test
        if cmd == "!t":
            power_down_test(False)
            continue
        if cmd == "!i":
            power_down_test(True)
            continue
# recognize imitation test
        if cmd == "~t":
            rcgn_imit_test()
            continue
# send garbage for stress test
        if cmd == "^g":
            send_garbage()
            continue
# simplest stress test
        if cmd == "^s":
            stress_test()
            continue
# nrf system power off mode
        if cmd == "*o":
            nrf_sys_pwroff()
            time.sleep(0.5)
            break
#read error stack
        if cmd == "?e":
            error_stack_read_all()
            continue
        send_cmd(cmd)
        read_ans()

def keyup(e):
    print('up', e.char)

def keydown(e):
    global term_win
    print('down', e.char)

def at_terminal(init, close, write, read, argv):

    first_cmd = None

    opts, args = getopt.getopt(argv, "c:", ["cmd=",])
    for o,a in opts:
        if o in ("-c", "--cmd"):
            first_cmd = a
        print(o,a)

    global write_proc, read_proc

    write_proc = write
    read_proc = read

    if close is not None:
        atexit.register(close)

    if init is not None:
        init()

    term(first_cmd)
