import numpy as np
from tkinter import *
from tkinter import ttk
import sys
import atexit
from threading import Thread, Lock
sys.path.append("./tests")
from cmd_helper import tof

global init_proc, close_proc, write_proc, read_proc

mutex = Lock()

def send(cmd, showcmd=False):
    if showcmd:
        print(cmd)
    write_proc(cmd)

def recv(showresp=False):
    resp = b""
    while True:
        c = read_proc(1)
        resp += c
        if c == b"":
            break
        if c == b"\n":
            break
    if showresp:
        print(resp)
    return resp

def himax_get_mode():
    global himax_mode, himax_status_lbl
    send(b"AT^MODE\n")
    resp = recv()
    himax_mode = resp.decode('utf-8')
    himax_status_lbl.config(text=f"Himax mode\n{himax_mode}", bg="dodgerblue" if himax_mode != "WORK\n" else "red")

def hm_pwr_ctrl():
    if himax_mode == "WORK\n":
        send(b"AT^MODE=PWROFF\n")
    else:
        send(b"AT^MODE=WORK\n")
    resp = recv()

tof_modes = (b"AUTO+\n", b"AUTO-\n", b"INTER\n", b"INTZ\n", b"TIMER\n", b"TMRZ\n", b"OFF\n")

def tof_get_mode():
    global tof_mode_rb_chosen
    with mutex:
        tof_mode_rb_chosen.set(-1)
        send(b"AT&SMODE\n")
        resp = recv()
        for i in range(len(tof_modes)):
            if resp == tof_modes[i]:
                tof_mode_rb_chosen.set(i)
                return True
        return False

def tof_set_mode():
    global tof_mode_rb_chosen
    with mutex:
        send(b"AT&SMODE="+tof_modes[tof_mode_rb_chosen.get()])
        resp = recv()

bw_show_status = False
def show_bigw_status():
    global bw_show_status
    bw_show_status = not bw_show_status

zn_show_status = False
def show_zone_status():
    global zn_show_status
    zn_show_status = not zn_show_status

RangeStatus = (
    "RNG_VALID",    # 0,RANGE_VALID-Ranging measurement is valid.
    "SIGM_FAIL",    # 1,SIGMA_FAIL-Raised if sigma estimator check is above the internal defined threshold.
    "SIGN_FAIL",    # 2,SIGNAL_FAIL-Raised in autonomous mode only if signal value is below the internally defined threshold.
    "RN_CLIPPED",   # 3,RANGE_VALID_MIN_RANG_E_CLIPPED-Raised in autonomous mode only if ranging is below a threshold defined in the tuning parameter.
    "OUT_BOUNDS",   # 4,OUTOFBOUNDS_FAIL-Raised when the range result is out of bounds.
    "HRDWR_FAIL",   # 5,HARDWARE_FAIL-Raised in case of hardware or VCSEL failure.
    "RV_NO_WRAP",   # 6,RANGE_VALID_NO_WRAP-No wraparound check has been done.
    "WRAPT_FAIL",   # 7,WRAP_TARGET_FAIL-Wrapped target not matching phases.
    "PROCS_FAIL",   # 8,PROCESSING_FAIL-Internal algorithm underflow or overflow in autonomous ranging.
    "XTALK_FAIL",   # 9,XTALK_SIGNAL_FAIL-Raised in autonomous modes when the signal is below the crosstalk threshold.
    "R_SYNC_INT",   # 10,RANGESTATUS_SYNCRONISATION_INT-Raised once after start, ranging value has to be ignored.
    "R_MERG_PLS",   # 11,RANGE_VALID_MERGED_PULSE-Raised in ranging and multizone scanning modes only. Range is OK, but the distance reported is the result of multiple targets merged together.
    "LACK_OF_SG",   # 12,TARGET_PRESENT_LACK_OF_SIGNAL-Indicates that there is a target, but the signal is too low to report ranging.
    "MIN_RN_FAIL",  # 13,MIN_RANGE_FAIL-Programmed ROI is not valid, selected ROI is out of the SPAD array.
    "INV_RNG_RES",  # 14,INVALID-The reported range is invalid, ignore it.
    "RNG_ST_NONE"   # 15,RANGESTATUS_NONE-Single ranging value: ranging is not updated, ignore reported value. Multiple ranging values: target not detected.
)

def tof_get_bigw_data():
    with mutex:
        send(b"AT&RDDIST\n", False)
        resp = recv(False).decode("utf-8")
    par = resp.rstrip().split(",")
    val = par[1]
    stat_index = int(par[0])
    stat = "" if bw_show_status == False else (f"\n{RangeStatus[stat_index].lower()}" if stat_index < 16 else f"\n{stat_index}")
    inf = f"{val}{stat}"
    fg = 'green' if stat_index == 0 else "red"
    btn_bigw.configure(text=inf, fg=fg, bg="bisque2", font=('Helvetica', '16' if bw_show_status else '20'))

def tof_get_zone_data():
    with mutex:
        send(b"AT&RDZONE\n", False)
        resp = recv(False).decode("utf-8")
    if resp == "NOT_SUPPORTED\n": # the simplest version das not support multizone measurements
        for i in range(16):
            btn_zone[3-(i//4)][i%4].configure(text="?", bg="cornsilk2", fg="gray", font=('Helvetica', '16' if zn_show_status else '20'))
        return
    resp = resp.rstrip().split(";")
    if len(resp) != 16:
        print(f"Get zone data failed: {resp}")
        return
    for i in range(16):
        par = resp[i].split(",")
        val = par[1]
        stat_index = int(par[0])
        stat = "" if zn_show_status == False else (f"\n{RangeStatus[stat_index].lower()}" if stat_index < 16 else f"\n{stat_index}")
        inf = f"{val}\n{stat}"
        fg = 'green' if stat_index == 0 else "red"
        btn_zone[3-(i//4)][i%4].configure(text=inf, bg="cornsilk2", fg = fg, font=('Helvetica', '16' if zn_show_status else '20'))

get_data_state = 0;
def dev_get_data():
    global root, get_data_state
    if get_data_state == 0:
        tof_get_bigw_data()
        himax_get_mode()
        tof_get_mode()
        get_data_state = 1
    else:
        tof_get_zone_data()
        get_data_state = 0
    root.after(10, dev_get_data)

def tof_get_param():
    global tof_param
    with mutex:
        send(b"AT&SPARAM\n")
        resp = recv().decode("utf-8")
        par = resp.rstrip().split(";")
    dist = par[0].split(",")
    tof_param["thresh_dist_low"].set(dist[0])
    tof_param["thresh_dist_high"].set(dist[1])
    tmbdgt = par[2].split(",")
    tof_param["time_budget_common"].set(tmbdgt[0])
    tof_param["time_budget_amode"].set(tmbdgt[1])

def tof_set_distance():
    global tof_param
    with mutex:
        send(f"AT&DIST={tof_param['thresh_dist_low'].get()},{tof_param['thresh_dist_high'].get()}\n".encode())
        resp = recv()

def tof_set_time_budget():
    global tof_param
    with mutex:
        send(f"AT&TMBD={tof_param['time_budget_common'].get()},{tof_param['time_budget_amode'].get()}\n".encode())
        resp = recv()

def tof_set_param():
    tof_set_distance()
    tof_set_time_budget()
    tof_set_mode()
    tof_get_param()

def on_exit():
    global close_proc
    if not close_proc is None:
        close_proc()

def tof_control(init_proc, _close_proc, _write_proc, _read_proc, argv = None):

    global root, btn_zone, btn_bigw, tof_param
    global close_proc, write_proc, read_proc
    global tof_mode_rb_chosen, himax_status_lbl

    close_proc = _close_proc
    write_proc = _write_proc
    read_proc = _read_proc

    if not init_proc is None:
        init_proc()

    atexit.register(on_exit)

    root = Tk()
    root.title("TOF control")
    root.geometry("480x640")

    himax_status_lbl = Label(text="", bg="dodgerblue", width=10, padx=5)
    himax_status_lbl.grid(row=0, column=3, rowspan=3)

    tof_param = {"thresh_dist_low": StringVar(value="?"),
                 "thresh_dist_high": StringVar(value="?"),
                 "time_budget_common" : StringVar(value="?"),
                 "time_budget_amode" : StringVar(value="?")}
    Label(text="", width=15).grid(row=11, column=1)
    Label(text=" Dist.threshold low,mm:", width=25).grid(row=11, column=2, sticky="e")
    Spinbox(from_=1, to=8000, textvariable=tof_param["thresh_dist_low"], width=10).grid(row=11, column=3)
    Label(text=" Dist.threshold high,mm:", width=25).grid(row=12, column=2, sticky="e")
    Spinbox(from_=1, to=8000, textvariable=tof_param["thresh_dist_high"], width=10).grid(row=12, column=3)
    Label(text="", width=15).grid(row=13, column=0)
    Label(text="Tm budget common,us:", width=20).grid(row=11, column=0, sticky="e")
    Spinbox(from_=20000, to=1000000, textvariable=tof_param["time_budget_common"], width=10).grid(row=11, column=1)
    Label(text="Tm budget amode,us:", width=20).grid(row=12, column=0, sticky="e")
    Spinbox(from_=20000, to=1000000, textvariable=tof_param["time_budget_amode"], width=10).grid(row=12, column=1)
    Button(text=f"Set all", width=20, command=tof_set_param).grid(row=13, column=1, columnspan=2, padx=1, pady=8)
    Button(text=f"Himax pwr on/off", width=20, command=hm_pwr_ctrl).grid(row=13, column=3, columnspan=2, padx=1, pady=8)

    tof_get_param()

    tof_mode_rb_chosen = IntVar()
    if tof_get_mode() == False:
        print("Get TOF mode operation failed")
    chkb_names = ("auto-is target", "auto-no target", "interrupt bigw", "interrupt zone", "timer-bigw", "timer-zone", "disabled")
    color_list = (None, None, "bisque2", "cornsilk2", "bisque2", "cornsilk2", None)
    chkb_value = np.arange(len(chkb_names))
    for i in chkb_value:
        rb = Radiobutton(text=chkb_names[i], bg=color_list[i], variable=tof_mode_rb_chosen, value=chkb_value[i], command=tof_set_mode)
        rb.grid(row = i, column=0, sticky="w")

    for r in range(12): root.rowconfigure(index=r, weight=1)
    for c in range(4): root.columnconfigure(index=c, weight=1)

    btn_bigw = Button(text=f"...", font=('Helvetica', '16'), width=20, command=show_bigw_status)
    btn_bigw.grid(row=1, column=1, columnspan=2, rowspan=5, padx=20, sticky='nsew')

    btn_zone = [[[None] for i in range(4)] for j in range(4)]
    for r in range(7,11):
        for c in range(4):
            btn_zone[r-7][c] = Button(text=f"({4-r+4},{c})", font=('Helvetica', '16'), width=16, command=show_zone_status)
            btn_zone[r-7][c].grid(row=r, column=c, padx=1, pady=1, ipadx=10, ipady=10)

    root.after(10, dev_get_data)

    tof_get_zone_data()

    root.mainloop()