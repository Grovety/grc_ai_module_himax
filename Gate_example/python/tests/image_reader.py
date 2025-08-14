import os
import getopt
import sys
import time
import numpy as np
from tkinter import *
from PIL import ImageTk, Image
from datetime import datetime
import atexit

WILD_CASE = False

if WILD_CASE == True:
	import sys
	import os
	sys.path.append(os.path.join(os.path.dirname(os.path.realpath(__file__)), "../../tfl_example/mobilenet_v1"))
	from animal_names_rus import animal_name_rus

from io import BytesIO
# from tof_control import RangeStatus
RangeStatus = ("RNG_VALID", "SIGM_FAIL", "SIGN_FAIL", "RN_CLIPPED", "OUT_BOUNDS", "HRDWR_FAIL", "RV_NO_WRAP", "WRAPT_FAIL",
               "PROCS_FAIL", "XTALK_FAIL", "R_SYNC_INT", "R_MERG_PLS", "LACK_OF_SG", "MIN_RN_FAIL", "INV_RNG_RES", "RNG_ST_NONE",)

write_proc = None
read_proc = None

def print_hex_buf(data):
    print(f"{len(data)}", end = ": ")
    for i in range(len(data)):
        print(f"{data[i]:02x}", end=" ")
    print("")

def set_time(): # 'OK\n', 'BAD_FORMAT\n', 'ERROR\n', 'BAD_PARAM\n'
    timestamp = datetime.fromtimestamp(time.time())
    str_date = str(timestamp)
    str_date = str_date[:str_date.find(".")]
    cmd = f"AT%TIME={str_date}\n"
    if not write_proc(cmd.encode()):
        return False
    ret = read_proc(3)
    if ret != b"OK\n":
        ret += read_proc(8)
        print(f"{cmd}: {ret}")
        return False
    return True

def clear_lrrp_list(): # 'OK\n'
    cmd = b"AT+LRCLEAR\n"
    if not write_proc(cmd):
        return False
    ret = read_proc(3)
    if ret != b"OK\n":
        print(f"{cmd}: {ret}")
        return False
    return True

def set_work_mode(): # 'OK\n', 'BAD_FORMAT\n', 'BAD_PARAM\n', 'BAD_MODE\n'
    cmd = b"AT^MODE=WORK\n"
    if not write_proc(cmd):
        return False
    ret = read_proc(3)
    if ret != b"OK\n":
        ret += read_proc(8)
        print(f"{cmd}: {ret}")
        return False
    return True

def take_jpeg_image(): # 'OK\n'
    cmd = b"AT&IMAGE\n"
    if not write_proc(cmd):
        return False
    ret = read_proc(3)
    if ret != b"OK\n":
        print(f"{cmd}: {ret}")
        return False
    return True

def read_jpeg_image(): # '#NNN..', '!NNN..', 'NRDY\n', 'BSEQ\n', 'RERR\n'
    JPEG_SIZE = 11520
    CHUNK_SIZE = 240
    MAX_FIRST_ITER = 30
    MAX_SECOND_ITER = JPEG_SIZE//CHUNK_SIZE-1
# prepare buffer
    img_buf = b""
    for iter in range(MAX_FIRST_ITER): # the old case was without while and with delay between take and read
# read first chunk
        cmd = b"AT&RDIMG\n"
        if not write_proc(cmd):
            return None
# read four bytes header
        ret = read_proc(4)
        if len(ret) < 4:
            print(f"length error-{cmd}: {ret}")
            return None
        if ret[0:4] == b'NRDY':
            read_proc(1) # read last '\n'
            ### print(f"{cmd}: {ret}")
            if iter < MAX_FIRST_ITER-1:
                continue
# without hymax - to continue working
            byte_io = BytesIO()
            image = Image.open(os.path.dirname(os.path.realpath(__file__)) + "/noready.jpg").save(byte_io, 'PNG')
            print("Image no ready")
            return byte_io.getbuffer()
        if ret[0] != ord('#') and ret[0] != ord('!'):
            print(f"header error-{cmd}: {ret}")
            return None
        break
# get payload length and payload itself
    dnum = int(ret[1:4])
    ret = read_proc(dnum)
    if len(ret) != dnum:
        print(f"buffer error-{cmd}:{dnum},{len(ret)}")
        return None
    img_buf += ret
    ### print_hex_buf(ret)
# read next chunks
    cmd = b"AT&RINEXT\n"
    for iter in range(MAX_SECOND_ITER): # maximum size 7200=240*30,first+29 next
        if not write_proc(cmd):
            return None
# read four bytes header
        ret = read_proc(4)
        if len(ret) < 4:
            print(f"length error-{cmd}: {ret}")
            return None
# check format and get payload length and last sign
        if ret[0] != ord('#') and ret[0] != ord('!'):
            print(f"header error-{cmd}: {ret}")
            return None
        dnum = int(ret[1:4])
        last = ret[0] == ord('!')
# read payload itself
        ret = read_proc(dnum)
        if len(ret) != dnum:
            print(f"buffer error-{cmd}:{dnum},{len(ret)}")
            return None
        ### print_hex_buf(ret)
        img_buf += ret
        if last:
            return img_buf
# if we are here it means too many iterations
    print(f"too many iterations: {iter}")
    return None

def clear_history(): # 'OK\n'
    cmd = b"AT+LRCLEAR\n"
    if not write_proc(cmd):
        return False
    ret = read_proc(3)
    if ret != b"OK\n":
        print(f"{cmd}: {ret}")
        return False
    return True

def read_str_answer(cmd):
    ret = ""
    if not write_proc(cmd):
        return ret
    while True:
        b = read_proc(1)
        if b == "":
            print(f"{cmd}: not answer")
            return None
        ret += b.decode("utf-8")
        if b == b"\n":
            break
    return ret

def read_lrrp_list():
    cmd = (b"AT+LRREAD=7\n", b"AT+LRRDNEXT=7\n")
    idx = 0
# prepare list
    lrrp_list = ""
# read first then next chunk
    while True:
        ret = read_str_answer(cmd[idx])
        if ret is None:
            return None
        lrrp_list += ret
        if ret == "\n": # it has alredy read
            return lrrp_list
        idx = 1

def format_lrrp_list(lrrp_list, print_all=True):
    lrrp_list = lrrp_list.replace(";", "\n")
    lrrp_list = lrrp_list.rstrip().split("\n")
    lrrp_list = list(filter(lambda s: s != "", lrrp_list))
    list_num = len(lrrp_list)
    print_num = list_num if print_all else (8 if list_num > 8 else list_num)
    print_str = ""
    for print_idx in range(print_num):
        print_str += f"{print_idx:<4}:  "
        print_str += lrrp_list[list_num-print_num+print_idx]
        print_str += "\n"
    return print_str

def read_distance():
    ret = read_str_answer(b"AT&RDDIST\n")
    if ret is None:
        return (None,None)
    return ret.rstrip().split(",") # (status, distance)

def read_lprp_list():
    global listbox_plist
    listbox_plist.delete(0, END)
    cmd = ("LPREAD", "LPRDNEXT")
    first = 0
    while True:
        ret = read_str_answer(f"AT+{cmd[first]}\n".encode())
        if ret is None:
            return None
        first = 1
        if ret == "\n":
            break
        lprp_list = ret.rstrip().split(",")
        for lprp in lprp_list:
            listbox_plist.insert(END, lprp)
    listbox_plist.update()

def update_lprp_list():
    global lprp_item
    cmd=f"AT+LPADD={lprp_item.get()}\n".encode()
    ret = read_str_answer(cmd) # todo handle error answer
    read_lprp_list()

update_state = 0

def get_gate_interval():
    global gate_interval
    gate_interval.set(read_str_answer(b"AT%GATEINT\n").rstrip())

def set_gate_interval():
    write_proc(f"AT%GATEINT={gate_interval.get()}\n".encode())
    ret = read_proc(3)
    if ret == b"OK\n":
        get_gate_interval()
    else:
        ret += read_proc(8)
        print(ret)

def image_reader(_init_proc, _close_proc, _write_proc, _read_proc, argv = None):

    iter_numb = None

    opts, args = getopt.getopt(argv, "i:", ["iter=",])
    for o,a in opts:
        if o in ("-i", "--iter"):
            iter_numb = int(a)

    global write_proc, read_proc
    write_proc = _write_proc
    read_proc = _read_proc

    def cleaup_proc():
        if _close_proc is not None:
            _close_proc()

    atexit.register(cleaup_proc)
    if _init_proc is not None:
        _init_proc()

    set_time()
    clear_lrrp_list()
    set_work_mode()

    iter_cntr = 0
    start_time = None
    elapsed = [None]*3
    elps_min = [sys.float_info.max] * 3
    elps_max = [sys.float_info.min] * 3

    def update_data():
        global root, label_image, listbox_rlist, label_iter, label_dist
        global show_image, show_rlist, show_dist
        nonlocal iter_numb, iter_cntr, start_time, elapsed, elps_min, elps_max
        global update_state

        try:
            if update_state == 0:
                if show_image.get():
                    take_jpeg_image()
                    img_buf = read_jpeg_image()
                    if img_buf is not None:
                        image = ImageTk.PhotoImage(Image.open(BytesIO(img_buf)))
                        label_image.configure(image=image)
                        label_image.image = image
                update_state = 1 if show_rlist.get() else 2 if show_dist.get() else 0
            elif update_state == 1:
                if show_rlist.get():
                    rcgn_list = read_lrrp_list()
                    if rcgn_list is not None:
                        rcgn_list_fmt = format_lrrp_list(rcgn_list)
                        rcgn_list_spl = rcgn_list_fmt.split("\n")[:-1]
                        listbox_rlist.delete(0,END)
                        for list_idx in range(len(rcgn_list_spl)):
                            if WILD_CASE == False: # gatebox
                                listbox_rlist.insert(0, rcgn_list_spl[list_idx])
                                if rcgn_list_spl[list_idx][-1] == 'Y':
                                    listbox_rlist.itemconfig(0, {'fg': 'green'})
                            else: # wild
                                rcn_res = rcgn_list_spl[list_idx].split(":")[1].split(",")
                                arg_max = rcn_res[0]
                                idx_max = int(rcn_res[1])
                                listbox_rlist.insert(0, animal_name_rus[idx_max] + f"({arg_max})")
                        listbox_rlist.update()
                update_state = 2 if show_dist.get() else 0 if show_image.get() else 1
            elif update_state == 2:
                if show_dist.get():
                    stat,dist = read_distance()
                    if stat is not None:
                        stat = RangeStatus[int(stat)]
                        color = "green" if stat == "RNG_VALID" else "red"
                        label_dist.configure(text=f"Distance (mm) {dist}\n({stat})", fg=color, font=("Arial", 14))
                update_state = 0 if show_image.get() else 1 if show_rlist.get() else 2

            if show_image.get():
                if update_state == 0:
                    iter_cntr += 1
            elif show_rlist.get():
                if update_state == 1:
                    iter_cntr += 1
            elif show_dist.get():
                if update_state == 2:
                    iter_cntr += 1

            label_text=f"Iterations {iter_cntr}"
            if not iter_numb is None:
                if iter_numb == iter_cntr:
                    root.quit()
                else:
                    label_text += f" from {iter_numb}"
            else:
                if start_time is None:
                    start_time = time.time()
                else:
                    end_time = time.time()
                    delta_time = end_time - start_time
                    start_time = end_time
                    if elps_min[update_state] > delta_time: elps_min[update_state] = delta_time
                    if elps_max[update_state] < delta_time: elps_max[update_state] = delta_time
                    elapsed[update_state] = delta_time

                if not None in elapsed:
                    label_text += f" cur {elapsed[0]:.2f}, {elapsed[1]:.2f}, {elapsed[2]:.2f},"
                    label_text += f" min {elps_min[0]:.2f}, {elps_min[1]:.2f}, {elps_min[2]:.2f},"
                    label_text += f" max {elps_max[0]:.2f}, {elps_max[1]:.2f}, {elps_max[2]:.2f}"

            label_iter.configure(text=label_text, fg="blue", font=("Arial", 10))
            root.after(1, update_data)
        except Exception as ex:
            print(str(ex))

    def create_window():
        global root, label_image, listbox_rlist, listbox_plist, label_iter, label_dist
        global show_image, show_rlist, show_dist, read_plist, lprp_item, gate_interval
        root = Tk()
        root.title("Image Viewer")
        #root.geometry("800x600")

        show_image = IntVar()
        show_image.set(1)
        checkbutton_show_image = Checkbutton(text="Update camera", variable=show_image)
        checkbutton_show_image.grid(row=0, column=0)

        show_rlist = IntVar()
        show_rlist.set(1)
        checkbutton_show_rlist = Checkbutton(text="Update history", variable=show_rlist)
        checkbutton_show_rlist.grid(row=0, column=1)

        button_read_plist = Button(text="Read permitted list", command=read_lprp_list)
        button_read_plist.grid(row=0, column=2, padx=5, pady = 1)

        image = ImageTk.PhotoImage(Image.new('RGB', (320, 240), np.random.randint(0,255)))
        label_image = Label(image=image, width=320, height=240, borderwidth=10, relief="sunken")
        label_image.grid(row=1, column=0, padx=5, pady=0)

        listbox_rlist = Listbox(width=35, height=15)
        listbox_rlist.grid(row=1, column=1, padx=5)

        listbox_plist = Listbox(width=25, height=15)
        listbox_plist.grid(row=1, column=2, padx=5)

        lprp_item = StringVar()
        lprp_item.set("")
        plist_added = Entry(textvariable=lprp_item)
        plist_added.grid(row=2, column=2, padx=5)

        button_update_plist = Button(text="Add to permitted list", command=update_lprp_list)
        button_update_plist.grid(row=3, column=2, padx=5, sticky='n')

        show_dist = IntVar()
        show_dist.set(1)
        checkbutton_show_dist = Checkbutton(text="Update distance", variable=show_dist)
        checkbutton_show_dist.grid(row=2, column=0)

        label_dist = Label(text="...")
        label_dist.grid(row=3, column=0, padx=0, pady=10)

        label_iter = Label(text="")
        label_iter.grid(row=4, column=0, columnspan=2, padx=0, pady=0, sticky="nw")

        button_exit = Button(root, text="Clear history", command=clear_history, width=20)
        button_exit.grid(row=2, column=1, pady=10)

        button_exit = Button(root, text="Exit", command=root.quit, width=20)
        button_exit.grid(row=3, column=1, pady=10 )

        gate_interval = StringVar()
        gate_interval_val = Spinbox(root, from_=10, to=3600, textvariable=gate_interval, width=10)
        gate_interval_val.grid(row=4, column=1, sticky='e')
        get_gate_interval()
        Button(root, text="Gate interval", command=set_gate_interval, width=20).grid(row=4, column=2)

        root.after(10, update_data)
        root.mainloop()

    create_window()