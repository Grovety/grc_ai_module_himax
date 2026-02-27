from enum import Enum


class AtCmd(str, Enum):
    """Enumeration of AT commands used to communicate with the Gatebox device."""
    # System
    CHECK = "AT"
    REBOOT = "AT^REBOOT"
    ECHO = "AT^ECHO"

    # Date/Time
    TIME_GET = "AT%TIME"
    TIME_SET = "AT%TIME"

    # Gatebox Control
    GATE_INTERVAL_GET = "AT%GATEINT"
    GATE_INTERVAL_SET = "AT%GATEINT"

    # Validation
    VALMASK_SET = "AT%VALMASK"

    # Himax Control
    MODE_WORK = "AT^MODE=WORK"
    MODE_PDOWN = "AT^MODE=PDOWN"
    MODE_DPD = "AT^MODE=DPD"
    MODE_OFF = "AT^MODE=PWROFF"

    # Image
    IMG_CAPTURE = "AT&IMAGE"
    IMG_READ_FIRST = "AT&RDIMG"
    IMG_READ_NEXT = "AT&RINEXT"

    # History (LRRP)
    HIST_READ_FIRST = "AT+LRREAD"
    HIST_READ_NEXT = "AT+LRRDNEXT"
    HIST_CLEAR = "AT+LRCLEAR"

    # Permitted List (LPRP)
    LIST_READ_FIRST = "AT+LPREAD"
    LIST_READ_NEXT = "AT+LPRDNEXT"
    LIST_ADD = "AT+LPADD"
    LIST_DEL = "AT+LPDEL"
    LIST_CLEAR = "AT+LPCLEAR"

    # ToF
    TOF_DIST = "AT&RDDIST"

    # Validation Masks
    VALMASK = "AT%VALMASK"

class ResponseStatus(str, Enum):
    """Enumeration of standard response strings from the device."""
    OK = "OK"
    ERROR = "ERROR"
    BAD_FORMAT = "BAD_FORMAT"
    BAD_PARAM = "BAD_PARAM"
    NOT_READY = "NRDY"
    BUSY = "BSEQ"
    READ_ERROR = "RERR"
    BUFFER_TOO_SMALL = "BUFFER_TOO_SMALL"

    OK_ADDED = "OK_ADDED"
    OK_CLEARED = "OK_CLEARED"
    ERROR_FULL = "ERROR_FULL"
    EMPTY = "EMPTY"
