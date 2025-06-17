/*
 * Copyright (c) 2025 Grovety Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "amode_proc.h"
#include "himax_control.h"
#include "tof_sensor_core.h"
#include "leds_ctrl.h"
#include "pwr_ctrl.h"
#include "detect_handler.h"
#include "mutex_wrapper.h"

#define LOG_MODULE_NAME amode_proc
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

static void amode_timer_handler(struct k_timer *dummy);
static void amode_timer_work_proc(struct k_work *work);

static K_TIMER_DEFINE(amode_timer, amode_timer_handler, NULL);
static K_WORK_DEFINE(amode_timer_work, amode_timer_work_proc);

#define AMODE_THREAD_STACKSIZE 1024
#define AMODE_THREAD_PRIORITY 7

static void amode_thread_proc();

static K_THREAD_DEFINE(amode_thread_id, AMODE_THREAD_STACKSIZE, amode_thread_proc, NULL, NULL, NULL, AMODE_THREAD_PRIORITY, 0, 0);
static K_EVENT_DEFINE(amode_rcgn_start_event);
static K_EVENT_DEFINE(amode_rcgn_finish_event);

K_MUTEX_DEF(amode_setup_mutex)

int amode_proc_init()
{
	K_MUTEX_INIT(amode_setup_mutex)
	return 0;
}

static void amode_setup_for_image_read()
{
	K_MUTEX_LOCK(amode_setup_mutex)
	// LOG_INF("amode_setup_for_image_read %d");
    tof_sensor_set_auto_mode(TOF_MODE_MEAS_DISTANCE_TIMER);
    himax_set_mode(HIMAX_MODE_CONT_WORK, HIMAX_ADV_READ_IMAGE);
	K_MUTEX_UNLOCK(amode_setup_mutex)
}

static void amode_setup_for_recognition()
{
	K_MUTEX_LOCK(amode_setup_mutex)
	// LOG_INF("amode_setup_for_recognition");
	tof_sensor_set_auto_mode(TOF_MODE_DISABLED_WHILE_RECOGN);
	himax_set_mode(HIMAX_MODE_CONT_WORK, HIMAX_ADV_DO_NOTHING);
	K_MUTEX_UNLOCK(amode_setup_mutex)
}

static void amode_setup_for_not_target_waiting()
{
	K_MUTEX_LOCK(amode_setup_mutex)
	himax_set_mode(HIMAX_POFF_MODE, HIMAX_ADV_DO_NOTHING);
	tof_sensor_set_auto_mode(TOF_MODE_AUTO_WAIT_DISAPPERANCE);
	K_MUTEX_UNLOCK(amode_setup_mutex)
}

static void amode_setup_for_is_target_waiting(bool himax_ctrl)
{
	K_MUTEX_LOCK(amode_setup_mutex)
	if (himax_ctrl)
		himax_set_mode(HIMAX_POFF_MODE, HIMAX_ADV_DO_NOTHING);
	tof_sensor_set_auto_mode(TOF_MODE_AUTO_WAIT_FOR_OBJECTS);
	K_MUTEX_UNLOCK(amode_setup_mutex)
}

void amode_image_read_notify(bool start)
{
	k_event_set(&amode_rcgn_start_event, start ? 2 : 4);
}

static unsigned asleep_cntr;

void amode_timer_cmd_recv_notify()
{
	K_MUTEX_LOCK(amode_setup_mutex)
	asleep_cntr = AUTO_MODE_TIMER_ASLEEP_PERIOD_SEC;
	k_timer_start(&amode_timer, K_MSEC(1000), K_MSEC(1000));
	K_MUTEX_UNLOCK(amode_setup_mutex)
}

static void amode_timer_handler(struct k_timer *dummy)
{
	k_work_submit(&amode_timer_work);
}

static void amode_timer_work_proc(struct k_work *work)
{
	K_MUTEX_LOCK(amode_setup_mutex)
	if (asleep_cntr == 1) {
		// LOG_INF("!!!asleep_cntr %d", asleep_cntr);
		k_event_post(&amode_rcgn_start_event, 8);
	}
	if (asleep_cntr > 0) {		
		asleep_cntr -= 1;
	}
	//> LOG_INF("asleep_cntr %d,pwroff_cntr %d", asleep_cntr, pwroff_cntr);
	if (asleep_cntr == 0) {
		k_timer_stop(&amode_timer);
	}
	K_MUTEX_UNLOCK(amode_setup_mutex)
}

void amode_do_recognize()
{
    k_event_set(&amode_rcgn_start_event, 1);
}

static atomic_t recgn_dummy_iter = ATOMIC_INIT(AMODE_MAX_RCGN_DUMMY_ITER);
static atomic_t recgn_found_iter = ATOMIC_INIT(0);

void amode_recognize_results_update(bool found, bool permission)
{
    if (atomic_get(&recgn_dummy_iter) < AMODE_MAX_RCGN_DUMMY_ITER)
    {
        atomic_inc(&recgn_dummy_iter);

        if (permission) {
            atomic_set(&recgn_dummy_iter, AMODE_MAX_RCGN_DUMMY_ITER);
			k_event_set(&amode_rcgn_finish_event, 1);
            detect_handler();
        }
        else {
            if (found) {
                if (atomic_get(&recgn_found_iter) < AMODE_MAX_RCGN_FOUND_ITER) {
                    atomic_inc(&recgn_found_iter);
                }
                else {
                    atomic_set(&recgn_dummy_iter, AMODE_MAX_RCGN_DUMMY_ITER);
					k_event_set(&amode_rcgn_finish_event, 2);
                }
            }
        }
    }
	if (atomic_get(&recgn_dummy_iter) >= AMODE_MAX_RCGN_DUMMY_ITER) {
		k_event_set(&amode_rcgn_finish_event, 4);
	}
	//if (k_event_test(&amode_rcgn_finish_event, 7)) {
    //	LOG_INF("(%d,%d): %ld,%ld", found, permission, atomic_get(&recgn_dummy_iter), atomic_get(&recgn_found_iter));
	//}
}

static void gbox_rcgn_proc()
{
	atomic_set(&recgn_found_iter, 0);
	atomic_set(&recgn_dummy_iter, 0);
	amode_setup_for_recognition();
#if APPL_SCENARIO == 3
	if (k_event_wait(&amode_rcgn_finish_event, 0x07, true, K_MSEC(AMODE_RECGN_TOUT_MSEC)) == 0) {
		LOG_ERR("Recognition timeout");
	}
#elif APPL_SCENARIO == 2
	k_sleep(K_MSEC(5000));
#endif
}

static void amode_thread_proc()
{
	typedef enum {WAIT_FOR_RCGN = 0, WAIT_FOR_DISA, READ_JPG_PICT} amode_state_t;
	amode_state_t state = WAIT_FOR_RCGN;
	uint32_t event;

    while (1)
    {
		k_timeout_t tout = state == WAIT_FOR_DISA ? K_MSEC(1000) : K_FOREVER;
		event = k_event_wait(&amode_rcgn_start_event, 0xf, false, tout);

		// LOG_INF("state %d, event %08x", state, event);

		if (event & 2) {
			k_event_clear(&amode_rcgn_start_event, 0xf);
			amode_setup_for_image_read();
			state = READ_JPG_PICT;
		}
		else if (event & 8) {
			if (state == READ_JPG_PICT) {
				k_event_clear(&amode_rcgn_start_event, 8);
				amode_setup_for_is_target_waiting(true);
				state = WAIT_FOR_RCGN;
			}
		}

		if (state == WAIT_FOR_RCGN) {
			if (event & 1) {
				k_event_clear(&amode_rcgn_start_event, 1);
				gbox_rcgn_proc();
				amode_setup_for_not_target_waiting();
				state = WAIT_FOR_DISA;
			}
		}
		else if (state == WAIT_FOR_DISA) {
			if (event == 0) {
				amode_setup_for_is_target_waiting(false);
				state = WAIT_FOR_RCGN;
			}
			else if (event & 1) {
// #define STRESS_TEST
#ifdef STRESS_TEST
				static unsigned cnt = 0;
				if (++cnt >= 5) {
					cnt = 0;
					k_event_clear(&amode_rcgn_start_event, 1);
					amode_setup_for_is_target_waiting(false);
					state = WAIT_FOR_RCGN;
				}
				else
#endif // STRESS_TEST
				{
					k_event_clear(&amode_rcgn_start_event, 1);
					amode_setup_for_not_target_waiting();
				}
			}
		}
	}
}
