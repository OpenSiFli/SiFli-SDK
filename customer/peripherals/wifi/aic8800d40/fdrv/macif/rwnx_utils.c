/*
 * SPDX-FileCopyrightText: 2026 AICSemi Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "co_math.h"
#include "co_list.h"
#include "co_utils.h"
#include "co_endian.h"
#include "fhost.h"
#include "fhost_rx.h"
#include "fhost_tx.h"
#include "fhost_cntrl.h"
#include "rwnx_rx.h"
#include "rwnx_defs.h"
#include "rwnx_cmds.h"
#include "rwnx_utils.h"
#include "rwnx_msg_rx.h"
#include "netif_port.h"
#include "sys_al.h"
#include "aic_plat_log.h"
#include <string.h>

static int8_t rssi_saved[NX_REMOTE_STA_MAX] = {0x7F};
extern int wifi_ap_isolate;

/**
 * rwnx_rx_get_vif - Return pointer to the destination vif
 *
 * @rwnx_hw: main driver data
 * @vif_idx: vif index present in rx descriptor
 *
 * Select the vif that should receive this frame. Returns NULL if the destination
 * vif is not active or vif is not specified in the descriptor.
 */
static inline
struct fhost_vif_tag *rwnx_rx_get_vif(int vif_idx)
{
    struct fhost_vif_tag *fhost_vif = NULL;
    if (vif_idx < NX_VIRT_DEV_MAX) {
        fhost_vif = fhost_env.mac2fhost_vif[vif_idx];
    }
    return fhost_vif;
}

int8_t data_pkt_rssi_get(uint8_t *mac_addr)
{
    struct vif_info_tag *mac_vif = fhost_env.vif[0].mac_vif; // TODO:

    if (mac_vif && (VIF_AP == mac_vif->type)) {
        struct mac_addr mac;
        MAC_ADDR_CPY(mac.array, mac_addr);
        uint8_t staid = vif_mgmt_get_staid(mac_vif, &mac);
        if(staid < FMAC_REMOTE_STA_MAX) {
            return rssi_saved[staid];
        }
    }
    return 0x7F;
}
void data_pkt_rssi_set(uint8_t *addr, int8_t rssi)
{
    struct vif_info_tag *mac_vif = fhost_env.vif[0].mac_vif; // TODO:

    if (mac_vif && (VIF_AP == mac_vif->type)) {
        struct mac_addr mac;
        MAC_ADDR_CPY(mac.array, addr);
        uint8_t staid = vif_mgmt_get_staid(mac_vif, &mac);
        if(staid < FMAC_REMOTE_STA_MAX) {
            rssi_saved[staid] = rssi;
        }
    }
}

#if (AICWF_RWNX_TIMER_EN)
#define RWNX_TIMER_COUNT    64

struct rwnx_timer_tag {
    struct co_list free_list;
    struct co_list post_list;
    struct co_list stop_list;
    rtos_queue task_queue;
    rtos_mutex task_mutex;
    rtos_task_handle task_handle;
};

struct rwnx_timer_msg_s {
    rwnx_timer_handle hdl;
    enum rwnx_timer_action_e action;
};

struct rwnx_timer_node_s rwnx_timer_node_pool[RWNX_TIMER_COUNT];
struct rwnx_timer_tag rwnx_timer_env;

bool rwnx_timer_cmp(struct co_list_hdr const *elementA,
                    struct co_list_hdr const *elementB)
{
    const rwnx_timer_handle hdlA = (rwnx_timer_handle)elementA;
    const rwnx_timer_handle hdlB = (rwnx_timer_handle)elementB;
    bool ret = false;
    if ((int)(hdlA->expired_ms - hdlB->expired_ms) < 0) {
        ret = true;
    }
    return ret;
}

void rwnx_timer_task(void *arg)
{
    struct rwnx_timer_tag *timer_env = arg;
    rwnx_timer_handle timer_hdl = NULL;
    #ifndef CONFIG_RWNX_TIMER_TASK_PB
    struct co_list timer_hdl_list;
    co_list_init(&timer_hdl_list);
    #endif /* CONFIG_RWNX_TIMER_TASK_PB */
    int ret, next_to_ms = -1;
    uint32_t cur_time_ms;
    struct rwnx_timer_msg_s msg;
    #ifdef CONFIG_RWNX_TIMER_TASK_TS
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    static volatile uint8_t tmr_count = 0;
    #endif
    while (1) {
        DBG_TIMER("bef rd, next_to_ms=%d\n",next_to_ms);
        ret = rtos_queue_read(timer_env->task_queue, &msg, next_to_ms, false);
        #ifdef CONFIG_RWNX_TIMER_TASK_TS
        tmr_count++;
        start_time = rtos_now(0);
        aic_dbg("tmr_in:%u/%u\n", start_time, tmr_count);
        #endif
        timer_hdl = msg.hdl;
        cur_time_ms = rtos_now(false);
        DBG_TIMER("aft rd, timer_hdl=%p, ret=%x\n",timer_hdl,ret);
        DBG_TIMER("aft rd, action=%d, state=%x\n",msg.action,timer_hdl->state);
        rtos_mutex_lock(timer_env->task_mutex, -1);
        if (ret == AIC_RTOS_SUCCESS) { // create/delete/start/stop action
            if (msg.action == RWNX_TIMER_ACTION_CREATE) {
                if (timer_hdl->state == RWNX_TIMER_STATE_FREE) {
                    if (timer_hdl->auto_load) {
                        co_list_insert(&timer_env->post_list, &timer_hdl->hdr, rwnx_timer_cmp);
                        timer_hdl->state = RWNX_TIMER_STATE_POST;
                    } else {
                        //co_list_push_back(&timer_env->stop_list, &timer_hdl->hdr);
                        co_list_insert(&timer_env->stop_list, &timer_hdl->hdr, rwnx_timer_cmp);
                        timer_hdl->state = RWNX_TIMER_STATE_STOP;
                    }
                } else {
                    WRN_TIMER("rwnx_timer created but not free: [%p] %x\n", timer_hdl, timer_hdl->state);
                }
            } else if (msg.action == RWNX_TIMER_ACTION_START) {
                if (timer_hdl->state == RWNX_TIMER_STATE_STOP) {
                    co_list_extract(&timer_env->stop_list, &timer_hdl->hdr);
                    co_list_insert(&timer_env->post_list, &timer_hdl->hdr, rwnx_timer_cmp);
                    timer_hdl->state = RWNX_TIMER_STATE_POST;
                } else {
                    WRN_TIMER("rwnx_timer: try to start none-stoped one: [%p] %x\n", timer_hdl, timer_hdl->state);
                }
            } else if (msg.action == RWNX_TIMER_ACTION_RESTART) {
                if (timer_hdl->state == RWNX_TIMER_STATE_POST) {
                    co_list_extract(&timer_env->post_list, &timer_hdl->hdr);
                    co_list_insert(&timer_env->post_list, &timer_hdl->hdr, rwnx_timer_cmp);
                    msg.action = RWNX_TIMER_ACTION_START;
                } else {
                    WRN_TIMER("rwnx_timer: try to restart none-posting one: [%p] %x\n", timer_hdl, timer_hdl->state);
                }
            } else if (msg.action == RWNX_TIMER_ACTION_STOP) {
                if (timer_hdl->state == RWNX_TIMER_STATE_POST) {
                    co_list_extract(&timer_env->post_list, &timer_hdl->hdr);
                    co_list_insert(&timer_env->stop_list, &timer_hdl->hdr, rwnx_timer_cmp);
                    timer_hdl->state = RWNX_TIMER_STATE_STOP;
                } else {
                    WRN_TIMER("rwnx_timer: try to stop none-post one: [%p] %x\n", timer_hdl, timer_hdl->state);
                }
            } else if (msg.action == RWNX_TIMER_ACTION_DELETE) {
                if (timer_hdl->state == RWNX_TIMER_STATE_POST) {
                    co_list_extract(&timer_env->post_list, &timer_hdl->hdr);
                    co_list_push_back(&timer_env->free_list, &timer_hdl->hdr);
                    timer_hdl->state = RWNX_TIMER_STATE_FREE;
                } else if (timer_hdl->state == RWNX_TIMER_STATE_STOP) {
                    co_list_extract(&timer_env->stop_list, &timer_hdl->hdr);
                    co_list_push_back(&timer_env->free_list, &timer_hdl->hdr);
                    timer_hdl->state = RWNX_TIMER_STATE_FREE;
                } else {
                    WRN_TIMER("rwnx_timer: try to delete free one: [%p] %x\n", timer_hdl, timer_hdl->state);
                }
            } else {
                WRN_TIMER("rwnx_timer action invalid, [%p] %x\n", timer_hdl, msg.action);
            }
            timer_hdl->action = RWNX_TIMER_ACTION_NONE;
        } else if (ret == AIC_RTOS_WAIT_ERROR) { // timed-out, callback funcs
            #if 1
            do {
                timer_hdl = (rwnx_timer_handle)co_list_pick(&timer_env->post_list);
                if (timer_hdl) {
                    if (timer_hdl->expired_ms <= cur_time_ms) {
                        timer_hdl = (rwnx_timer_handle)co_list_pop_front(&timer_env->post_list);
                        co_list_insert(&timer_env->stop_list, &timer_hdl->hdr, rwnx_timer_cmp);
                        #ifndef CONFIG_RWNX_TIMER_TASK_PB
                        //timer_hdl->state = RWNX_TIMER_STATE_STOP;
                        DBG_TIMER("rwnx_timer process1: %p\n",timer_hdl);
                        co_list_push_back(&timer_hdl_list, &timer_hdl->hdr);
                        //(timer_hdl->cb)(timer_hdl->arg); // callback func
                        #else /* CONFIG_RWNX_TIMER_TASK_PB */
                        timer_hdl->state = RWNX_TIMER_STATE_STOP;
                        DBG_TIMER("rwnx_timer process1: %p\n",timer_hdl);
                        (timer_hdl->cb)(timer_hdl->arg,timer_hdl->arg1); // callback func
                        #endif /* CONFIG_RWNX_TIMER_TASK_PB */
                    } else {
                        break;
                    }
                }
            } while (timer_hdl);
            #endif
        } else if (ret != AIC_RTOS_QUEUE_EMPTY) {
            WRN_TIMER("rwnx_timer queue read fail, ret = %x\n", ret);
            if (ret == AIC_RTOS_QUEUE_ERROR) {
                WRN_TIMER("TX_QUEUE_ERROR\n");
                break;
            }
        }
        #if 1
        do {
            timer_hdl = (rwnx_timer_handle)co_list_pick(&timer_env->post_list);
            if (timer_hdl) {
                if (timer_hdl->expired_ms <= cur_time_ms) {
                    timer_hdl = (rwnx_timer_handle)co_list_pop_front(&timer_env->post_list);
                    co_list_insert(&timer_env->stop_list, &timer_hdl->hdr, rwnx_timer_cmp);
                    #ifndef CONFIG_RWNX_TIMER_TASK_PB
                    //timer_hdl->state = RWNX_TIMER_STATE_STOP;
                    DBG_TIMER("rwnx_timer process2: %p, %d<=%d\n",timer_hdl,timer_hdl->expired_ms,cur_time_ms);
                    co_list_push_back(&timer_hdl_list, &timer_hdl->hdr);
                    //(timer_hdl->cb)(timer_hdl->arg); // callback func
                    #else /* CONFIG_RWNX_TIMER_TASK_PB */
                    timer_hdl->state = RWNX_TIMER_STATE_STOP;
                    DBG_TIMER("rwnx_timer process2: %p, %d<=%d\n",timer_hdl,timer_hdl->expired_ms,cur_time_ms);
                    (timer_hdl->cb)(timer_hdl->arg,timer_hdl->arg1); // callback func
                    #endif /* CONFIG_RWNX_TIMER_TASK_PB */
                } else {
                    break;
                }
            }
        } while (timer_hdl);
        #endif
        timer_hdl = (rwnx_timer_handle)co_list_pick(&timer_env->post_list);
        if (timer_hdl == NULL) {
            next_to_ms = -1;
        } else {
            next_to_ms = (int)(timer_hdl->expired_ms - cur_time_ms);
            if (next_to_ms <= 0) {
                WRN_TIMER("expired timer is not processed, %x, %x\n", timer_hdl->expired_ms, cur_time_ms);
                next_to_ms = 1;
            }
        }
        rtos_mutex_unlock(timer_env->task_mutex);
        #ifndef CONFIG_RWNX_TIMER_TASK_PB
        rwnx_timer_handle timer_hdl_exec = NULL;
        do {
            timer_hdl_exec = (rwnx_timer_handle)co_list_pop_front(&timer_hdl_list);
            if (timer_hdl_exec) {
                (timer_hdl_exec->cb)(timer_hdl_exec->arg,timer_hdl_exec->arg1);
                timer_hdl_exec->state = RWNX_TIMER_STATE_STOP;
            }
        } while (timer_hdl_exec);
        #endif /* CONFIG_RWNX_TIMER_TASK_PB */
        #ifdef CONFIG_RWNX_TIMER_TASK_TS
        end_time = rtos_now(0);
        aic_dbg("tmr_out:%u/%u\n", end_time, tmr_count);
        #endif
    }
}

void rwnx_timer_init(void)
{
    int ret, idx;
    struct rwnx_timer_tag *timer_env = &rwnx_timer_env;
    rwnx_timer_handle timer_hdl;
    co_list_init(&timer_env->free_list);
    for (idx = 0; idx < RWNX_TIMER_COUNT; idx++) {
        struct rwnx_timer_node_s timer_node_void = {{NULL},};
        timer_hdl = &rwnx_timer_node_pool[idx];
        *timer_hdl = timer_node_void;
        co_list_push_back(&timer_env->free_list, &timer_hdl->hdr);
    }
    co_list_init(&timer_env->post_list);
    co_list_init(&timer_env->stop_list);
    ret = rtos_queue_create(sizeof(struct rwnx_timer_msg_s), RWNX_TIMER_COUNT * 32, &timer_env->task_queue, "TimerTaskQueue");
    if (ret < 0) {
        WRN_TIMER("rwnx_timer queue create fail\n");
        return;
    }
    ret = rtos_mutex_create(&timer_env->task_mutex, "TimerTaskMutex");
    if (ret < 0) {
        WRN_TIMER("rwnx_timer mutex create fail\n");
        return;
    }
    ret = rtos_task_create(rwnx_timer_task, "rwnx_timer_task",
                           RWNX_TIMER_TASK, rwnx_timer_stack_size,
                           (void*)timer_env, rwnx_timer_priority,
                           &timer_env->task_handle);
    if (ret < 0) {
        WRN_TIMER("rwnx_timer_task create fail\n");
        return;
    }
}

void rwnx_timer_deinit(void)
{
    uint32_t msg;
    struct rwnx_timer_tag *timer_env = &rwnx_timer_env;
    if (timer_env->task_handle) {
        rtos_task_delete(timer_env->task_handle);
        timer_env->task_handle = NULL;
    }
    if (timer_env->task_queue) {
        while (!rtos_queue_is_empty(timer_env->task_queue)) {
            rtos_queue_read(timer_env->task_queue, &msg, 30, false);
            WRN_TIMER("timer_env->task_queue msg:%X\n", msg);
        }
        rtos_queue_delete(timer_env->task_queue);
        timer_env->task_queue = NULL;
    }
    if (timer_env->task_mutex) {
        rtos_mutex_delete(timer_env->task_mutex);
        timer_env->task_mutex = NULL;
    }
}

int rwnx_timer_create(rwnx_timer_handle *timer_ptr, uint32_t period_ms,
                      void *arg,void *arg1, rwnx_timer_cb_t cb,
                      bool auto_load, bool periodic)
{
    int ret = 0;
    rwnx_timer_handle timer_hdl = NULL;
    struct rwnx_timer_tag *timer_env = &rwnx_timer_env;
    struct rwnx_timer_msg_s msg;
    rtos_mutex_lock(timer_env->task_mutex, -1);
    do {
        timer_hdl = (rwnx_timer_handle)co_list_pop_front(&timer_env->free_list);
        if (timer_hdl == NULL) {
            ERR_TIMER("rwnx_timer create fail\n");
            ret = -1;
            break;
        }
        timer_hdl->cb = cb;
        timer_hdl->arg = arg;
		timer_hdl->arg1 = arg1;
        timer_hdl->expired_ms = rtos_now(false) + period_ms;
        timer_hdl->periodic = periodic;
        timer_hdl->auto_load = auto_load;
        timer_hdl->action = msg.action = RWNX_TIMER_ACTION_CREATE;
        msg.hdl = timer_hdl;
        DBG_TIMER("rwnx_timer_create, timer_hdl=%p, ms=%d\n",timer_hdl,timer_hdl->expired_ms);

        #if 0
		do{
			struct rwnx_timer_msg_s msg;
			if(rtos_queue_is_full(timer_env->task_queue));
			{
				WRN_TIMER("C: tmr q full, cnt=%d\n", rtos_queue_cnt(timer_env->task_queue));
				int ret_rd = rtos_queue_read(timer_env->task_queue, &msg, 1, false);
                if (ret_rd) {
                    ERR_TIMER("C: tmr q rd fail: ret=%d\n", ret_rd);
                }
			}

		}while(0);
        #endif

        ret = rtos_queue_write(timer_env->task_queue, &msg, 0, false); // to be confirmed
        if (ret) {
            WRN_TIMER("rwnx_timer_create queue write fail, ret=%x\n", ret);
            co_list_push_back(&timer_env->free_list, &timer_hdl->hdr);
            ret = -2;
            break;
        }
        *timer_ptr = timer_hdl;
    } while (0);
    rtos_mutex_unlock(timer_env->task_mutex);
    return ret;
}

#ifndef CONFIG_RWNX_TIMER_TASK_PB
int rwnx_timer_start(rwnx_timer_handle timer_hdl, uint32_t period_ms)
{
    int ret = 0;
    if ((timer_hdl < &rwnx_timer_node_pool[0]) || (timer_hdl > &rwnx_timer_node_pool[RWNX_TIMER_COUNT - 1])) {
        ERR_TIMER("rwnx_timer_start invalid handle=%p\n", timer_hdl);
        ret = -1; // invalid ptr
    } else {
        struct rwnx_timer_tag *timer_env = &rwnx_timer_env;
        struct rwnx_timer_msg_s msg;
        rtos_mutex_lock(timer_env->task_mutex, -1);
        timer_hdl->expired_ms = rtos_now(false) + period_ms;
        timer_hdl->action = msg.action = RWNX_TIMER_ACTION_START;
        msg.hdl = timer_hdl;
        DBG_TIMER("rwnx_timer_start, timer_hdl=%p, ms=%d\n",timer_hdl,timer_hdl->expired_ms);

        #if 0
		do{
			struct rwnx_timer_msg_s msg;
			if(rtos_queue_is_full(timer_env->task_queue));
			{
				WRN_TIMER("T: tmr q full, cnt=%d\n", rtos_queue_cnt(timer_env->task_queue));
				int ret_rd = rtos_queue_read(timer_env->task_queue, &msg, 1, false);
                if (ret_rd) {
                    ERR_TIMER("T: tmr q rd fail: ret=%d\n", ret_rd);
                }
			}

		}while(0);
        #endif

        ret = rtos_queue_write(timer_env->task_queue, &msg, 0, false); // to be confirmed
        if (ret) {
            ERR_TIMER("rwnx_timer_start queue write fail, ret=%x\n", ret);
			ret = -2;
        }
        rtos_mutex_unlock(timer_env->task_mutex);
    }
    return ret;
}
#else /* CONFIG_RWNX_TIMER_TASK_PB */
int rwnx_timer_start(rwnx_timer_handle timer_hdl, uint32_t period_ms, bool lock)
{
    int ret = 0;
    if ((timer_hdl < &rwnx_timer_node_pool[0]) || (timer_hdl > &rwnx_timer_node_pool[RWNX_TIMER_COUNT - 1])) {
        WRN_TIMER("rwnx_timer_start invalid handle=%p\n", timer_hdl);
        ret = -1; // invalid ptr
    } else {
        struct rwnx_timer_tag *timer_env = &rwnx_timer_env;
        struct rwnx_timer_msg_s msg;
        if (lock) {
            rtos_mutex_lock(timer_env->task_mutex, -1);
        }
        timer_hdl->expired_ms = rtos_now(false) + period_ms;
        msg.action = RWNX_TIMER_ACTION_START;
        msg.hdl = timer_hdl;
        DBG_TIMER("rwnx_timer_start, timer_hdl=%p, ms=%d\n",timer_hdl,timer_hdl->expired_ms);
        ret = rtos_queue_write(timer_env->task_queue, &msg, 0, false); // to be confirmed
        if (ret) {
            WRN_TIMER("rwnx_timer_start queue write fail, ret=%x\n", ret);
            ret = -2;
        }
        if (lock) {
            rtos_mutex_unlock(timer_env->task_mutex);
        }
    }
    return ret;
}
#endif /* CONFIG_RWNX_TIMER_TASK_PB */
int rwnx_timer_restart(rwnx_timer_handle timer_hdl, uint32_t period_ms)
{
    int ret = 0;
    if ((timer_hdl < &rwnx_timer_node_pool[0]) || (timer_hdl > &rwnx_timer_node_pool[RWNX_TIMER_COUNT - 1])) {
        ERR_TIMER("rwnx_timer_start invalid handle=%p\n", timer_hdl);
        ret = -1; // invalid ptr
    } else {
        struct rwnx_timer_tag *timer_env = &rwnx_timer_env;
        struct rwnx_timer_msg_s msg;
        rtos_mutex_lock(timer_env->task_mutex, -1);
        timer_hdl->expired_ms = rtos_now(false) + period_ms;
        //if (timer_hdl->action != RWNX_TIMER_ACTION_RESTART) {
            timer_hdl->action = msg.action = RWNX_TIMER_ACTION_RESTART;
            msg.hdl = timer_hdl;
            DBG_TIMER("rwnx_timer_restart, timer_hdl=%p, ms=%d\n",timer_hdl,timer_hdl->expired_ms);
            ret = rtos_queue_write(timer_env->task_queue, &msg, 0, false); // to be confirmed
            if (ret) {
                WRN_TIMER("rwnx_timer_start queue write fail, ret=%x\n", ret);
                ret = -2;
            }
        //}
        rtos_mutex_unlock(timer_env->task_mutex);
    }
    return ret;
}

int rwnx_timer_stop(rwnx_timer_handle timer_hdl)
{
    int ret = 0;
    if ((timer_hdl < &rwnx_timer_node_pool[0]) || (timer_hdl > &rwnx_timer_node_pool[RWNX_TIMER_COUNT - 1])) {
        ERR_TIMER("rwnx_timer_stop invalid handle=%p\n", timer_hdl);
        ret = -1; // invalid ptr
    } else {
        struct rwnx_timer_tag *timer_env = &rwnx_timer_env;
        struct rwnx_timer_msg_s msg;
        rtos_mutex_lock(timer_env->task_mutex, -1);
        timer_hdl->action = msg.action = RWNX_TIMER_ACTION_STOP;
        msg.hdl = timer_hdl;

        #if 0
		do{
			struct rwnx_timer_msg_s msg;
			if(rtos_queue_is_full(timer_env->task_queue));
			{
				WRN_TIMER("P: tmr q full, cnt=%d\n", rtos_queue_cnt(timer_env->task_queue));
				int ret_rd = rtos_queue_read(timer_env->task_queue, &msg, 1, false);
                if (ret_rd) {
                    ERR_TIMER("P: tmr q rd fail: ret=%d\n", ret_rd);
                }
			}

		}while(0);
        #endif

        DBG_TIMER("rwnx_timer_stop, timer_hdl=%p\n",timer_hdl);
        ret = rtos_queue_write(timer_env->task_queue, &msg, 0, false); // to be confirmed
        if (ret) {
            WRN_TIMER("rwnx_timer_stop queue write fail, ret=%x\n", ret);
            ret = -2;
        }
        rtos_mutex_unlock(timer_env->task_mutex);
    }
    return ret;
}

int rwnx_timer_delete(rwnx_timer_handle timer_hdl)
{
    int ret = 0;
    if ((timer_hdl < &rwnx_timer_node_pool[0]) || (timer_hdl > &rwnx_timer_node_pool[RWNX_TIMER_COUNT - 1])) {
        ERR_TIMER("rwnx_timer_delete invalid handle=%p\n", timer_hdl);
        ret = -1; // invalid ptr
    } else {
        struct rwnx_timer_tag *timer_env = &rwnx_timer_env;
        struct rwnx_timer_msg_s msg;
        rtos_mutex_lock(timer_env->task_mutex, -1);
        timer_hdl->action = msg.action = RWNX_TIMER_ACTION_DELETE;
        msg.hdl = timer_hdl;
        DBG_TIMER("rwnx_timer_delete, timer_hdl=%p\n",timer_hdl);

		do{
			struct rwnx_timer_msg_s msg;
			uint32_t qcnt = rtos_queue_cnt(timer_env->task_queue);
			while(qcnt--)
			{
				rtos_queue_read(timer_env->task_queue, &msg, 1, false);
			}

		}while(0);

        ret = rtos_queue_write(timer_env->task_queue, &msg, 0, false); // to be confirmed
        if (ret) {
            WRN_TIMER("rwnx_timer_stop queue write fail, ret=%x\n", ret);
            ret = -2;
        }
        rtos_mutex_unlock(timer_env->task_mutex);
    }
    return ret;
}

bool rwnx_timer_is_posted(rwnx_timer_handle timer_hdl)
{
    bool posted = false;
    if ((timer_hdl < &rwnx_timer_node_pool[0]) || (timer_hdl > &rwnx_timer_node_pool[RWNX_TIMER_COUNT - 1])) {
        ERR_TIMER("rwnx_timer_is_posted invalid handle=%p\n", timer_hdl);
    } else {
        struct rwnx_timer_tag *timer_env = &rwnx_timer_env;
        rtos_mutex_lock(timer_env->task_mutex, -1);
        if (timer_hdl->state == RWNX_TIMER_STATE_POST) {
            posted = true;
        }
        rtos_mutex_unlock(timer_env->task_mutex);
    }
    return posted;
}

bool rwnx_timer_action_in_progress(rwnx_timer_handle timer_hdl, enum rwnx_timer_action_e action)
{
    bool action_in_prog = false;
    if ((timer_hdl < &rwnx_timer_node_pool[0]) || (timer_hdl > &rwnx_timer_node_pool[RWNX_TIMER_COUNT - 1])) {
        ERR_TIMER("rwnx_timer_is_posted invalid handle=%p\n", timer_hdl);
    } else {
        struct rwnx_timer_tag *timer_env = &rwnx_timer_env;
        rtos_mutex_lock(timer_env->task_mutex, -1);
        if (timer_hdl->action == action) {
            action_in_prog = true;
        }
        rtos_mutex_unlock(timer_env->task_mutex);
    }
    return action_in_prog;
}
#endif

#if (AICWF_RX_REORDER)
struct co_list stas_reord_list;
rtos_mutex stas_reord_lock;

#ifdef CONFIG_RX_NOCOPY
void reord_rxframe_free(struct recv_msdu *rxframe, uint8_t free_frame)
{
    // check whether free at now
    if (free_frame) {
        net_buf_rx_free(rxframe->lwip_msg);
    }
	rtos_free(rxframe);
}
#else
void reord_rxframe_free(struct recv_msdu *rxframe)
{
    rtos_free(rxframe->rx_buf);
    rtos_free(rxframe);
}
#endif /* CONFIG_RX_NOCOPY */

struct recv_msdu *reord_rxframe_alloc(struct fhost_rx_buf_tag *buf, uint16_t seq_num, uint8_t tid, uint8_t forward)
{
    struct recv_msdu *rxframe;
    struct fhost_rx_buf_tag *rx_buf;
    uint8_t *data = (uint8_t *)buf;
    uint16_t buf_len = sizeof(struct rx_info) + (*data | (*(data + 1) << 8));

    rxframe = rtos_malloc(sizeof(struct recv_msdu));
    if (rxframe == NULL) {
        WRN_REORD("rxframe alloc fail\n");
        return NULL;
    }

#ifdef CONFIG_RX_NOCOPY
	//.....[|fhost_rx_buf_tag->rx_info|payl_offset|real_payload]

	uint8_t payl_offset = fhost_mac2ethernet_offset(buf);
	uint16_t p_headsize = sizeof(struct rx_info)+payl_offset;
	uint16_t p_reallen = buf_len-p_headsize;

	AIC_ASSERT_ERR((p_headsize) <= AICWF_WIFI_HDR);

	uint8_t *lwip_buf =net_buf_rx_alloc(p_reallen+AICWF_WIFI_HDR);

	if(lwip_buf == NULL)
	{
        WRN_REORD("rxframe alloc fail\n");
        return NULL;
	}
	rx_buf = (void *)(lwip_buf+AICWF_WIFI_HDR-p_headsize);
	rxframe->lwip_msg = platform_net_buf_rx_desc(lwip_buf);
#else
    rx_buf = rtos_malloc(buf_len);
    if (rx_buf == NULL) {
        rtos_free(rxframe);
        WRN_REORD("rxframe buf alloc fail\n");
        aic_dbg("rxframe buf alloc fail\n");
        return NULL;
    }
#endif /* CONFIG_RX_NOCOPY */

    memcpy(rx_buf, buf, buf_len);
    rxframe->rx_buf = rx_buf;
    rxframe->buf_len = buf_len;
    rxframe->seq_num = seq_num;
    rxframe->tid = tid;
    rxframe->forward = forward;
    return rxframe;
}

struct reord_ctrl_info *reord_init_sta(const uint8_t *mac_addr)
{
    int i, ret;
    struct reord_ctrl *preorder_ctrl = NULL;
    struct reord_ctrl_info *reord_info;
    char reord_list_lock_name[32];
    WRN_REORD("reord_init_sta %02x:%02x:%02x\n", mac_addr[0], mac_addr[1], mac_addr[2]);
    reord_info = rtos_malloc(sizeof(struct reord_ctrl_info));
    if (!reord_info) {
        WRN_REORD("reord_info alloc fail\n");
        return NULL;
    }
    memcpy(reord_info->mac_addr, mac_addr, 6);
    for (i = 0; i < 8; i++) {
        memset(reord_list_lock_name, 0, sizeof(reord_list_lock_name));
        sprintf(reord_list_lock_name, "reord_list_lock%d", i);
        preorder_ctrl = &reord_info->preorder_ctrl[i];
        preorder_ctrl->enable = true;
        preorder_ctrl->wsize_b = AICWF_REORDER_WINSIZE;
        preorder_ctrl->ind_sn = 0xffff;
        preorder_ctrl->list_cnt = 0;
        co_list_init(&preorder_ctrl->reord_list);
        ret = rtos_mutex_create(&preorder_ctrl->reord_list_lock, reord_list_lock_name);
        if (ret) {
            WRN_REORD("reord_ctrl[%x] mutex create fail\n", i);
            continue;
        }
        preorder_ctrl->reord_timer = NULL;
    }
    return reord_info;
}

int reord_flush_tid(uint8_t *mac, uint8_t tid)
{
    struct reord_ctrl_info *reord_info;
    struct reord_ctrl *preorder_ctrl = NULL;
    struct recv_msdu *prframe;
    int ret;
    rtos_mutex_lock(stas_reord_lock, -1);
    reord_info = (struct reord_ctrl_info *)co_list_pick(&stas_reord_list);
    while (reord_info) {
        if (memcmp(mac, reord_info->mac_addr, 6/*ETH_ALEN*/) == 0) {
            preorder_ctrl = &reord_info->preorder_ctrl[tid];
            break;
        }
        reord_info = (struct reord_ctrl_info *)co_list_next(&reord_info->hdr);
    }
    rtos_mutex_unlock(stas_reord_lock);
    if ((NULL == preorder_ctrl) || (false == preorder_ctrl->enable)) { // not found or disabled
        return 0;
    }
    rtos_mutex_lock(preorder_ctrl->reord_list_lock, -1);
    do {
        prframe = (struct recv_msdu*)co_list_pop_front(&preorder_ctrl->reord_list);
        if (prframe == NULL) {
            break;
        }
        reord_single_frame_ind(prframe);
    } while (1);
    WRN_REORD("reord flush:tid=%d", tid);
    preorder_ctrl->enable = false;
    preorder_ctrl->ind_sn = 0xffff;
    preorder_ctrl->list_cnt = 0;
    rtos_mutex_unlock(preorder_ctrl->reord_list_lock);
    //rtos_mutex_delete(preorder_ctrl->reord_list_lock);
    if (preorder_ctrl->reord_timer) {
        #if (AICWF_RWNX_TIMER_EN)
        rwnx_timer_delete(preorder_ctrl->reord_timer);
        #else
        rtos_timer_delete(preorder_ctrl->reord_timer, 0);
        #endif
        preorder_ctrl->reord_timer = NULL;
    }
    return 0;
}

void reord_deinit_sta(struct reord_ctrl_info *reord_info)
{
    int idx;
    uint8_t *mac_addr = &reord_info->mac_addr[0];
    WRN_REORD("reord_deinit_sta %02x:%02x:%02x\n", mac_addr[0], mac_addr[1], mac_addr[2]);
    for (idx = 0; idx < 8; idx++) {
        struct reord_ctrl *preorder_ctrl = &reord_info->preorder_ctrl[idx];
        struct recv_msdu *prframe;
        preorder_ctrl->enable = false;
        rtos_mutex_lock(preorder_ctrl->reord_list_lock, -1);
        do {
            prframe = (struct recv_msdu*)co_list_pop_front(&preorder_ctrl->reord_list);
            if (prframe) {
                #ifdef CONFIG_RX_NOCOPY
                reord_rxframe_free(prframe, true);
                #else
                reord_rxframe_free(prframe);
                #endif
            }
        } while (prframe);
        rtos_mutex_unlock(preorder_ctrl->reord_list_lock);
        rtos_mutex_delete(preorder_ctrl->reord_list_lock);
        if (preorder_ctrl->reord_timer) {
            #if (AICWF_RWNX_TIMER_EN)
            rwnx_timer_delete(preorder_ctrl->reord_timer);
            #else
            rtos_timer_delete(preorder_ctrl->reord_timer, 0);
            #endif
            preorder_ctrl->reord_timer = NULL;
        }
    }
    rtos_free(reord_info);
}
void reord_deinit_sta_by_mac(const uint8_t *mac_addr)
{
    int idx;
    struct reord_ctrl_info *reord_info = NULL;
    struct reord_ctrl *preorder_ctrl = NULL;

	if(mac_addr != NULL)
		AIC_LOG_PRINTF("reord_deinit_sta mac  %02x:%02x:%02x\n",mac_addr[0], mac_addr[1], mac_addr[2]);
	rtos_mutex_lock(stas_reord_lock, -1);

	while(1)
	{
	    reord_info = (struct reord_ctrl_info *)co_list_pick(&stas_reord_list);

		while (reord_info) {
			if(mac_addr == NULL)
				break;
			AIC_LOG_PRINTF("reord_deinit_sta_mac find %02x:%02x:%02x\n", reord_info->mac_addr[0], reord_info->mac_addr[1], reord_info->mac_addr[2]);
	        if (memcmp(mac_addr, reord_info->mac_addr, 6/*ETH_ALEN*/) == 0) {
	            break;
	        }
	        reord_info = (struct reord_ctrl_info *)co_list_next(&reord_info->hdr);
	    }

		if(reord_info != NULL )
		{

			co_list_extract(&stas_reord_list, &reord_info->hdr);

			reord_deinit_sta(reord_info);
			if(mac_addr != NULL)
				break;
		}
		else
		{
			break;
		}

	};

	rtos_mutex_unlock(stas_reord_lock);
}

int reord_single_frame_ind(struct recv_msdu *prframe)
{
    DBG_REORD("reord_single_frame_ind:[%x]%d\n", prframe->tid, prframe->seq_num);

    #ifdef CONFIG_RX_NOCOPY
    if (prframe->forward)
	{
        fhost_rx_buf_forward(prframe->rx_buf, prframe->lwip_msg, true);
		reord_rxframe_free(prframe, false);
	}
	else
	{
		reord_rxframe_free(prframe, true);
	}

    #else
    if (prframe->forward) {
        fhost_rx_buf_forward(prframe->rx_buf);
    }
    reord_rxframe_free(prframe);
    #endif /* CONFIG_RX_NOCOPY */

    return 0;
}

bool reord_rxframes_process(struct reord_ctrl *preorder_ctrl, int bforced)
{
    struct list_head *phead, *plist;
    struct recv_msdu *prframe;
    bool bPktInBuf = false;
    uint16_t prev_sn = preorder_ctrl->ind_sn;
    if (co_list_is_empty(&preorder_ctrl->reord_list)) {
        return false;
    }
    prframe = (struct recv_msdu *)co_list_pick(&preorder_ctrl->reord_list);
    if (bforced == true && prframe != NULL) {
        preorder_ctrl->ind_sn = prframe->seq_num;
        WRN_REORD("reord force ind_sn=%d,%d\n", preorder_ctrl->ind_sn,prframe->buf_len);
    }
    while (prframe) {
        if (!SN_LESS(preorder_ctrl->ind_sn, prframe->seq_num)) {
            if (SN_EQUAL(preorder_ctrl->ind_sn, prframe->seq_num)) {
                preorder_ctrl->ind_sn = (preorder_ctrl->ind_sn + 1) & 0xFFF;
            }
        } else {
            bPktInBuf = true;
            break;
        }
        prframe = (struct recv_msdu *)co_list_next(&prframe->hdr);
    }
    if (prev_sn != preorder_ctrl->ind_sn) {
        WRN_REORD("reord process set ind_sn=%d\n", preorder_ctrl->ind_sn);
    }
    return bPktInBuf;
}

void reord_rxframes_ind(struct reord_ctrl *preorder_ctrl)
{
    struct co_list *reord_list;
    struct recv_msdu *prframe;
    reord_list = &preorder_ctrl->reord_list;
    rtos_mutex_lock(preorder_ctrl->reord_list_lock, -1);
    if (co_list_is_empty(reord_list)) {
        rtos_mutex_unlock(preorder_ctrl->reord_list_lock);
        return;
    }
    do {
        prframe = (struct recv_msdu *)co_list_pick(reord_list);
        if (!prframe) {
            break;
        } else if (SN_LESS(preorder_ctrl->ind_sn, prframe->seq_num)){
            break;
        }
        co_list_pop_front(reord_list);
        //co_list_extract(reord_list, &prframe->rxframe_list);
        reord_single_frame_ind(prframe);
        preorder_ctrl->list_cnt--;
    } while (1);
    rtos_mutex_unlock(preorder_ctrl->reord_list_lock);
}

int reord_need_check(struct reord_ctrl *preorder_ctrl, uint16_t seq_num)
{
    uint8_t wsize = preorder_ctrl->wsize_b;
    uint16_t wend = (preorder_ctrl->ind_sn + wsize -1) & 0xFFF;
    if (preorder_ctrl->ind_sn == 0xFFFF) {
        preorder_ctrl->ind_sn = seq_num;
        DBG_REORD("reord chk&set 1 ind_sn=%d\n",preorder_ctrl->ind_sn);
    }
    if (SN_LESS(seq_num, preorder_ctrl->ind_sn)) {
        return -1;
    }
    if (SN_EQUAL(seq_num, preorder_ctrl->ind_sn)) {
        preorder_ctrl->ind_sn = (preorder_ctrl->ind_sn + 1) & 0xFFF;
    } else if (SN_LESS(wend, seq_num)) {
        if (seq_num >= (wsize-1))
            preorder_ctrl->ind_sn = seq_num-(wsize-1);
        else
            preorder_ctrl->ind_sn = 0xFFF - (wsize - (seq_num + 1)) + 1;
    }
    DBG_REORD("reord chk&set 2 ind_sn=%d\n",preorder_ctrl->ind_sn);
    return 0;
}

int reord_rxframe_enqueue(struct reord_ctrl *preorder_ctrl, struct recv_msdu *prframe)
{
    struct co_list *preord_list = &preorder_ctrl->reord_list;
    struct recv_msdu *plframe = (struct recv_msdu *)co_list_pick(preord_list);
    int idx = 0;
    DBG_REORD("reord_enq,sn=%d\n",prframe->seq_num);
    while (plframe) {
        DBG_REORD(" (%d)->%d,%d\n",idx++,plframe->seq_num,plframe->buf_len);
        if (SN_LESS(plframe->seq_num, prframe->seq_num)) {
            plframe = (struct recv_msdu *)co_list_next(&plframe->hdr);
            continue;
        } else if (SN_EQUAL(plframe->seq_num, prframe->seq_num)) {
            WRN_REORD("reord dup, drop sn=%d\n",prframe->seq_num);
            return -1;
        } else {
            break;
        }
    }
    if (plframe) {
        DBG_REORD("reord insert, sn_new=%d bef sn_old=%d\n",prframe->seq_num, plframe->seq_num);
        co_list_insert_before(preord_list, &plframe->hdr, &prframe->hdr);
    } else {
        DBG_REORD("reord push back, sn_new=%d\n",prframe->seq_num);
        co_list_push_back(preord_list, &prframe->hdr);
    }
    preorder_ctrl->list_cnt++;
    return 0;
}

void reord_timeout_worker(struct reord_ctrl *preorder_ctrl)
{
    reord_rxframes_ind(preorder_ctrl);
}

void reord_timeout_handler(void *arg,void *arg1)
{
    struct reord_ctrl *preorder_ctrl = (struct reord_ctrl *)arg;
	struct reord_ctrl_info *reord_info = (struct reord_ctrl_info *)arg1;
    WRN_REORD("reord TO\n");

    rtos_mutex_lock(stas_reord_lock, -1);
	bool find = co_list_find(&stas_reord_list, &reord_info->hdr);

	if(!find)
	{
		rtos_mutex_unlock(stas_reord_lock);
		return;
	}

	rtos_mutex_lock(preorder_ctrl->reord_list_lock, -1);
	bool inbuf = reord_rxframes_process(preorder_ctrl, true);
	rtos_mutex_unlock(preorder_ctrl->reord_list_lock);

    reord_timeout_worker(preorder_ctrl);

	rtos_mutex_unlock(stas_reord_lock);

    if (inbuf == true) {
        int ret;
        #if (AICWF_RWNX_TIMER_EN)
        if (preorder_ctrl->reord_timer) {
            WRN_REORD("reord_timeout_handler, start timer,ind_sn=%d,cnt=%d\n",preorder_ctrl->ind_sn,preorder_ctrl->list_cnt);
            #ifndef CONFIG_RWNX_TIMER_TASK_PB
            rwnx_timer_start(preorder_ctrl->reord_timer, REORDER_UPDATE_TIME);
            #else /* CONFIG_RWNX_TIMER_TASK_PB */
            rwnx_timer_start(preorder_ctrl->reord_timer, REORDER_UPDATE_TIME, false);
            #endif /* CONFIG_RWNX_TIMER_TASK_PB */
        } else {
            WRN_REORD("reord_timeout_handler, null timer handle\n");
        }
        #else
        if (preorder_ctrl->reord_timer) {
            rtos_timer_stop(preorder_ctrl->reord_timer, 0);
            rtos_timer_delete(preorder_ctrl->reord_timer, 0);
            preorder_ctrl->reord_timer = NULL;
        } else {
            WRN_REORD("reord_timeout_handler, null timer handle\n");
        }
        ret = rtos_timer_create("reord_timer", &preorder_ctrl->reord_timer,
                                REORDER_UPDATE_TIME, 1, preorder_ctrl, reord_timeout_handler);
        if (ret) {
            WRN_REORD("[TO] reord_ctrl timer create fail\n");
        }
        #endif
    }

    reord_timeout_worker(preorder_ctrl);
}

int reord_process_unit(struct fhost_rx_buf_tag *buf, uint16_t seq_num, uint8_t *macaddr, uint8_t tid, uint8_t forward)
{
    int ret;
    struct recv_msdu *pframe;
    struct reord_ctrl *preorder_ctrl = NULL;
    struct reord_ctrl_info *reord_info = NULL;

    #ifdef CONFIG_REORD_PROCESS_UNIT_TS
    uint32_t start_time = 0;
    uint32_t end_time = 0;
    static volatile uint8_t reord_count = 0;
    #endif

    #ifdef CONFIG_REORD_PROCESS_UNIT_TS
    reord_count++;
    start_time = rtos_now(0);
    aic_dbg("reo_in:%u/%u\n", start_time, reord_count);
    #endif

    pframe = reord_rxframe_alloc(buf, seq_num, tid, forward);
    if (pframe == NULL) {
        ERR_REORD("reord rxframe alloc fail\n");
        return -1;
    }
    rtos_mutex_lock(stas_reord_lock, -1);
    reord_info = (struct reord_ctrl_info *)co_list_pick(&stas_reord_list);
    while (reord_info) {
        if (memcmp(macaddr, reord_info->mac_addr, 6) == 0) {
            preorder_ctrl = &reord_info->preorder_ctrl[tid];
            break;
        }
        reord_info = (struct reord_ctrl_info *)co_list_next(&reord_info->hdr);
    }
    if (preorder_ctrl == NULL) {
        reord_info = reord_init_sta(macaddr);
        if (!reord_info) {
            WRN_REORD("reord init fail\n");
            rtos_mutex_unlock(stas_reord_lock);
            return -1;
        }
        co_list_push_back(&stas_reord_list, &reord_info->hdr);
        preorder_ctrl = &reord_info->preorder_ctrl[tid];
    } else {
        if (preorder_ctrl->enable == false) {
            preorder_ctrl->enable = true;
            preorder_ctrl->ind_sn = 0xffff;
            preorder_ctrl->wsize_b = AICWF_REORDER_WINSIZE;
            DBG_REORD("reord reset [%x],ind_sn=%d,wsize_b=%d\n",tid,preorder_ctrl->ind_sn,preorder_ctrl->wsize_b);
        }
    }
    rtos_mutex_unlock(stas_reord_lock);
    rtos_mutex_lock(preorder_ctrl->reord_list_lock, -1);
    if (preorder_ctrl->enable == false) {
        preorder_ctrl->ind_sn = seq_num;
        DBG_REORD("reord_ctrl en=false,seq_num=%d\n", seq_num);
        reord_single_frame_ind(pframe);
        preorder_ctrl->ind_sn = (preorder_ctrl->ind_sn + 1) & (4096 - 1);
        DBG_REORD("reord set [%x],ind_sn=%d\n",tid,preorder_ctrl->ind_sn);
        rtos_mutex_unlock(preorder_ctrl->reord_list_lock);
        return 0;
    }
    if (reord_need_check(preorder_ctrl, pframe->seq_num)) {
        DBG_REORD("reord_need_check, seq_num=%d\n", pframe->seq_num);
        reord_single_frame_ind(pframe);
        rtos_mutex_unlock(preorder_ctrl->reord_list_lock);
        return 0;
    }
    if (reord_rxframe_enqueue(preorder_ctrl, pframe)) {
        rtos_mutex_unlock(preorder_ctrl->reord_list_lock);
        #ifdef CONFIG_RX_NOCOPY
        reord_rxframe_free(pframe, true);
        #else
        reord_rxframe_free(pframe);
        #endif
        return -1;
    }

    #ifndef CONFIG_RWNX_TIMER_TASK_PB
    ret = reord_rxframes_process(preorder_ctrl, false);
    rtos_mutex_unlock(preorder_ctrl->reord_list_lock);
    if (ret == true) { //wait ind_sn frame
        #if (AICWF_RWNX_TIMER_EN)
        if (preorder_ctrl->reord_timer) {
            if (!rwnx_timer_is_posted(preorder_ctrl->reord_timer) &&
                !rwnx_timer_action_in_progress(preorder_ctrl->reord_timer, RWNX_TIMER_ACTION_START)) {
                WRN_REORD("reord_process_unit, start timer,ind_sn=%d,cnt=%d\n", preorder_ctrl->ind_sn,preorder_ctrl->list_cnt);
                rwnx_timer_start(preorder_ctrl->reord_timer, REORDER_UPDATE_TIME);
            } else {
                #if 0
                DBG_REORD("reord_process_unit, restart timer,ind_sn=%d,cnt=%d\n",preorder_ctrl->ind_sn,preorder_ctrl->list_cnt);
                rwnx_timer_restart(preorder_ctrl->reord_timer, REORDER_UPDATE_TIME);
                #else
                DBG_REORD("reord_process_unit, timer posted,ind_sn=%d,cnt=%d\n",preorder_ctrl->ind_sn,preorder_ctrl->list_cnt);
                #endif
            }
        } else {
            DBG_REORD("reord_process_unit, create timer\n");
            rwnx_timer_create(&preorder_ctrl->reord_timer, REORDER_UPDATE_TIME,
                              preorder_ctrl,reord_info, reord_timeout_handler,
                              true, false);
        }
        #else
        if (preorder_ctrl->reord_timer) {
            rtos_timer_stop(preorder_ctrl->reord_timer, 0);
            rtos_timer_delete(preorder_ctrl->reord_timer, 0);
            preorder_ctrl->reord_timer = NULL;
        }
        ret = rtos_timer_create("reord_timer", &preorder_ctrl->reord_timer,
                                REORDER_UPDATE_TIME, 1, preorder_ctrl, reord_timeout_handler);
        if (ret) {
            WRN_REORD("reord_ctrl timer create fail\n");
        }
        #endif
    } else {
        if (preorder_ctrl->reord_timer) {
            #if (AICWF_RWNX_TIMER_EN)
            if (rwnx_timer_is_posted(preorder_ctrl->reord_timer) &&
                !rwnx_timer_action_in_progress(preorder_ctrl->reord_timer, RWNX_TIMER_ACTION_STOP)) {
                WRN_REORD("reord_process_unit, stop timer,ind_sn=%d,cnt=%d\n",preorder_ctrl->ind_sn,preorder_ctrl->list_cnt);
                rwnx_timer_stop(preorder_ctrl->reord_timer);
            } else {
                DBG_REORD("reord timer stoped\n");
            }
            #else
            rtos_timer_stop(preorder_ctrl->reord_timer, 0);
            rtos_timer_delete(preorder_ctrl->reord_timer, 0);
            preorder_ctrl->reord_timer = NULL;
            #endif
        }
    }
    #else /* CONFIG_RWNX_TIMER_TASK_PB */
    ret = reord_rxframes_process(preorder_ctrl, false);
    rtos_mutex_unlock(preorder_ctrl->reord_list_lock);
    if (ret == true) {
        #if (AICWF_RWNX_TIMER_EN)
        if (preorder_ctrl->reord_timer) {
            if (!rwnx_timer_is_posted(preorder_ctrl->reord_timer)) {
                WRN_REORD("reord_process_unit, start timer,ind_sn=%d,cnt=%d\n", preorder_ctrl->ind_sn,preorder_ctrl->list_cnt);
                rwnx_timer_start(preorder_ctrl->reord_timer, REORDER_UPDATE_TIME, true);
            } else {
                #if 0
                DBG_REORD("reord_process_unit, restart timer,ind_sn=%d,cnt=%d\n",preorder_ctrl->ind_sn,preorder_ctrl->list_cnt);
                rwnx_timer_restart(preorder_ctrl->reord_timer, REORDER_UPDATE_TIME);
                #else
                DBG_REORD("reord_process_unit, timer posted,ind_sn=%d,cnt=%d\n",preorder_ctrl->ind_sn,preorder_ctrl->list_cnt);
                #endif
            }
        } else {
            DBG_REORD("reord_process_unit, create timer\n");
            rwnx_timer_create(&preorder_ctrl->reord_timer, REORDER_UPDATE_TIME,
                              preorder_ctrl,reord_info, reord_timeout_handler,
                              true, false);
        }
        #else
        if (preorder_ctrl->reord_timer) {
            rtos_timer_stop(preorder_ctrl->reord_timer, 0);
            rtos_timer_delete(preorder_ctrl->reord_timer, 0);
            preorder_ctrl->reord_timer = NULL;
        }
        ret = rtos_timer_create("reord_timer", &preorder_ctrl->reord_timer,
                                REORDER_UPDATE_TIME, 1, preorder_ctrl, reord_timeout_handler);
        if (ret) {
            WRN_REORD("reord_ctrl timer create fail\n");
        }
        #endif
    } else {
        if (preorder_ctrl->reord_timer) {
            #if (AICWF_RWNX_TIMER_EN)
            if (rwnx_timer_is_posted(preorder_ctrl->reord_timer)) {
                DBG_REORD("reord_process_unit, stop timer,ind_sn=%d,cnt=%d\n",preorder_ctrl->ind_sn,preorder_ctrl->list_cnt);
                rwnx_timer_stop(preorder_ctrl->reord_timer);
            } else {
                DBG_REORD("reord timer stoped\n");
            }
            #else
            rtos_timer_stop(preorder_ctrl->reord_timer, 0);
            rtos_timer_delete(preorder_ctrl->reord_timer, 0);
            preorder_ctrl->reord_timer = NULL;
            #endif
        }
    }
    #endif /* CONFIG_RWNX_TIMER_TASK_PB */
    reord_rxframes_ind(preorder_ctrl);

    #ifdef CONFIG_REORD_PROCESS_UNIT_TS
    end_time = rtos_now(0);
    aic_dbg("reo_out:%u/%u\n", end_time, reord_count);
    #endif

    return 0;
}

int rwnx_rxdataind_aicwf(struct fhost_rx_buf_tag *buf)
{
    uint8_t *frame = (uint8_t *)buf->payload;
    struct mac_hdr *machdr_ptr = (struct mac_hdr *)frame;
    struct rx_info *info =&buf->info;
    uint8_t mac_hdr_len = MAC_SHORT_MAC_HDR_LEN;
    struct llc_snap *snap_hdr;
    uint16_t eth_type, *qos;
    struct mac_addr *da, *sa;
    int need_reord, vif_idx;
    uint8_t macaddr[6] = {0,};
    uint8_t tid = 0;
    struct fhost_vif_tag *fhost_vif;
    bool resend = false, forward = true;
    if (info->flags & RX_FLAGS_MONITOR_BIT) {
        fhost_rx_monitor_cb(buf, false);
        // return -1; // p2p + monitor
    }
    if ((info->flags & RX_FLAGS_UPLOAD_BIT) == 0) {
        WRN_REORD("rxdata not upload\n");
        return -1;
    }
    if (info->flags & RX_FLAGS_NON_MSDU_MSK) {
        DBG_REORD("rxdata is nonmasdu\n");
        #ifdef CONFIG_RX_NOCOPY
        fhost_rx_buf_forward(buf, NULL, false);
        #else
        fhost_rx_buf_forward(buf);
        #endif

        return 0;
    }
    need_reord = (info->flags & RX_FLAGS_NEED_TO_REORD_BIT);
    vif_idx = (info->flags & RX_FLAGS_VIF_INDEX_MSK) >> RX_FLAGS_VIF_INDEX_OFT;
    fhost_vif = rwnx_rx_get_vif(vif_idx);
    if (fhost_vif == NULL) {
        WRN_REORD("rxdata invalid vif_idx=%x\n", vif_idx);
        return -2;
    }

	if (IS_QOS_DATA(machdr_ptr->fctl)) {
        mac_hdr_len += MAC_HDR_QOS_CTRL_LEN;
    }
    if (machdr_ptr->fctl & MAC_FCTRL_ORDER) {
        mac_hdr_len += MAC_HTCTRL_LEN;
    }

    switch (info->vect.statinfo & RX_HD_DECRSTATUS) {
        case RX_HD_DECR_CCMP128:
        case RX_HD_DECR_TKIP:
            mac_hdr_len += MAC_IV_LEN + MAC_EIV_LEN;
            break;
        case RX_HD_DECR_WEP:
            mac_hdr_len += MAC_IV_LEN;
            break;
        default:
            break;
    }

    snap_hdr = (struct llc_snap *)(frame + mac_hdr_len);
    eth_type = snap_hdr->proto_id;
    if (machdr_ptr->fctl & MAC_FCTRL_TODS) {// Get DA
        da = &machdr_ptr->addr3;
        sa = &machdr_ptr->addr2;
    } else {
        da = &machdr_ptr->addr1;
        sa = &machdr_ptr->addr3;
    }
    data_pkt_rssi_set((uint8_t *)sa, (int8_t)((info->vect.recvec1b >> 8) & 0xFF));
    if (fhost_vif->mac_vif && fhost_vif->mac_vif->type == VIF_AP) {
        if (MAC_ADDR_GROUP(da))
		{
            /* broadcast pkt need to be forwared to upper layer and resent
               on wireless interface */
			if(wifi_ap_isolate&0x02)
            	resend = false;
			else
				resend = true;
        }
		else
		{
			//@haf note ap config sta none isolation
            /* unicast pkt for STA inside the BSS, no need to forward to upper
               layer simply resend on wireless interface */
            int dst_idx = (info->flags & RX_FLAGS_DST_INDEX_MSK) >> RX_FLAGS_DST_INDEX_OFT;
            if (dst_idx != INVALID_STA_IDX)
			{
                uint8_t staid = vif_mgmt_get_staid(fhost_vif->mac_vif, da);
                if(staid < FMAC_REMOTE_STA_MAX)
				{
                    struct sta_info_tag *sta = vif_mgmt_get_sta_by_staid(staid);
                    if (sta && sta->valid)
					{
						if(wifi_ap_isolate&0x04)
							resend = false;
						else
							resend = true;

                        forward = false;
                    }
                }
            }
        }
    }
    if (resend) {
        int ret;
        struct mac_addr *sa;
        if (machdr_ptr->fctl & MAC_FCTRL_FROMDS) {// Get SA
            sa = &machdr_ptr->addr3;
        } else {
            sa = &machdr_ptr->addr2;
        }
        ret = fhost_rx_data_resend(fhost_vif->net_if, buf, da, sa, mac_hdr_len);
        if (ret) {
            WRN_REORD("rxdata resend fail: ret=%d\n", ret);
        }
    }
    if (machdr_ptr->fctl & MAC_FCTRL_TODS) {
        memcpy(macaddr, &machdr_ptr->addr2, 6);
    } else if (machdr_ptr->fctl & MAC_FCTRL_FROMDS) {
        memcpy(macaddr, &machdr_ptr->addr1, 6);
    }
    qos = &((struct mac_hdr_qos *)machdr_ptr)->qos;
    tid = (*qos & MAC_QOSCTRL_UP_MSK);
    if (forward) {
        if (!IS_QOS_DATA(machdr_ptr->fctl) || (co_ntohs(eth_type) == LLC_ETHERTYPE_EAP_T) || MAC_ADDR_GROUP(da)) {
            #ifdef CONFIG_RX_NOCOPY
            fhost_rx_buf_forward(buf, NULL, false);
            #else
            fhost_rx_buf_forward(buf);
            #endif

        } else {
            uint8_t staid = vif_mgmt_get_staid(fhost_vif->mac_vif, (struct mac_addr*)macaddr);
            if(staid < FMAC_REMOTE_STA_MAX) {
                struct sta_info_tag *sta = vif_mgmt_get_sta_by_staid(staid);
                if (sta)
                    sta->last_active_time_us = rtos_now(0);
            }
            if (need_reord) {
                uint16_t seq_num = machdr_ptr->seq >> MAC_SEQCTRL_NUM_OFT;
                DBG_REORD("reord1:[%x]%d,%d\n",tid,seq_num,sizeof(struct rx_info) + info->vect.frmlen);
                reord_process_unit(buf, seq_num, macaddr, tid, 1);
            } else {
                reord_flush_tid(macaddr, tid);
                #ifdef CONFIG_RX_NOCOPY
                fhost_rx_buf_forward(buf, NULL, false);
                #else
                fhost_rx_buf_forward(buf);
                #endif

            }
        }
    } else {
        if (need_reord) {
            uint16_t seq_num = machdr_ptr->seq >> MAC_SEQCTRL_NUM_OFT;
            WRN_REORD("reord0:[%x]%d,%d\n",tid,seq_num,sizeof(struct rx_info) + info->vect.frmlen);
            reord_process_unit(buf, seq_num, macaddr, tid, 0);
        } else {
            reord_flush_tid(macaddr, tid);
        }
    }
    return 0;
}

void rwnx_reord_init(void)
{
    int ret;
    co_list_init(&stas_reord_list);
    ret = rtos_mutex_create(&stas_reord_lock, "stas_reord_lock");
    if (ret < 0) {
        WRN_REORD("reord mutex create fail\n");
    }
    #if (AICWF_RWNX_TIMER_EN)
    rwnx_timer_init();
    #endif
}

void rwnx_reord_deinit(void)
{
    struct reord_ctrl_info *reord_info;
    do {
        reord_info = (struct reord_ctrl_info *)co_list_pop_front(&stas_reord_list);
        if (reord_info) {
            reord_deinit_sta(reord_info);
        }
    } while (reord_info);
    if (stas_reord_lock) {
        rtos_mutex_delete(stas_reord_lock);
        stas_reord_lock = NULL;
    }
#if (AICWF_RWNX_TIMER_EN)
	rwnx_timer_deinit();
#endif
}
#endif

