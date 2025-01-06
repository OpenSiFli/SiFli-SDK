/*
 * Copyright (c) 2019, redoc
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-01-12     redoc        the first version
 */

#define LOG_TAG      "state.post"
#define LOG_LVL      LOG_LVL_DBG
 
#include "state.h"
#include <ulog.h>

/*  post state graph
 *
 *  
 * +-----------------------------------------------------------------+
 * |                   root--------+                                 |        
 * |                               |                                 |        
 * |                               |                                 |        
 * |  +----------+  fail      +----------+  pass     +----------+    |        
 * |  | postfail | <--------  |   post   | --------> | postpass |    |        
 * |  +----------+            +----------+           +----------+    |        
 * |                              ^    \post                         |        
 * |                          post|     \break                       |        
 * |                         break|      \on                         |        
 * |                           off|       \---->  +-----------+      |        
 * |                              +-------------  | postbreak |      |        
 * |                                              +-----------+      |        
 * |                                                                 |           
 * +-----------------------------------------------------------------+
 * 
 */

enum event_post_type 
{
    EVENT_POST_NULL,
    EVENT_POST_START,
    EVENT_POST_BREAKON,
    EVENT_POST_BREAKOFF,
    EVENT_POST_ANSWER,

    EVENT_POST_NUMS,
};

static struct state state_root, state_post, state_postpass, state_postfail,
                    state_postbreak;

static void print_msg_err( void *state_data, struct event *event );
static void print_msg_enter( void *state_data, struct event *event );
static void print_msg_exit( void *state_data, struct event *event );
static void state_post_enter( void *state_data, struct event *event );
static bool guard_post_pass( void *condition, struct event *event );
static bool guard_post_fail( void *condition, struct event *event );
static void action_post_break( void *oldstate_data, struct event *event,
      void *state_new_data );
static void action_post_pass( void *oldstate_data, struct event *event,
      void *state_new_data );
static void action_post_fail( void *oldstate_data, struct event *event,
      void *state_new_data );

static struct state state_root = {
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[]){
        /* event_type, condition, guard, action, next, state  */
        { EVENT_POST_START, NULL, NULL, NULL, &state_post },
    },
    .transition_nums = 1,
    .data = "ROOT",
    .action_entry = &print_msg_enter,
    .action_exti = &print_msg_exit,
};

static struct state state_post = {
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[]){
        { EVENT_POST_BREAKON,     NULL,             NULL,   &action_post_break, &state_postbreak },
        { EVENT_POST_ANSWER,  (void*)1, &guard_post_fail,    &action_post_fail,  &state_postfail },
        { EVENT_POST_ANSWER,  (void*)2, &guard_post_pass,    &action_post_pass,  &state_postpass },
    },
    .transition_nums = 3,
    .data = "POST",
    .action_entry = &state_post_enter,
    .action_exti = &print_msg_exit,
};

static struct state state_postpass = {
    .state_parent = NULL,
    .state_entry = NULL,
    .data = "POSTPASS",
    .action_entry = &print_msg_enter,
    .action_exti = &print_msg_exit,
};

static struct state state_postfail = {
    .state_parent = NULL,
    .state_entry = NULL,
    .data = "POSTFAIL",
    .action_entry = &print_msg_enter,
    .action_exti = &print_msg_exit,
};

static struct state state_postbreak = {
    .state_parent = NULL,
    .state_entry = NULL,
    .transitions = (struct transition[]){
        { EVENT_POST_BREAKOFF, NULL, NULL, NULL, &state_post },
    },
    .transition_nums = 1,
    .data = "POSTBREAK",
    .action_entry = &print_msg_enter,
    .action_exti = &print_msg_exit,
};

static struct state state_error = { 
    .data = "ERROR",
    .action_entry = &print_msg_err,
};

/* post process start */
static void state_post_enter( void *state_data, struct event *event )
{
    print_msg_enter(state_data, event);
    log_i("post start..."); 
}

static void action_post_break( void *oldstate_data, struct event *event,
      void *state_new_data )
{
    log_i("post break,display break...");
}

static bool guard_post_pass( void *condition, struct event *event )
{
    if(event->type != EVENT_POST_ANSWER)
    {
        return false;
    }
    
    return ((int)event->data == (int)condition);
}

static void action_post_pass( void *oldstate_data, struct event *event,
      void *state_new_data )
{
    log_i("post pass,display pass...");
}

static bool guard_post_fail( void *condition, struct event *event )
{
    if(event->type != EVENT_POST_ANSWER)
    {
        return false;
    }
    
    return ((int)event->data == (int)condition);
}

static void action_post_fail( void *oldstate_data, struct event *event,
      void *state_new_data )
{
    log_i("post fail,display fail...");
}
/* post process end */

static void print_msg_err( void *state_data, struct event *event )
{
    log_e( "entered error state!" );
}

static void print_msg_enter( void *state_data, struct event *event )
{
    log_i( "entering %s state", (char *)state_data );
}

static void print_msg_exit( void *state_data, struct event *event )
{
    log_i( "Eexiting %s state", (char *)state_data );
}

static rt_mq_t mq_event_post;
static struct state_machine m_post;

int state_post_event_set(enum event_post_type event, void* data)
{
    struct event e;

    RT_ASSERT(event < EVENT_POST_NUMS);
	
    e.type = event;
    e.data = data;
    
    return rt_mq_send(mq_event_post, &e, sizeof(struct event));
}

static void state_process(void *parameter)
{
    struct event e;
    statem_init( &m_post, &state_root, &state_error );

    while(1)
    {
        if(RT_EOK == rt_mq_recv(mq_event_post, &e, sizeof(struct event), 20))
        {
            statem_handle_event( &m_post, &(struct event){ e.type, e.data } );
        }
    }
} 

static int state_post_init(void)
{
    rt_thread_t tid = RT_NULL;

    mq_event_post = rt_mq_create("event_post",sizeof(struct event), 16, RT_IPC_FLAG_FIFO);

    tid = rt_thread_create("state_post", state_process, RT_NULL, 1024, 10, 100);
    if (tid == RT_NULL)
    {
        rt_kprintf("state post initialize failed! thread create failed!\r\n");
        
        return -RT_ENOMEM;
    }
    rt_thread_startup(tid);

    return RT_EOK;
}
INIT_APP_EXPORT(state_post_init);


#ifdef FINSH_USING_MSH
static void post_event_set(uint8_t argc, char **argv) 
{
    struct event e;
    
    if(argc < 2)
    {
        rt_kprintf("state post event set <event> <data>\n");
    }
    else
    {
        const char *operator = argv[1];
        
        if (argc == 3)
        {
            e.data = (void*)atoi(argv[2]);
        }
        else
        {
            e.data = RT_NULL;
        }

        if (!rt_strcmp(operator, "start"))
        {
            e.type = EVENT_POST_START;
        }
        else if (!rt_strcmp(operator, "breakon"))
        {
            e.type = EVENT_POST_BREAKON;
        }
        else if (!rt_strcmp(operator, "breakoff"))
        {
            e.type = EVENT_POST_BREAKOFF;
        }
        else if (!rt_strcmp(operator, "answer"))
        {
            e.type = EVENT_POST_ANSWER;
        }
        else
        {
            rt_kprintf("state key set:%s\n",argv[1]);
            return;
        }
        
        state_post_event_set(e.type, e.data);
    }
}
MSH_CMD_EXPORT(post_event_set, state post event set <event> <data>.);

static void post_current_get(uint8_t argc, char **argv) 
{
    if(!m_post.state_current->data)
    {
        rt_kprintf("post current state is NULL\n");
    }
    else
    {
        rt_kprintf("post current state is %s\n",m_post.state_current->data);
    }
}
MSH_CMD_EXPORT(post_current_get, get current state.);

#endif /* FINSH_USING_MSH */

