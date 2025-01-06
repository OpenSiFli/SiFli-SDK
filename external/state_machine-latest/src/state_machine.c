/* 
 * Copyright (c) 2013 Andreas Misje
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "state_machine.h"

static void go_to_state_error( struct state_machine *state_machine,
      struct event *const event );
static struct transition *get_transition( struct state_machine *state_machine,
      struct state *state, struct event *const event );

int statem_init( struct state_machine *fsm,
      struct state *state_init, struct state *state_error )
{
    if ( !fsm )
    {
        return -1;
    }

    fsm->state_current = state_init;
    fsm->state_previous = NULL;
    fsm->state_error = state_error;
      
    return 0;      
}

int statem_handle_event( struct state_machine *fsm, struct event *event )
{
    if ( !fsm || !event )
    {
        return STATEM_ERR_ARG;
    }

    if ( !fsm->state_current )
    {
        go_to_state_error( fsm, event );
        return STATEM_ERR_STATE_RECHED;
    }

    /* 如果这个状态没有儿子，不用转移了,那他自己是儿子吗 */
    if ( (!fsm->state_current->transition_nums) && (!fsm->state_current->state_parent))
    {
        return STATEM_STATE_NOCHANGE;
    }

    struct state *state_next = fsm->state_current;
    
    do {
        struct transition *transition = get_transition( fsm, state_next, event );

        /* If there were no transitions for the given event for the current
        * state, check if there are any transitions for any of the parent
        * states (if any): */
        /*  continue并不会跳过while的条件判断 */
        if ( !transition )
        {
            state_next = state_next->state_parent;
            continue;
        }

        /* A transition must have a next state defined. If the user has not
        * defined the next state, go to error state: */
        if ( !transition->state_next )
        {
            go_to_state_error( fsm, event );
            return STATEM_ERR_STATE_RECHED;
        }

        state_next = transition->state_next;

        /* If the new state is a parent state, enter its entry state (if it has
        * one). Step down through the whole family tree until a state without
        * an entry state is found: */
        while ( state_next->state_entry )
        {
            state_next = state_next->state_entry;
        }

        /* Run exit action only if the current state is left (only if it does
        * not return to itself): */
        if ( state_next != fsm->state_current && fsm->state_current->action_exti )
        {
            fsm->state_current->action_exti( fsm->state_current->data, event );
        }

        /* Run transition action (if any): */
        if ( transition->action )
        {
            transition->action( fsm->state_current->data, event, state_next->data );
        }

        fsm->state_previous = fsm->state_current;
        
        /* Call the new state's entry action if it has any (only if state does
        * not return to itself): */
        if ( state_next != fsm->state_current && state_next->action_entry )
        {
            state_next->action_entry( state_next->data, event );
        }

        fsm->state_current = state_next;

        /* If the state returned to itself: */
        if ( fsm->state_current == fsm->state_previous )
        {
            return STATEM_STATE_LOOPSELF;
        }

        if ( fsm->state_current == fsm->state_error )
        {
            return STATEM_ERR_STATE_RECHED;
        }

        /* If the new state is a final state, notify user that the state
        * machine has stopped: */
        /* 下一个状态没有儿子,也没有父亲，你们家只有你了 */
        if ((!fsm->state_current->transition_nums) && (!fsm->state_current->state_parent))
        {
            return STATEM_FINAL_STATE_RECHED;
        }

        return STATEM_STATE_CHANGED;
    } while ( state_next );

    return STATEM_STATE_NOCHANGE;
}

struct state *statem_state_current( struct state_machine *fsm )
{
    if ( !fsm )
    {
        return NULL;
    }

    return fsm->state_current;
}

struct state *statem_state_previous( struct state_machine *fsm )
{
    if ( !fsm )
    {
        return NULL;
    }

    return fsm->state_previous;
}

static void go_to_state_error( struct state_machine *fsm,
      struct event *const event )
{
    fsm->state_previous = fsm->state_current;
    fsm->state_current = fsm->state_error;

    /* 本地错误状态要执行进入，进入错误状态肯定是数据设置错误，不是状态机不符合逻辑 */
    if ( fsm->state_current && fsm->state_current->action_entry )
    {
        fsm->state_current->action_entry( fsm->state_current->data, event );
    }
}

static struct transition *get_transition( struct state_machine *fsm,
      struct state *state, struct event *const event )
{
    size_t i;

    if( !state )
    {
        return NULL;
    }

    for ( i = 0; i < state->transition_nums; ++i )
    {
        struct transition *t = &state->transitions[ i ];

        /* A transition for the given event has been found: */
        if ( t->event_type == event->type )
        {
            /* 轮询监视，如果没有监视，或者监视测试通过返回这个转移
             * 监视输入条件和事件，可以做成当前转移的条件要和事件的数据吻合
             * 也可以做成当前条件要符合什么，事件的数据参数要符合什么
             */
            if ( !t->guard )
            {
                return t;
            }
            /* If transition is guarded, ensure that the condition is held: */
            else if ( t->guard( t->condition, event ) )
            {
                return t;
            }
        }
    }

    /* No transitions found for given event for given state: */
    return NULL;
}

int statem_stopped( struct state_machine *state_machine )
{
    if ( !state_machine )
    {
        return -1;
    }

    return (state_machine->state_current->transition_nums == 0);
}
