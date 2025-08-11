/* microbit.c - BBC micro:bit specific hooks */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ctype.h>
#include <stdlib.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/mesh.h>

#define BTN_COUNT 2

static struct k_work button_work;
extern void board_button_1_pressed(void);

static void button_send_pressed(struct k_work *work)
{
    printk("button_send_pressed()\n");
    board_button_1_pressed();
}

static void button_pressed(uint32_t pins)
{
    if (pins)
    {
        k_work_submit(&button_work);
    }
    else
    {
        printk("button_pressed pins %x %x\n", pins);
    }
}

static void mesh_btn(uint8_t argc, char **argv)
{
    if (argc > 1)
    {
        uint8_t sel = strtol(argv[1], NULL, 0);
        if (sel > BTN_COUNT || sel == 0)
            printk("Usage: mesh_btn [1]");
        else
            button_pressed(sel);
    }
}
MSH_CMD_EXPORT(mesh_btn, enter button for Mesh provisioner example);


void board_play_tune(const char *str)
{
    printk("board_play_tune:%s\n", str);
}

void board_heartbeat(uint8_t hops, uint16_t feat)
{
    printk("board_heartbeat: %u hops\n", hops);
}

void board_other_dev_pressed(uint16_t addr)
{
    printk("board_other_dev_pressed(0x%04x)\n", addr);

}

void board_attention(bool attention)
{
    printk("board_attention %d\n", attention);
}

int board_init(uint16_t *addr)
{
    printk("Board init\n");
    k_work_init(&button_work, button_send_pressed);
    return 0;
}
