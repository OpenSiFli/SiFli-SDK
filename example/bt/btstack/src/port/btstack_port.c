#include <stdio.h>

#include "btstack_config.h"

#include <btstack.h>
#include <btstack_debug.h>
#include <btstack_event.h>
#include <btstack_memory.h>
#include <btstack_run_loop.h>
#include <btstack_stdin.h>
#include <btstack_tlv.h>
#include <btstack_tlv_flash_bank.h>

#include <ble/le_device_db_tlv.h>
#include <bluetooth.h>
#include <classic/btstack_link_key_db_tlv.h>
#include <hal_time_ms.h>
#include <hci.h>
#include <hci_dump.h>
#include <hci_dump_embedded_stdout.h>

#include <finsh.h>
#include <ipc_queue.h>
#include <rtthread.h>

#include "btstack_nvds.h"
#include "btstack_run_loop_rtthread.h"

void (*transport_packet_handler)(uint8_t packet_type, uint8_t *packet,
                                 uint16_t size);
void (*port_stdin_handler)(char c);
// data source for integration with BTstack Runloop
static btstack_data_source_t transport_data_source;
extern rt_mailbox_t to_btstack;
extern ipc_queue_handle_t ipc_port;
int remain_size = 0;

__ALIGNED(4) uint8_t hci_tmp[1024];

static void transport_notify_packet_send()
{
    // notify upper stack that it might be possible to send again
    uint8_t event[] = {HCI_EVENT_TRANSPORT_PACKET_SENT, 0};
    transport_packet_handler(HCI_EVENT_PACKET, &event[0], sizeof(event));
    return;
}

static void transport_notify_ready(void)
{
    // notify upper stack that it transport is ready
    uint8_t event[] = {HCI_EVENT_TRANSPORT_READY, 0};
    transport_packet_handler(HCI_EVENT_PACKET, &event[0], sizeof(event));
    return;
}

uint32_t hal_time_ms(void)
{
    return rt_tick_get() * (1000 / RT_TICK_PER_SECOND);
}

static void transport_deliver_hci_packets(void)
{
    rt_uint32_t size = remain_size;
    if (remain_size == 0)
    {
        rt_mb_recv(to_btstack, &size, RT_WAITING_FOREVER);
        if (!size)
            return;
    }

    uint8_t *ptr = hci_tmp;
    int read_len = ipc_queue_read(ipc_port, ptr, 5);
    if (read_len >= 5)
    {
        if (ptr[0] == HCI_EVENT_PACKET)
        {
            read_len = ptr[2] + 3;
        }
        else if (ptr[0] == HCI_ACL_DATA_PACKET || ptr[0] == HCI_ISO_DATA_PACKET)
        {
            read_len = *(uint16_t *)(ptr + 3) + 5;
        }
        else
        {
            read_len = size;
        }
        ipc_queue_read(ipc_port, ptr + 5, read_len - 5);
    }
    else if (read_len < 3)
    {
        return;
    }

    transport_packet_handler(ptr[0], ptr + 1, read_len - 1);

    remain_size = size - read_len;
    if (remain_size)
    {
        btstack_run_loop_poll_data_sources_from_irq();
    }
}
static void transport_process(btstack_data_source_t *ds,
                              btstack_data_source_callback_type_t callback_type)
{
    switch (callback_type)
    {
    case DATA_SOURCE_CALLBACK_POLL:
        transport_notify_ready();
        transport_deliver_hci_packets();
        break;
    default:
        break;
    }
}
/**
 * init transport
 * @param transport_config
 */
static void transport_init(const void *transport_config)
{
    btstack_run_loop_set_data_source_handler(&transport_data_source,
                                             &transport_process);
    btstack_run_loop_enable_data_source_callbacks(&transport_data_source,
                                                  DATA_SOURCE_CALLBACK_POLL);
    btstack_run_loop_add_data_source(&transport_data_source);
    return;
}

/**
 * open transport connection
 */
int transport_open(void)
{
    return 0;
}

/**
 * close transport connection
 */
int transport_close(void)
{
    return 0;
}

/**
 * register packet handler for HCI packets: ACL, SCO, and Events
 */
void transport_register_packet_handler(void (*handler)(uint8_t packet_type,
                                                       uint8_t *packet,
                                                       uint16_t size))
{
    transport_packet_handler = handler;
}

/**
 * support async transport layers, e.g. IRQ driven without buffers
 */
int transport_can_send_packet_now(uint8_t packet_type)
{
    return 1;
}

int send_to_malibox(uint8_t *ptr, int size);

/**
 * send packet
 */
int transport_send_to_controller(uint8_t packet_type, uint8_t *packet, int size)
{
    log_info("%s write %d", __func__, size + sizeof(packet_type));
    send_to_malibox(&packet_type, 1);
    send_to_malibox(packet, size);
    transport_notify_packet_send();
    return 0;
}

static const hci_transport_t transport = {
    "SiFli_HCI",
    &transport_init,
    &transport_open,
    &transport_close,
    &transport_register_packet_handler,
    &transport_can_send_packet_now,
    &transport_send_to_controller,
    NULL, // set baud rate
    NULL, // reset link
    NULL, // set SCO config
};
static const hci_transport_t *transport_get_instance(void)
{
    return &transport;
}

static btstack_packet_callback_registration_t hci_event_callback_registration;
static void local_version_information_handler(uint8_t *packet)
{
    printf("Local version information:\n");
    uint16_t hci_version = packet[6];
    uint16_t hci_revision = little_endian_read_16(packet, 7);
    uint16_t lmp_version = packet[9];
    uint16_t manufacturer = little_endian_read_16(packet, 10);
    uint16_t lmp_subversion = little_endian_read_16(packet, 12);
    printf("- HCI Version    %#04x\n", hci_version);
    printf("- HCI Revision   %#04x\n", hci_revision);
    printf("- LMP Version    %#04x\n", lmp_version);
    printf("- LMP Subversion %#04x\n", lmp_subversion);
    printf("- Manufacturer   %#04x\n", manufacturer);
}

static const btstack_tlv_t btstack_tlv_impl = {
    .get_tag = &port_get_tag,
    .store_tag = &port_store_tag,
    .delete_tag = &port_delete_tag,
};

static bd_addr_t local_addr = {0};
static void packet_handler(uint8_t packet_type, uint16_t channel,
                           uint8_t *packet, uint16_t size)
{
    const uint8_t *params;
    if (packet_type != HCI_EVENT_PACKET)
        return;
    switch (hci_event_packet_get_type(packet))
    {
    case BTSTACK_EVENT_STATE:
        switch (btstack_event_state_get_state(packet))
        {
        case HCI_STATE_WORKING:
            printf("BTstack up and running on %s.\n",
                   bd_addr_to_str(local_addr));
            // setup global tlv
            btstack_tlv_set_instance(&btstack_tlv_impl, NULL);

            hci_set_link_key_db(
                btstack_link_key_db_tlv_get_instance(&btstack_tlv_impl, NULL));
            // setup LE Device DB using TLV
            le_device_db_tlv_configure(&btstack_tlv_impl, NULL);
            break;
        case HCI_STATE_OFF:
            printf("Good bye, see you.\n");
            break;
        default:
            break;
        }
        break;
    case HCI_EVENT_COMMAND_COMPLETE:
        switch (hci_event_command_complete_get_command_opcode(packet))
        {
        case HCI_OPCODE_HCI_READ_LOCAL_VERSION_INFORMATION:
            local_version_information_handler(packet);
            break;
        case HCI_OPCODE_HCI_READ_BD_ADDR:
            params = hci_event_command_complete_get_return_parameters(packet);
            if (params[0] != 0)
                break;
            if (size < 12)
                break;
            reverse_48(&params[1], local_addr);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void cmd_btstack(int argc, char **argv)
{
    if (argc > 1 && port_stdin_handler != NULL)
    {
        port_stdin_handler(argv[1][0]);
    }
}

MSH_CMD_EXPORT_ALIAS(cmd_btstack, btstack,btstack stdin);
void btstack_stdin_setup(void (*stdin_handler)(char c))
{
    port_stdin_handler = stdin_handler;
}
extern int btstack_main(int argc, const char *argv[]);
void btstack_thread(void *args)
{
    // hci_dump_init(hci_dump_embedded_stdout_get_instance());

    /// GET STARTED with BTstack ///
    btstack_memory_init();
    btstack_run_loop_init(btstack_run_loop_rtthread_get_instance());

    // init HCI
    hci_init(transport_get_instance(), NULL);

    // inform about BTstack state
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    btstack_main(0, NULL);

    log_info("btstack executing run loop...");
    btstack_run_loop_execute();
}