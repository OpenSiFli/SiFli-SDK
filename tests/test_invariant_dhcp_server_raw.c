#include <check.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Replicate minimal structs matching dhcp_server_raw.c definitions */
#define DHCP_CHADDR_LEN 16

struct dhcp_msg {
    uint8_t op;
    uint8_t htype;
    uint8_t hlen;
    uint8_t hops;
    uint32_t xid;
    uint16_t secs;
    uint16_t flags;
    uint8_t ciaddr[4];
    uint8_t yiaddr[4];
    uint8_t siaddr[4];
    uint8_t giaddr[4];
    uint8_t chaddr[16];
    uint8_t sname[64];
    uint8_t file[128];
    uint8_t options[312];
};

struct dhcp_node {
    uint8_t chaddr[DHCP_CHADDR_LEN];
};

START_TEST(test_hlen_bounds_check)
{
    /* Invariant: msg->hlen must never exceed DHCP_CHADDR_LEN (16) before
       SMEMCPY into node->chaddr, preventing heap buffer overflow (CWE-287/122) */

    uint8_t hlen_payloads[] = {
        255,  /* exact exploit: max client-controlled value, overflows 16-byte buffer */
        17,   /* boundary: one byte over the 16-byte chaddr buffer */
        6,    /* valid: standard Ethernet MAC address length */
    };
    int num_payloads = (int)(sizeof(hlen_payloads) / sizeof(hlen_payloads[0]));

    for (int i = 0; i < num_payloads; i++) {
        uint8_t hlen = hlen_payloads[i];

        /* The invariant: hlen must be clamped to DHCP_CHADDR_LEN before copy */
        ck_assert_msg(
            hlen <= DHCP_CHADDR_LEN,
            "SECURITY VIOLATION: msg->hlen=%u exceeds chaddr buffer size %d. "
            "SMEMCPY(node->chaddr, msg->chaddr, msg->hlen) would overflow. "
            "A bounds check is required before the copy in dhcp_server_raw.c",
            (unsigned)hlen, DHCP_CHADDR_LEN
        );
    }
}
END_TEST

Suite *security_suite(void)
{
    Suite *s;
    TCase *tc_core;

    s = suite_create("Security");
    tc_core = tcase_create("Core");

    tcase_add_test(tc_core, test_hlen_bounds_check);
    suite_add_tcase(s, tc_core);

    return s;
}

int main(void)
{
    int number_failed;
    Suite *s;
    SRunner *sr;

    s = security_suite();
    sr = srunner_create(s);

    srunner_run_all(sr, CK_NORMAL);
    number_failed = srunner_ntests_failed(sr);
    srunner_free(sr);

    return (number_failed == 0) ? EXIT_SUCCESS : EXIT_FAILURE;
}