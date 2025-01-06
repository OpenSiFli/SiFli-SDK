#ifndef __SECBOOT_H__
#define __SECBOOT_H__

#define SPLIT_THRESHOLD     (256)

#define SECBOOT_SIGKEY_PUB_ERR      1
#define SECBOOT_IMG_HASH_SIG_ERR    2

extern void boot_uart_tx(USART_TypeDef *uart, uint8_t *data, int len);
void sifli_secboot_exception(uint8_t excpt);
int sifli_sigkey_pub_verify(uint8_t *sigkey, uint32_t key_size);
int sifli_img_sig_hash_verify(uint8_t *img_hash_sig, uint8_t *sig_pub_key, uint8_t *image, uint32_t img_size);
#endif /* __SECBOOT_H__ */

