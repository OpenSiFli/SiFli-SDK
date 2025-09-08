/*
 *      Copyright (C) 2020 Apple Inc. All Rights Reserved.
 *
 *      Find My Network ADK is licensed under Apple Inc.’s MFi Sample Code License Agreement,
 *      which is contained in the License.txt file distributed with the Find My Network ADK,
 *      and only to those who accept that license.
 */

/*
 * WolfSSL shim, providing the core cryptographic functionality
 * required to implement the FindMy protocol.
 */

#ifndef FM_CRYPTO_H
#define FM_CRYPTO_H

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/types.h>
#include <wolfssl/wolfcrypt/ecc.h>

typedef struct fm_crypto_ckg_context
{
    ecc_key key;
    byte r1[32];
    byte r2[32];
    ecc_point *p;
} *fm_crypto_ckg_context_t;

/*! @function fm_crypto_sha256
 @abstract Hashes a given message using SHA-256.

 @param msg_nbytes Byte length of message.
 @param msg        Message to hash.
 @param out        32-byte output buffer for SHA-256 digest.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_sha256(word32 msg_nbytes, const byte *msg, byte out[32]);

/*! @function fm_crypto_ckg_init
 @abstract Initializes a given collaborative key generation context.

 @param ctx Collaborative key generation context.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_ckg_init(fm_crypto_ckg_context_t ctx);

/*! @function fm_crypto_ckg_gen_c1
 @abstract Generates message C1.

 @param ctx Collaborative key generation context.
 @param out 32-byte output buffer for C1.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_ckg_gen_c1(fm_crypto_ckg_context_t ctx, byte out[32]);

/*! @function fm_crypto_ckg_gen_c3
 @abstract Generates message C3.

 @param ctx Collaborative key generation context.
 @param c2  89-byte message C2 from the Apple device.
 @param out 60-byte output buffer for C3.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_ckg_gen_c3(fm_crypto_ckg_context_t ctx,
                         const byte c2[89],
                         byte out[60]);

/*! @function fm_crypto_ckg_finish
 @abstract Finalizes collaborative key generation.

 @param ctx Collaborative key generation context.
 @param p   57-byte output buffer for final public key P.
 @param skn 32-byte output buffer for symmetric key SKN.
 @param sks 32-byte output buffer for symmetric key SKS.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_ckg_finish(fm_crypto_ckg_context_t ctx,
                         byte p[57],
                         byte skn[32],
                         byte sks[32]);

/*! @function fm_crypto_ckg_free
 @abstract Frees a given collaborative key generation context.

 @param ctx Collaborative key generation context.
 */
void fm_crypto_ckg_free(fm_crypto_ckg_context_t ctx);

/*! @function fm_crypto_generate_seedk1
 @abstract Generates SeedK1.

 @param out 32-byte output buffer for SeedK1.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_generate_seedk1(byte out[32]);

/*! @function fm_crypto_derive_server_shared_secret
 @abstract Derives the ServerSharedSecret.

 @param seeds  32-byte unique server seed for pairing.
 @param seedk1 32-byte encryption key seed for pairing.
 @param out    32-byte output buffer for ServerSharedSecret.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_derive_server_shared_secret(const byte seeds[32],
        const byte seedk1[32],
        byte out[32]);

/*! @function fm_crypto_authenticate_with_ksn
 @abstract Authenticates a given message using the KSN.

 @param serverss   32-byte ServerSharedSecret
 @param msg_nbytes Byte length of message.
 @param msg        Message.
 @param out        32-byte output buffer for MAC.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_authenticate_with_ksn(const byte serverss[32],
                                    word32 msg_nbytes,
                                    const byte *msg,
                                    byte out[32]);

/*! @function fm_crypto_encrypt_to_server
 @abstract Encrypt a message to the Apple server.

 @param pub        Apple server encryption key in X9.63 format.
 @param msg_nbytes Byte length of message.
 @param msg        Message to encrypt.
 @param out_nbytes Pointer to length of output buffer.
                   (MUST be at least 65 + msg_nbytes + 16.)
 @param out        Output buffer for ciphertext.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_encrypt_to_server(const byte pub[65],
                                word32 msg_nbytes,
                                const byte *msg,
                                word32 *out_nbytes,
                                byte *out);

/*! @function fm_crypto_verify_s2
 @abstract Verifies signature S2 received from the server.

 @param pub        Apple server signature verification key in X9.63 format.
 @param sig_nbytes Byte length of the signature.
 @param sig        Signature over message.
 @param msg_nbytes Byte length of message to verify.
 @param msg        Message to verify.

 @return 0 if the signature is valid, a negative value otherwise.
 */
int fm_crypto_verify_s2(const byte pub[65],
                        word32 sig_nbytes,
                        const byte *sig,
                        word32 msg_nbytes,
                        const byte *msg);

/*! @function fm_crypto_decrypt_e3
 @abstract Decrypts server message E3.

 @param serverss   32-byte ServerSharedSecret
 @param e3_nbytes  Byte length of message E3.
 @param e3         Message E3.
 @param out_nbytes Pointer to length of output buffer.
                   (MUST be at least e3_nbytes - 16.)
 @param out        Output buffer for plaintext.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_decrypt_e3(const byte serverss[32],
                         word32 e3_nbytes,
                         const byte *e3,
                         word32 *out_nbytes,
                         byte *out);

/*! @function fm_crypto_roll_sk
 @abstract Computes SK_i+1 from a given SK_i. SK can be SKN or SKS.

 @param sk  32-byte symmetric key SKN_i or SKS_j.
 @param out 32-byte output buffer for SKN_i+1 or SKS_j+1.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_roll_sk(const byte sk[32], byte out[32]);

/*! @function fm_crypto_derive_ltk
 @abstract Derives LTK_i from a given SKN_i.

 @param skn 32-byte symmetric key SKN_i.
 @param out 16-byte output buffer for LTK_i.

 @return 0 on success, a negative value on error.
 */
int fm_crypto_derive_ltk(const byte skn[32], byte out[16]);

/*! @function fm_crypto_derive_primary_or_secondary_x
 @abstract Derives a primary key P_i or a secondary key PW_j.

 @param sk  32-byte symmetric key SKN_i or SKS_j.
 @param p   57-byte public key P as generated at pairing.
 @param out 28-byte output buffer for x(P_i) or x(PW_j).

 @return 0 on success, a negative value on error.
 */
int fm_crypto_derive_primary_or_secondary_x(const byte sk[32],
        const byte p[57],
        byte out[28]);

#endif // FM_CRYPTO_H
