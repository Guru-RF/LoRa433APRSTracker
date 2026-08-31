/*
 * aes128.h - AES-128 in ECB mode.
 *
 * MeshCore group channels use AES-128-ECB (16-byte key = first 16 bytes of the
 * channel secret, 16-byte blocks, final partial block zero-padded). The
 * repeater needs this only for PUBLIC channels (published key) to read/compose
 * group-text content (e.g. ping/pong); private traffic is never decrypted.
 *
 * Implementation adapted from the public-domain tiny-AES-c (Unlicense).
 */
#ifndef MC_AES128_H
#define MC_AES128_H

#include <stdint.h>
#include <stddef.h>

typedef struct {
    uint8_t round_key[176];   /* 11 x 16-byte expanded round keys */
} aes128_ctx_t;

void aes128_init(aes128_ctx_t *ctx, const uint8_t key[16]);

/* Encrypt / decrypt a single 16-byte block in place. */
void aes128_encrypt_block(const aes128_ctx_t *ctx, uint8_t block[16]);
void aes128_decrypt_block(const aes128_ctx_t *ctx, uint8_t block[16]);

/* ECB over a buffer whose length is a multiple of 16 (in place). */
void aes128_ecb_encrypt(const aes128_ctx_t *ctx, uint8_t *buf, size_t len);
void aes128_ecb_decrypt(const aes128_ctx_t *ctx, uint8_t *buf, size_t len);

#endif /* MC_AES128_H */
