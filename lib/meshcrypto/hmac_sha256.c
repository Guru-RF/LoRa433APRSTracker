/* hmac_sha256.c - HMAC-SHA256 built on the vendored Brad Conte SHA-256. */
#include "hmac_sha256.h"
#include "sha256.h"

#include <string.h>

/* SHA-256 processes 64-byte blocks. NB: the vendored header's SHA256_BLOCK_SIZE
 * is 32 (the DIGEST size, misnamed) - do not use it as the HMAC block size. */
#define HMAC_BLOCK  64
#define SHA256_LEN  32

void hmac_sha256(const uint8_t *key, size_t keylen,
                 const uint8_t *msg, size_t msglen,
                 uint8_t out[32])
{
    uint8_t k[HMAC_BLOCK];
    uint8_t ipad[HMAC_BLOCK], opad[HMAC_BLOCK];
    uint8_t inner[SHA256_LEN];
    SHA256_CTX ctx;

    /* keys longer than the block are hashed down; shorter keys are zero-padded */
    memset(k, 0, sizeof(k));
    if (keylen > HMAC_BLOCK) {
        sha256_init(&ctx);
        sha256_update(&ctx, key, keylen);
        sha256_final(&ctx, k);        /* first 32 bytes set, remainder stays zero */
    } else {
        memcpy(k, key, keylen);
    }

    for (int i = 0; i < HMAC_BLOCK; i++) {
        ipad[i] = (uint8_t)(k[i] ^ 0x36);
        opad[i] = (uint8_t)(k[i] ^ 0x5c);
    }

    /* inner = H(ipad || msg) */
    sha256_init(&ctx);
    sha256_update(&ctx, ipad, HMAC_BLOCK);
    sha256_update(&ctx, msg, msglen);
    sha256_final(&ctx, inner);

    /* out = H(opad || inner) */
    sha256_init(&ctx);
    sha256_update(&ctx, opad, HMAC_BLOCK);
    sha256_update(&ctx, inner, SHA256_LEN);
    sha256_final(&ctx, out);
}
