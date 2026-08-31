/*
 * hmac_sha256.h - HMAC-SHA256 (RFC 2104) over the vendored Brad Conte SHA-256.
 *
 * Used to verify MeshCore group-channel MACs (encrypt-then-MAC) WITHOUT
 * decrypting: the 2-byte channel MAC is the first two bytes of
 * HMAC-SHA256(key = 32-byte channel secret, msg = ciphertext).  This lets the
 * repeater prove a group message belongs to a configured PUBLIC channel using
 * only SHA-256 (no AES).
 */
#ifndef MC_HMAC_SHA256_H
#define MC_HMAC_SHA256_H

#include <stdint.h>
#include <stddef.h>

/* Compute HMAC-SHA256(key, msg) into out (32 bytes). */
void hmac_sha256(const uint8_t *key, size_t keylen,
                 const uint8_t *msg, size_t msglen,
                 uint8_t out[32]);

#endif /* MC_HMAC_SHA256_H */
