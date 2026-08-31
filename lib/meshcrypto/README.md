# meshcrypto

AES-128, SHA-256 and HMAC-SHA256, vendored unmodified from
`../meshcore-repeater/src` so that a group message this firmware encrypts is
decrypted by exactly the code that will receive it.

Used only for MeshCore public-channel group text (`PAYLOAD_TYPE_GRP_TXT`),
where the channel key is `SHA256("#name")[0..15]`, the on-air channel selector
is `SHA256(key,16)[0]`, and the 2-byte MAC is `HMAC-SHA256(key, plaintext)`.

Two things here are easy to get wrong, and both mirror `channel_build_txt()`
in the repeater's `src/mesh.c`:

- **AES takes the 16-byte key; the HMAC takes it zero-padded to 32.**
- **It is encrypt-then-MAC.** The repeater encrypts in place and only then
  calls `hmac_sha256()` on the same buffer, so the MAC covers the
  *ciphertext* even though the call reads as though it covers the plaintext.
  Getting that backwards yields a packet that is received, matched to the
  right channel, and dropped with "failed public-channel MAC".

Nothing here is used for the Ed25519 adverts - those are in `lib/ed25519`.
