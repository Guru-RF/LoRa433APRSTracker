# meshcrypto

AES-128, SHA-256 and HMAC-SHA256, vendored unmodified from
`../meshcore-repeater/src` so that a group message this firmware encrypts is
decrypted by exactly the code that will receive it.

Used only for MeshCore public-channel group text (`PAYLOAD_TYPE_GRP_TXT`),
where the channel key is `SHA256("#name")[0..15]`, the on-air channel selector
is `SHA256(key,16)[0]`, and the 2-byte MAC is `HMAC-SHA256(key, plaintext)`.

Note the asymmetry, which is easy to get wrong: AES takes the **16-byte** key,
the HMAC takes it **zero-padded to 32**. That mirrors `channel_build_txt()` in
the repeater's `src/mesh.c`.

Nothing here is used for the Ed25519 adverts - those are in `lib/ed25519`.
