# ed25519

Orson Peters' Ed25519 (https://github.com/orlp/ed25519), vendored unmodified
from the copy MeshCore itself uses, so adverts this firmware signs verify
against stock MeshCore nodes and against the RF.Guru meshcore-repeater.

`seed.c`, `key_exchange.c` and `add_scalar.c` are removed: seed.c pulls in
Win32/stdio randomness that does not exist here (the seed comes from the
RP2040's ROSC instead, see src/meshcore.cpp), and the other two are unused.
Build with `-DED25519_NO_SEED`.

Measured on this board, RP2040 at 133 MHz:

| operation      | time    |
|----------------|---------|
| create_keypair | 36.5 ms |
| sign (57 B)    | 39.3 ms |
| verify         | 97.3 ms |

Signing at 39 ms sits comfortably inside the 5 s watchdog, so an advert needs
no special handling. Verify is unused - this firmware only transmits.
