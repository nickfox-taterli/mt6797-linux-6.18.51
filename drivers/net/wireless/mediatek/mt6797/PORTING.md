# MT6797 gen3 development checkpoint

The protocol core comes from the project-local MediaTek 3.18 BSP at
`X20-96-board/kernel-3.18/drivers/misc/mediatek/connectivity/wlan/gen3`.
Keep its STA, AP, P2P, security and management implementations while adapting
the selected Linux/AHB path to 6.18. This is an experimental driver.

The native AHB glue uses the owned ConSys/WMT/HIF providers. Firmware loading
uses request_firmware; NVRAM uses a 512-byte NVMEM cell, with an explicit
read-only firmware fallback. Opaque WMT calibration is retained by the shared
provider. Mainline cfg80211, timer, wakeup, file, proc and frequency-QoS APIs
replace the corresponding obsolete platform calls.

Verified on 2026-10-17: ARM64 module linking; bounded firmware validation,
NVRAM and PIO fault tests; board WMT initialization/calibration, HIF claim,
two IDLM download acknowledgements, bounded EMI writes and failure teardown.
Module unload works after a failed probe. Wi-Fi Start is sent, but WCIR stays
0x00100279 with mailbox0=10 and firmware-ready times out. There is no verified
wlan0, scan, association or data traffic yet. Module load returning zero does
not imply the platform device bound successfully.

An opt-in AP-DMA backend now preserves the dedicated MT6797 BSP channel
programming, using a coherent bounce buffer, bounded completion/reset and
failure propagation. PIO remains the default; `180f0000.wifi/transfer_mode`
can select DMA only between WLAN sessions. Real-function fault tests and
ARM64 compilation pass. Board DMA completed 13 FIFO transfers with valid
configuration ACKs, but firmware-ready still timed out, as with PIO in the
same kernel. Failure teardown returned the channel and ConSys power. This
is not evidence of firmware execution or wireless traffic.

Remaining work includes firmware startup, data queues/interrupts, AP-DMA validation,
dynamic virtual interfaces, standard userspace role tests, factory calibration
provenance, shared fault recovery and power management. AP/P2P static paths
are retained but unverified. Unsafe raw diagnostic mappings remain unavailable
until they have an owned mapping. Do not treat this checkpoint as a release.
