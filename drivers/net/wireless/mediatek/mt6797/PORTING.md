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
Module unload works after a failed probe. The UART rescue Preloader must run the normal master-domain initialization:
without it ConSys cannot read its protected EMI firmware. With Preloader
fa68e7a, ROM reads match both EMI section headers and firmware starts;
wlan0/p2p0 are present. A standard iw scan returned 21 BSS entries, including
the laboratory SSIDs on 2432 and 5200 MHz. Laboratory 2.4-GHz WPA2/CCMP with
PMF, DHCP, build-host SSH, Q6A bidirectional ping and hash-verified SFTP in both
directions now pass. An initial upload reset is retained as a failure. After reserving the BSP secure
64 KiB DRAM area, ordinary SysV tests passed for laboratory 2.4-GHz STA,
5-GHz WPA2 STA to Q6A, and MTK AP to Q6A STA, each with 96 MiB of bidirectional
SHA-verified SFTP. A successful WLAN/WMT unload, full power-off and reload
passed, followed by three STA reconnects. Minute-scale bandwidth testing is
now in progress; first measurements exposed a missing BSP CPUxGPT divide-by-two
initialization, corrected in the RAM ATF before repeating duration tests. Module load returning zero does
not imply the platform device bound successfully.

An opt-in AP-DMA backend now preserves the dedicated MT6797 BSP channel
programming, using a coherent bounce buffer, bounded completion/reset and
failure propagation. PIO remains the default; `180f0000.wifi/transfer_mode`
can select DMA only between WLAN sessions. Real-function fault tests and
ARM64 compilation pass. Board DMA completed 13 FIFO transfers with valid
configuration ACKs, but firmware-ready still timed out, as with PIO in the
same kernel. Failure teardown returned the channel and ConSys power. This
is not evidence of firmware execution or wireless traffic.

The first netdev open exposed legacy direct dev_addr writes. STA, static P2P,
and retained BOW/role creation now use eth_hw_addr_set to maintain the kernel's
address list and shadow. ARM64 modules link; replacement and wlan0 open passed
without a new dev_addr warning. Warnings while unloading the preceding module
describe its old address state, not the replacement.

Remaining work includes sustained traffic/recovery, AP-DMA validation, dynamic
virtual interfaces, AP station statistics/dump, P2P negotiation, factory calibration
provenance, shared fault recovery and power management. Static AP has real peer
data evidence; this does not validate P2P negotiation. Native Bluetooth now has
real LE central pairing and GATT traffic while WLAN is associated, but fast BT
off/on exposed a shared firmware failure; concurrent load and lifecycle recovery
remain open. Unsafe raw diagnostic mappings remain unavailable
until they have an owned mapping. Do not treat this checkpoint as a release.
