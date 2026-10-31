# MT6797 gen3 port checkpoint

The protocol core comes from the project-local MediaTek 3.18 BSP at
`X20-96-board/kernel-3.18/drivers/misc/mediatek/connectivity/wlan/gen3`.
AIS/SAA, static AP/P2P roles and firmware CCMP remain in that framework;
Linux 6.18 cfg80211 and the owned ConSys/WMT/HIF providers supply the host API.
Firmware uses request_firmware. NVRAM accepts a checked 512-byte NVMEM record
or read-only firmware fallback. The official default record is not this
board's factory calibration. Workspace tests derive and persist distinct
board-assigned WiFi/BT addresses from eMMC CID while preserving other bytes.

## Provider ownership and startup

- `mt6797-consys` owns the shared connectivity power, its regulators and
  reserved EMI window. Its STP host claim serializes control of the shared
  block; Wi-Fi and BT have separate VCN33 function supplies.
- The first `mt6797-wmt` client opens the ConSys host, starts a BTIF-backed
  STP session, loads the WMT patches and runs firmware initialization. WMT
  serializes client power changes and retains the session until the last
  client closes. BT uses WMT function control; the WLAN AHB probe owns its
  own HIF function enable. RX client lifetime is protected by SRCU.
- `mt6797-wlan-hif` owns WLAN's AHB PIO/DMA and IRQ claim; the gen3 WLAN
  adapter uses that provider and the WMT client, rather than directly owning
  the shared ConSys power sequence. `btmt6797` is a separate WMT client and
  exposes the Bluetooth HCI device.
- On last close, WMT stops the STP session before releasing the ConSys host.
  The patch binaries, WLAN firmware, NVRAM/fallback record and board-derived
  radio identities are distinct inputs; replacing one with another chip's
  firmware or a generic calibration record is not an equivalent cleanup.

On 2026-10-31 the X20 board also completed a live Bluetooth discovery while
six WLAN ICMP requests to 192.168.31.50 succeeded; HCI remained up after
discovery. See `out/headless-power-next/wlan-bt-coexist.log` in the workspace.
This covers concurrent discovery and WLAN reachability, not a Bluetooth data
connection or every coexistence mode.

## Implemented and tested on 2026-10-18

- Standard dual-band STA, WPA2/CCMP/PMF, HT20/HT40/VHT80 and static AP have
  real X20/Q6A data evidence. VHT wiphy capabilities and air IEs share policy:
  NSS1 MCS0..9, remaining streams unsupported; no 160/80+80 declaration.
- AHB FIFO APDMA uses an aligned coherent bounce buffer, actual physical
  address bit32, bounded fast polling plus slow completion/reset. The X20
  storage DT selects it at probe. Diagnostic DTs keep PIO. The sysfs
  `180f0000.wifi/transfer_mode` control changes mode only between sessions.
  Dual DMA with BTIF passed each-direction 210-second SHA streams at about
  145.9/118.4 Mbps on the Q6A VHT80 fixture, with zero DMA faults.
- Native BTIF DMA/VFIFO, standard HCI, encrypted BR/EDR L2CAP and BLE
  central/peripheral GATT passed with independent peer SHA checks and WLAN
  load. BlueZ uses the qualified 4000-ms LE supervision policy; the original
  420-ms RF/coexistence gap is not claimed fixed. Firmware DLE/privacy/ECC
  event limitations remain; do not enable unsupported feature bits.
- WMT coordinates bounded shared recovery across both consumers, power-off,
  firmware/calibration and re-probe. Native udev/SysV restores association,
  fixed IP and HCI. Manual and real-send provider-error injection passed,
  followed by WiFi and saved-pair GATT data. This is not a DMA-hardware-hang
  or every firmware-fault qualification.
- STP/WMT PSM performs firmware sleep/host-ack, the real BTIF wake edge and
  BGF peer wake. It drains TX, masks/synchronizes IRQ, retains VFIFO storage,
  releases this consumer's clocks and restores clocks before wake protocol.
  Clock-off peer wake plus 180-second encrypted GATT/WLAN overlap passed.
  Native integration selects PSM auto after association. Shared APDMA may
  remain clocked for other consumers; system suspend/CPU_OFF is not qualified.

## WPA3-Personal STA

The user explicitly added SAE to the scope on 2026-10-18, superseding its
previous exclusion. Standard cfg80211 external_auth lets unmodified
wpa_supplicant 2.10 perform SAE; management frames use standard mgmt_tx/rx.
Completion returns through the adapter mailbox and checks JOIN/state,
BSSID/SSID and STA lifetime. Bounded timeout/cancel sends the original auth
identity back to userspace. RSNXE H2E and optional PMKID are bounded/copied;
firmware retains its RSN/CCMP/key-installation ABI, with the actual SAE RSNE.

`sae_external_auth=1` is the default; zero disables the STA-only SAE feature.
Only RSN/CCMP with required PMF and userspace external_auth is accepted.
AP/P2P do not advertise SAE. No FT-SAE, SAE-PK or WPA3-Enterprise expansion.
Q6A SAE-only/PMF and Xiaomi_F5F6 H2E/Group19 passed; wrong password rejected,
three uncached reconnects passed, and each-direction 180-second streams
matched full SHA (about 187.2/51.8 Mbps on that laboratory AP).

Evidence and the final boot-integration state are in the workspace
`docs/bsp/20261018-alignment/{DMA,RECOVERY,SAE,STORAGE}.md` and release manifests.
A successful module load is not proof that a device bound. Retain UART/RAM
rescue when replacing built-in providers, DTs or a different kernel ABI.

Remaining qualifications include P2P negotiation/concurrency, roaming/TDLS,
DFS roles, additional media/thermal conditions and system PM's ATF prerequisite.
Dynamic VIF, AP station dump/statistics and scheduled scan are BSP interface
limitations or optional integration, not enabled by these changes. Unsafe
raw diagnostic mappings remain unavailable without an owned mapping.
