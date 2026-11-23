#!/usr/bin/env python3
"""
Automates Vitis Unified IDE Platform + FreeRTOS UDP Perf Client Workspace Creation
Target: Eclypse Z7 (Zynq-7000 ps7_cortexa9_0)
Run with: vitis -s create_vitis_udp_workspace.py
"""

import os
import vitis

# =============================================================================
# HARDCODED CONFIGURATION PARAMETERS
# =============================================================================
WORKSPACE_DIR = os.path.abspath("./vitis_workspace")
XSA_PATH      = os.path.abspath("./design_1_wrapper.xsa")

PLATFORM_NAME = "eclypse_z7_udp_platform"
DOMAIN_NAME   = "freertos_ps7_cortexa9_0"
APP_NAME      = "sdr_udp_perf_client"
CPU_NAME      = "ps7_cortexa9_0"
TEMPLATE_NAME = "freertos_lwip_udp_perf_client"

# FreeRTOS OS Settings
FREERTOS_HEAP_SIZE = "524288"   # 512 KB FreeRTOS heap

# Hardware Timer Settings (xiltimer BSP library)
TIMER_INSTANCE     = "ps7_ttc_0"  # If Vitis SDT reports an error, try "ttc0"

# lwIP 2.2.0 BSP Settings (Vitis set_config expects lowercase "true"/"false" strings)
LWIP_LIB_NAME = "lwip220"
LWIP_CONFIGS = {
    "lwip220_dhcp":                     "true",
    "lwip220_lwip_dhcp_does_acd_check": "true",
    "lwip220_pbuf_pool_size":           "16384",
    "lwip220_memp_n_pbuf":              "1024",
    "lwip220_mem_size":                 "524288",
    "lwip220_n_tx_descriptors":         "512",
}
# =============================================================================

client = vitis.create_client()
client.set_workspace(path=WORKSPACE_DIR)

# 1. Create Platform Component
platform = client.create_platform_component(
    name=PLATFORM_NAME,
    hw_design=XSA_PATH,
    os="freertos",
    cpu=CPU_NAME,
    domain_name=DOMAIN_NAME,
    no_boot_bsp=False,
)

# 2. Add FreeRTOS Domain pre-configured for the UDP Perf Client template
udp_domain_name = f"{DOMAIN_NAME}_udp"
platform.add_domain(
    cpu=CPU_NAME,
    os="freertos",
    name=udp_domain_name,
    display_name=udp_domain_name,
    support_app=TEMPLATE_NAME,
)
udp_domain = platform.get_domain(name=udp_domain_name)

# 3. Apply FreeRTOS Heap Size
if FREERTOS_HEAP_SIZE:
    udp_domain.set_config(
        option="os",
        param="freertos_total_heap_size",
        value=FREERTOS_HEAP_SIZE,
    )

# 4. Configure Hardware Timer (ps7_ttc_0) via xiltimer
#    Enables the interval timer and binds it to ps7_ttc_0 for FreeRTOS/lwIP ticks
udp_domain.set_config(
    option="lib",
    lib_name="xiltimer",
    param="XILTIMER_en_interval_timer",
    value="true",
)
udp_domain.set_config(
    option="lib",
    lib_name="xiltimer",
    param="XILTIMER_tick_timer",
    value=TIMER_INSTANCE,
)

# 5. Apply Custom lwip220 Settings
for param, val in LWIP_CONFIGS.items():
    udp_domain.set_config(
        option="lib",
        lib_name=LWIP_LIB_NAME,
        param=param,
        value=val,
    )

# 6. Build the Platform (generates .xpfm)
platform.build()

# 7. Create & Build the Application Component
xpfm_path = os.path.join(
    WORKSPACE_DIR, PLATFORM_NAME, "export", PLATFORM_NAME, f"{PLATFORM_NAME}.xpfm"
)
app = client.create_app_component(
    name=APP_NAME,
    platform=xpfm_path,
    domain=udp_domain_name,
    template=TEMPLATE_NAME,
)
app.build()

vitis.dispose()
print(f"[DONE] Workspace ready at: {WORKSPACE_DIR}")
