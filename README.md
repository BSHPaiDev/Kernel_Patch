# PAI 1.0
#### This repository contains patch for linux kernel.

Kernel Source can be obtained from
```
repo init -u https://source.codeaurora.org/quic/la/platform/manifest.git -b release -m 
LA.UM.6.6.r1-04400-89xx.0.xml --repo-url=https://source.codeaurora.org/tools/repo.git 
--repo-branch=caf-stable
```
## Installation
Navigate to kernel/msm and rin following commands in the terminal
```
patch -s -p0 < patchfile.patch
```
## Disclosures of Open Source Content - Firmware
### 1. PWM Fan Control
   **Original source code:** [eInfoChips](https://www.einfochips.com/)

   **License:** GNU General Public License v2.0

### 2. PWM output using GPIO
   **Original source code:** [mrdunk/home_automation_2](https://github.com/mrdunk/home_automation_2/blob/master/openwrt/sw_pwm_kernel_module/gpio-pwm.c)
   
   **License:** GNU General Public License v2.0
### 3. ISSI IS31FL32xx LED Driver
   **Original source code:** [torvalds/linux](https://github.com/torvalds/linux/blob/master/drivers/leds/leds-is31fl32xx.c)
   
   **License:** GNU General Public License v2.0
### 4. NTC Temperature Monitor ADC Driver
   **Original source code:** [eInfoChips](https://www.einfochips.com/)
   
   **License:** GNU General Public License v2.0
### 5. VOUT Temperature Monitor ADC Driver
   **Original source code:** [eInfoChips](https://www.einfochips.com/)
   
   **License:** GNU General Public License v2.0
### 6. ST FlightSense Time-of-Flight Sensor Driver
   **Original source code:** [STFlightSenseLinuxDrivers/vl6180-driver
](https://github.com/STFlightSenseLinuxDrivers/vl6180-driver)
   
   **License:** GNU General Public License v2.0
   **Installation:** copy folder `vl6180` to `kernel/msm-3.18/drivers/input/misc`
