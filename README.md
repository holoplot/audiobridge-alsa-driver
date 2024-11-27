# Holoplot AudioBridge

This repository contains the source code for the Holoplot AudioBridge ALSA driver.

The AudioBridge is a PCIe card that features Dante and Ravenna network interfaces
and can be used to convert between AoIP protocols and send and receive audio data
to the PCI host.

# Features

## ALSA

The ALSA PCM interface of this driver has the following features:

* 1-256 channels
* 32-bit PCM format (`S32_LE`)
* 48 kHz sample rate

## Sysfs

The driver registers an hwdep device that can be used to read some information
about the card.

| Attribute                     | Description                                  |
| ----------------------------- | -------------------------------------------- |
| `device_id`                   | The device ID of the card                    |
| `fpga_design_type`            | The type of the FPGA design                  |
| `fpga_design_version`         | The version of the FPGA design               |
| `primary_ip_address`          | The primary IP address of the card           |
| `secondary_ip_address`        | The secondary IP address of the card         |
| `primary_mac_address`         | The primary MAC address of the card          |
| `secondary_mac_address`       | The secondary MAC address of the card        |

E.g, to read the device ID of the card, you can use the following command:

```bash
# cat /sys/class/sound/hwC0D0/device_id
ABC-1234
```

# Compilation

To compile the driver, you need to have the kernel headers installed.
You can then build the sources with the following command:

```bash
make -C /usr/lib/modules/$(uname -r)/source M=$(pwd)/modules
```

# License

This driver is licensed under the GNU General Public License version 2.
