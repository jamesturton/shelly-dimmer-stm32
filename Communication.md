## Packet format

| Start byte | Packet ID | Command ID | Payload length | Payload data | Checksum | End byte |
| ---------- | --------- | ---------- | -------------- | ------------ | -------- | -------- |
|                                                                                           |

The maximum packet length is 256 bytes.

Multi-byte words are sent in little-endian order.

### Start byte
Length: 1 byte  
Description: Start byte (`SHD_START_BYTE`) is defined as `0x01`.

### Packet ID
Length: 1 byte  
Description: Packet ID counts in a loop from `0` to `255` and increments for packet sent. The reply sent by the STM32 must match this packet ID which the ESP8266 sent.

### Command ID
Length: 1 byte  
Description: Command ID used to define what the packet should do. Along with the packet ID, this should be matched in the reply. The following commands are defined:

| Command name            | ID     |
| ----------------------- | ------ |
| `SHD_SWITCH_CMD`        | `0x01` |
| `SHD_SWITCH_FADE_CMD`   | `0x02` |
| `SHD_POLL_CMD`          | `0x10` |
| `SHD_VERSION_CMD`       | `0x11` |
| `SHD_SETTINGS_CMD`      | `0x20` |
| `SHD_WARMUP_CMD`        | `0x21` |
| `SHD_CALIBRATION1_CMD`  | `0x30` |
| `SHD_CALIBRATION2_CMD`  | `0x31` |

Descriptions of commands can be found below.

### Payload length
Length: 1 byte  
Description: Describes the length of the payload data. This is not the total packet length.

### Payload data
Length: 249 maximum, does not need to be longer than command requires.  
Description: This contains the data defined by the command specification. Read below.

### Checksum
Length: 2 bytes  
Description: A simple 16-bit addition checksum. Add up all bytes of data from packet ID through to the last byte of payload data (start byte is excluded from checksum).

### End byte
Length: 1 byte  
Description: End byte (`SHD_END_BYTE`) is defined as `0x04`.

## Commanmd format
### Switch command - `0x01`
#### Query (from ESP8266)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0-1]`   | Requested brightness      |

Payload length: 2 bytes  
Requested brightness (16-bit): Valid values between 0 (off) and 1000 (fully on).

#### Response (from STM32)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0]`     | Acknowledge               |

Payload length: 1 byte  
Acknowledge: Command accepted. Defined as `0x01`.

### Switch fade command - `0x02`
#### Query (from ESP8266)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0-1]`   | Requested brightness      |
| `[2-3]`   | Brightness delta          |
| `[4-5]`   | Unknown!                  |

Payload length: 6 bytes  
Requested brightness (16-bit): Valid values between 0 (off) and 1000 (fully on).  
Brightness delta (16-bit): Not used in this firmware. From stock Shelly firmware, seemed to be definded as 0.8 * Δ brightness.  
Unknown (16-bit): Included for compatibility with stock firmware, however the function of this is unknown and can safely be sent as `0x00`.

#### Response (from STM32)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0]`     | Acknowledge               |

Payload length: 1 byte  
Acknowledge: Command accepted. Defined as `0x01`.

### Poll command - `0x10`
| Byte      | Payload value             |
| --------- | ------------------------- |
|           |                           |

Payload length: 0 bytes  
Description: No payload sent from ESP8266.

#### Response (from STM32)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0-1]`   | Hardware version          |
| `[2-3]`   | Actual brightness         |
| `[4-7]`   | Active power              |
| `[8-11]`  | Voltage                   |
| `[12-15]` | Current                   |
| `[16]`    | Leading edge              |

Payload length: 17 bytes  
Hardware version (16-bit): This differs from the stock firmware. Shelly dimmer 1 is defined as `0x00`. Shelly dimmer 2 is defined as `0x01`.  
Actual brightness (16-bit): What brightness the dimmer is currently set to. Valid values between 0 (off) and 1000 (fully on).  
Active power (32-bit): Used on the Shelly dimmer 1 and Shelly dimmer 2 when neutral is connected. Value is taken from raw PWM frequency from HLW8012 power measurment chip, thus acutal power in watts can be calculated as 880373 / power.  
Voltage (32-bit): Used on the Shelly dimmer 1 if patched (see https://github.com/arendst/Tasmota/issues/6914#issuecomment-691668208) and Shelly dimmer 2 when neutral is connected. Value is taken from raw PWM frequency from HLW8012 power measurment chip, thus voltage in volts can be calculated as 347800 / voltage.  
Current (32-bit): Used on the Shelly dimmer 1 and Shelly dimmer 2. Value is taken from raw PWM frequency from HLW8012 power measurment chip, thus current in amps can be calculated as 1448 / current.  
Leading edge (8-bit): This differs from the stock firmware. Reports the dimming mode currently being used. Trailing edge is defined as `0x00` (false) and leading edge is defined as `0x01` (true).

### Version command - `0x11`
#### Query (from ESP8266)
| Byte      | Payload value             |
| --------- | ------------------------- |
|           |                           |

Payload length: 0 bytes  
Description: No payload sent from ESP8266.

#### Response (from STM32)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0]`     | Minor version number      |
| `[1]`     | Major version number      |

Payload length: 2 bytes  
Description: Version number interpreted as `v(major).(minor)`, for example `v50.1`.

### Settings command - `0x20`
#### Query (from ESP8266)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0-1]`   | Requested brightness      |
| `[2-3]`   | Leading edge              |
| `[4-5]`   | Fade rate                 |
| `[6-7]`   | Warmup brightness         |
| `[8-9]`   | Warmup time               |

Payload length: 10 bytes  
Requested brightness (16-bit): Valid values between 0 (off) and 1000 (fully on).  
Leading edge (16-bit): Which dimming mode should be used. Trailing edge is defined as `0x02` and leading edge is defined as `0x01`.  
Fade rate (16-bit): Not used in this firmware, however kept for compatibiliity with stock firmware. Brightness fading controlled by ESP8266 is preferred. Can safely be sent as `0x00`.  
Warmup brightness (16-bit): Not used in this firmware. From stock Shelly firmware, used to 'kick start' the bulb when turning on at low brightnesses. Warmup brightness valid values between 0 (off) and 1000 (fully on).  
Warmup time (16-bit): Not used in this firmware. From stock Shelly firmware, used to 'kick start' the bulb when turning on at low brightnesses. Warmup time given in milliseconds.

#### Response (from STM32)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0]`     | Acknowledge               |

Payload length: 1 byte  
Acknowledge: Command accepted. Defined as `0x01`.

### Warmup command - `0x21`
#### Query (from ESP8266)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0-1]`   | Warmup brightness         |
| `[2-3]`   | Warmup time               |

Payload length: 2 bytes   
Warmup brightness (16-bit): Not used in this firmware. From stock Shelly firmware, used to 'kick start' the bulb when turning on at low brightnesses. Warmup brightness valid values between 0 (off) and 1000 (fully on).  
Warmup time (16-bit): Not used in this firmware. From stock Shelly firmware, used to 'kick start' the bulb when turning on at low brightnesses. Warmup time given in milliseconds.

#### Response (from STM32)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0]`     | Acknowledge               |

Payload length: 1 byte  
Acknowledge: Command accepted. Defined as `0x01`.

### Calibration1 command - `0x30`
#### Query (from ESP8266)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0-199]` | Calibration data          |

Payload length: 200 bytes   
Calibration data: Not used in this firmware. Unknown calibration data format.

#### Response (from STM32)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0-199]` | Calibration data          |

Payload length: 200 bytes   
Calibration data: Not used in this firmware. Unknown calibration data format.

### Calibration2 command - `0x31`
#### Query (from ESP8266)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0-199]` | Calibration data          |

Payload length: 200 bytes   
Calibration data: Not used in this firmware. Unknown calibration data format.

#### Response (from STM32)
| Byte      | Payload value             |
| --------- | ------------------------- |
| `[0-199]` | Calibration data          |

Payload length: 200 bytes   
Calibration data: Not used in this firmware. Unknown calibration data format.
