## Packet format

| Start byte | Packet ID | Command ID | Payload length | Payload data | Checksum | End byte |
| ---------- | --------- | ---------- | -------------- | ------------ | -------- | -------- |
|                                                                                           |

The maximum packet length is 256 bytes.

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
Description: A simple 16 bit addition checksum. Add up all bytes of data from packet ID through to the last byte of payload data (start byte is excluded from checksum).

### End byte
Length: 1 byte  
Description: End byte (`SHD_END_BYTE`) is defined as `0x04`.

## Commanmd format
### Switch command
...
### Switch fade command
...
### Poll command
...
### Version command
...
### Settings command
...
### Warmup command
...
### Calibration1 command
...
### Calibration2 command
...