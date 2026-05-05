# Communication Protocol — OpenMV → STM32

## Physical Layer

| Parameter | Value |
|-----------|-------|
| UART | USART3 |
| Pins | OpenMV TX(P4) → STM32 PB11(RX), GND ↔ GND |
| Baud rate | 115200 |
| Data bits | 8 |
| Stop bits | 1 |
| Parity | None |

## Frame Format (5 bytes)

```
[0xFF] [0xFE] [hasBlob] [tx] [ty]
```

| Byte | Field | Range | Description |
|------|-------|-------|-------------|
| 0 | Header 1 | `0xFF` | Frame sync |
| 1 | Header 2 | `0xFE` | Frame sync |
| 2 | hasBlob | `0x00` or `0x01` | Target detection flag |
| 3 | tx | 0–255 | Normalized X (128 = center) |
| 4 | ty | 0–255 | Normalized Y (128 = center) |

No checksum. The 2-byte header provides resynchronization on data loss.

## Coordinate Mapping

QVGA 320×240 → normalized 0-255:

```
tx = round(cx / 320 × 255)
ty = round(cy / 240 × 255)
```

Image center (cx=160, cy=120) maps to (tx=128, ty=128).

## State Machine (STM32 receiver)

The USART3 RX interrupt implements a header-detection state machine:

1. Scan for `0xFF 0xFE` header sequence
2. After header locked, collect 3 payload bytes
3. Write `[hasBlob, tx, ty]` to `OpenMV_Rxbuf`
4. Set `OpenMV_Usart_Compelet = 1`
5. Control loop (100 Hz) consumes the buffer

Frame boundaries are self-synchronizing: if payload bytes coincidentally form `0xFF 0xFE`, the state machine resyncs on the next valid header (1-frame glitch, probability ≈ 1/65536).
