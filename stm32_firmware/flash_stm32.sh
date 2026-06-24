#!/bin/bash
# STM32 V3.0 firmware flash script via stm32flash
# Requires: stm32flash (apt-get install stm32flash)
# Usage: bash flash_stm32.sh [firmware.bin]
#
# The STM32 must be put into bootloader mode:
#   1. Hold BOOT0 button on V3.0 board
#   2. Press and release RESET button
#   3. Release BOOT0 button
#   4. Script will detect and flash within 1 second

FIRMWARE="${1:-firmware.bin}"
PORT="/dev/stm32_board"

if [ ! -f "$FIRMWARE" ]; then
    echo "Firmware not found: $FIRMWARE"
    exit 1
fi

echo "=== STM32 V3.0 Flasher ==="
echo "Firmware: $FIRMWARE ($(stat -c%s "$FIRMWARE") bytes)"
echo "Port: $PORT"
echo ""
echo "Put STM32 into bootloader mode NOW (BOOT0+RESET)..."

# Try write with 0.3s retry interval (bootloader window is ~1s after reset)
for i in $(seq 1 30); do
    OUT=$(stm32flash -b 115200 -w "$FIRMWARE" -v -g 0x0 "$PORT" 2>&1)
    if echo "$OUT" | grep -qE "Wrote and verified|Done"; then
        echo "$OUT"
        echo ""
        echo "=== FLASH SUCCESS ==="
        echo "New firmware is running. Restart ROS2 service:"
        echo "  systemctl restart tracked-vehicle-display"
        exit 0
    fi
    sleep 0.3
done

echo "FAILED: Bootloader not detected. Ensure BOOT0 is held during RESET."
exit 1
