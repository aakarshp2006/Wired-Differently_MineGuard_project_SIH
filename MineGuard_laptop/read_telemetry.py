"""
MineGuard - Phase 4
Simple script: laptop se ESP32 ka telemetry data baar-baar mangna.

Ye kya karta hai:
    - Har 1 second mein ESP32 ko poochta hai "/telemetry"
    - Jawab (JSON) ko padhta hai
    - Distance, speed, motor status console mein print karta hai

Kaise chalayein:
    1. Laptop ko "MineGuard_Robot" WiFi se connect karo
    2. Terminal/Command Prompt kholo
    3. Type karo: python read_telemetry.py
    4. Band karne ke liye: Ctrl + C dabao
"""

import requests
import time

# ESP32 ka address -- jab ESP32 apna WiFi banata hai (AP mode),
# uska address hamesha 192.168.4.1 hota hai (default)
ESP32_URL = "http://192.168.4.1/telemetry"

# Kitni der mein ek baar data mangna hai (seconds mein)
POLL_INTERVAL_SECONDS = 1.0


def get_telemetry():
    """
    ESP32 se ek baar telemetry data mangta hai.
    Agar sab thik raha toh data (dictionary) return karta hai.
    Agar koi problem hui (WiFi disconnect, ESP32 band, etc.) toh None deta hai.
    """
    try:
        response = requests.get(ESP32_URL, timeout=2)
        response.raise_for_status()  # agar HTTP error aaya toh yahi rukega
        return response.json()
    except requests.exceptions.RequestException as error:
        print(f"[ERROR] ESP32 se data nahi mila: {error}")
        return None


def main():
    print("=" * 50)
    print("MineGuard - Telemetry Reader")
    print("=" * 50)
    print(f"ESP32 se connect ho raha hai: {ESP32_URL}")
    print("Rukne ke liye Ctrl+C dabao")
    print("=" * 50)

    while True:
        data = get_telemetry()

        if data is not None:
            # Data mil gaya -- ab usme se jo chahiye wo nikal ke print karo
            distance = data.get("distance_m", "N/A")
            distance_status = data.get("distance_status", "N/A")
            speed = data.get("speed_mps", "N/A")
            movement = data.get("movement_state", "N/A")
            safety = data.get("safety_state", "N/A")

            print(
                f"Distance: {distance}m ({distance_status}) | "
                f"Speed: {speed} m/s | "
                f"Movement: {movement} | "
                f"Safety: {safety}"
            )

        time.sleep(POLL_INTERVAL_SECONDS)


if __name__ == "__main__":
    main()
