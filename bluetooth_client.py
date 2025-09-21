#!/usr/bin/env python3
"""
GBG Robot Bluetooth Client Example

This script demonstrates how to connect to the GBG Robot and send joystick
commands and stop signals via Bluetooth Low Energy.

Requirements:
- Python 3.7+
- bleak library (pip install bleak)

Usage:
    python3 bluetooth_client.py
"""

import asyncio
import struct
from bleak import BleakClient, BleakScanner

# GBG Robot BLE identifiers
ROBOT_NAME = "GBG Robot"
SERVICE_UUID = "12345678-1234-1234-1234-123456789abc"
JOYSTICK_CHAR_UUID = "12345678-1234-1234-1234-123456789abd"
STOP_CHAR_UUID = "12345678-1234-1234-1234-123456789abe"

class GBGRobotClient:
    def __init__(self):
        self.client = None
        self.connected = False
        
    async def scan_for_robot(self, timeout=10):
        """Scan for GBG Robot devices"""
        print(f"Scanning for {ROBOT_NAME}...")
        devices = await BleakScanner.discover(timeout=timeout)
        
        for device in devices:
            if device.name == ROBOT_NAME:
                print(f"Found {ROBOT_NAME} at {device.address}")
                return device.address
        
        print(f"No {ROBOT_NAME} found in scan")
        return None
    
    async def connect(self, address=None):
        """Connect to the robot"""
        if not address:
            address = await self.scan_for_robot()
            if not address:
                return False
        
        try:
            self.client = BleakClient(address)
            await self.client.connect()
            self.connected = True
            print(f"Connected to robot at {address}")
            
            # Verify services are available
            services = await self.client.get_services()
            service = services.get_service(SERVICE_UUID)
            if service:
                print("Robot control service found!")
                return True
            else:
                print("Robot control service not found!")
                await self.disconnect()
                return False
                
        except Exception as e:
            print(f"Connection failed: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from the robot"""
        if self.client and self.connected:
            await self.client.disconnect()
            self.connected = False
            print("Disconnected from robot")
    
    async def send_joystick_data(self, x, y):
        """
        Send joystick data to the robot
        
        Args:
            x (int): X-axis value (-100 to +100)
            y (int): Y-axis value (-100 to +100)
        """
        if not self.connected:
            print("Not connected to robot")
            return False
        
        # Constrain values to valid range
        x = max(-100, min(100, x))
        y = max(-100, min(100, y))
        
        try:
            # Pack as little-endian 16-bit signed integers with padding
            data = struct.pack('<hhxxxx', x, y)
            await self.client.write_gatt_char(JOYSTICK_CHAR_UUID, data)
            print(f"Sent joystick data: X={x}, Y={y}")
            return True
        except Exception as e:
            print(f"Failed to send joystick data: {e}")
            return False
    
    async def send_stop_command(self):
        """Send emergency stop command"""
        if not self.connected:
            print("Not connected to robot")
            return False
        
        try:
            data = bytes([1])  # Non-zero value triggers stop
            await self.client.write_gatt_char(STOP_CHAR_UUID, data)
            print("Sent STOP command")
            return True
        except Exception as e:
            print(f"Failed to send stop command: {e}")
            return False

async def demo_sequence():
    """Demonstrate robot control with a simple sequence"""
    robot = GBGRobotClient()
    
    # Connect to robot
    if not await robot.connect():
        print("Failed to connect to robot")
        return
    
    try:
        print("\n--- Starting demo sequence ---")
        
        # Forward
        print("Moving forward...")
        await robot.send_joystick_data(0, 50)
        await asyncio.sleep(2)
        
        # Turn right
        print("Turning right...")
        await robot.send_joystick_data(50, 50)
        await asyncio.sleep(2)
        
        # Turn left
        print("Turning left...")
        await robot.send_joystick_data(-50, 50)
        await asyncio.sleep(2)
        
        # Backward
        print("Moving backward...")
        await robot.send_joystick_data(0, -50)
        await asyncio.sleep(2)
        
        # Stop
        print("Stopping...")
        await robot.send_stop_command()
        await asyncio.sleep(1)
        
        print("Demo sequence complete!")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted")
        await robot.send_stop_command()
    
    finally:
        await robot.disconnect()

async def interactive_mode():
    """Interactive mode for manual control"""
    robot = GBGRobotClient()
    
    # Connect to robot
    if not await robot.connect():
        print("Failed to connect to robot")
        return
    
    print("\n--- Interactive Mode ---")
    print("Commands:")
    print("  w/a/s/d - Move forward/left/backward/right")
    print("  q/e - Turn left/right")
    print("  x - Stop")
    print("  quit - Exit")
    print()
    
    try:
        while True:
            command = input("Enter command: ").strip().lower()
            
            if command == 'quit':
                break
            elif command == 'w':
                await robot.send_joystick_data(0, 50)
            elif command == 's':
                await robot.send_joystick_data(0, -50)
            elif command == 'a':
                await robot.send_joystick_data(-50, 0)
            elif command == 'd':
                await robot.send_joystick_data(50, 0)
            elif command == 'q':
                await robot.send_joystick_data(-50, 50)
            elif command == 'e':
                await robot.send_joystick_data(50, 50)
            elif command == 'x':
                await robot.send_stop_command()
            else:
                print("Unknown command")
    
    except KeyboardInterrupt:
        print("\nExiting...")
        await robot.send_stop_command()
    
    finally:
        await robot.disconnect()

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == "demo":
        asyncio.run(demo_sequence())
    else:
        asyncio.run(interactive_mode())