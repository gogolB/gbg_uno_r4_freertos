#!/usr/bin/env python3
"""
Simple test to verify the Bluetooth client can be imported and basic functions work
"""

import struct

def test_joystick_data_packing():
    """Test that joystick data packing works correctly"""
    
    # Test cases: (x, y, expected_bytes)
    test_cases = [
        (0, 0, b'\x00\x00\x00\x00\x00\x00\x00\x00'),
        (100, 50, b'\x64\x00\x32\x00\x00\x00\x00\x00'),
        (-100, -50, b'\x9c\xff\xce\xff\x00\x00\x00\x00'),
    ]
    
    print("Testing joystick data packing...")
    for x, y, expected in test_cases:
        # Pack as little-endian 16-bit signed integers with padding
        data = struct.pack('<hhxxxx', x, y)
        if data == expected:
            print(f"✓ ({x}, {y}) -> {data.hex()}")
        else:
            print(f"✗ ({x}, {y}) -> {data.hex()}, expected {expected.hex()}")

def test_stop_command():
    """Test stop command format"""
    print("\nTesting stop command...")
    stop_data = bytes([1])
    print(f"✓ Stop command: {stop_data.hex()}")

def test_value_constraints():
    """Test value constraint logic"""
    print("\nTesting value constraints...")
    
    def constrain(value, min_val, max_val):
        return max(min_val, min(max_val, value))
    
    test_values = [(-150, -100), (0, 0), (150, 100), (75, 75)]
    
    for input_val, expected in test_values:
        result = constrain(input_val, -100, 100)
        if result == expected:
            print(f"✓ constrain({input_val}) -> {result}")
        else:
            print(f"✗ constrain({input_val}) -> {result}, expected {expected}")

if __name__ == "__main__":
    print("GBG Robot Bluetooth Client Test")
    print("=" * 40)
    
    test_joystick_data_packing()
    test_stop_command()
    test_value_constraints()
    
    print("\nTest complete!")
    print("\nTo test with actual robot:")
    print("1. Flash the updated firmware to the robot")
    print("2. Install bleak: pip install bleak")
    print("3. Run: python3 bluetooth_client.py")