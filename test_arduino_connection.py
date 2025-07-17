#!/usr/bin/env python3
"""
Simple Arduino Connection Test Script
Run this to verify Arduino communication before testing ROS2 node
"""

import serial
import time
import sys

def test_arduino_connection():
    """Test connection to Arduino and basic communication"""
    
    # Configuration
    port = '/dev/tty_arduino'
    baud = 9600
    timeout = 2.0
    
    print(f"Testing Arduino connection at {port}...")
    print("=" * 50)
    
    try:
        # Open serial connection
        ser = serial.Serial(port, baud, timeout=timeout)
        print(f"✅ Serial port opened successfully")
        
        # Wait for Arduino to initialize
        print("⏳ Waiting for Arduino to initialize...")
        time.sleep(2.0)
        
        # Clear any pending data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        # Send STATUS command
        print("📤 Sending STATUS command...")
        ser.write(b'STATUS\n')
        ser.flush()
        
        # Read response
        print("📥 Reading Arduino response...")
        response_lines = []
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line:
                    response_lines.append(line)
                    print(f"   Arduino: {line}")
                    
                    # Check for completion
                    if any(keyword in line for keyword in ['STATUS', '====']):
                        continue
                    elif 'Running:' in line or 'Current position:' in line:
                        break
            except:
                break
        
        if response_lines:
            print("✅ Arduino responded successfully!")
            print("\n🎯 Test Commands (type one and press Enter):")
            print("   STATUS - Get current status")
            print("   P - Print configuration")
            print("   R - Run stepper motor")
            print("   S - Stop stepper motor")
            print("   C:800:4000:2000:1 - Configure stepper")
            print("   quit - Exit test")
            print("\n" + "=" * 50)
            
            # Interactive testing
            while True:
                try:
                    cmd = input("Enter command: ").strip()
                    if cmd.lower() == 'quit':
                        break
                    
                    # Send command
                    ser.write((cmd + '\n').encode())
                    ser.flush()
                    
                    # Read response
                    time.sleep(0.5)
                    while ser.in_waiting > 0:
                        try:
                            line = ser.readline().decode('utf-8').strip()
                            if line:
                                print(f"   Arduino: {line}")
                        except:
                            break
                            
                except KeyboardInterrupt:
                    break
            
        else:
            print("❌ No response from Arduino")
            print("   Check:")
            print("   1. Arduino is powered and connected")
            print("   2. Correct sketch is uploaded")
            print("   3. Serial port permissions")
            
        ser.close()
        
    except serial.SerialException as e:
        print(f"❌ Serial connection failed: {e}")
        print("   Check:")
        print("   1. Arduino is connected")
        print("   2. Device permissions: sudo usermod -a -G dialout $USER")
        print("   3. udev rules are applied")
        
    except Exception as e:
        print(f"❌ Unexpected error: {e}")

def check_device_exists():
    """Check if Arduino device exists"""
    import os
    
    if os.path.exists('/dev/tty_arduino'):
        print("✅ /dev/tty_arduino exists")
        
        # Check what it points to
        if os.path.islink('/dev/tty_arduino'):
            target = os.readlink('/dev/tty_arduino')
            print(f"   → Points to: {target}")
        
        # Check permissions
        import stat
        st = os.stat('/dev/tty_arduino')
        mode = stat.filemode(st.st_mode)
        print(f"   → Permissions: {mode}")
        
        return True
    else:
        print("❌ /dev/tty_arduino does not exist")
        print("   Check:")
        print("   1. Arduino is connected")
        print("   2. udev rules are applied")
        print("   3. Run: sudo udevadm control --reload-rules && sudo udevadm trigger")
        return False

if __name__ == "__main__":
    print("🤖 Arduino Connection Test")
    print("=" * 50)
    
    # Check device first
    if check_device_exists():
        print()
        test_arduino_connection()
    
    print("\n✅ Test completed!") 