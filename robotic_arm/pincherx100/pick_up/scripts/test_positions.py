#!/usr/bin/env python3
"""
Test script for arm positions

Tests the different arm positions: home and retract
"""

import sys
import os
import time

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from scripts.pick_place import ThreeBottlePickPlace


def test_positions():
    """Test different arm positions."""
    print("\n" + "="*60)
    print("ARM POSITION TEST")
    print("="*60)
    
    system = ThreeBottlePickPlace()
    
    try:
        # Initialize arm
        print("\n1. Initializing arm controller...")
        if not system.arm_controller.initialize():
            print("✗ Failed to initialize arm")
            return False
        print("✓ Arm initialized")
        time.sleep(1)
        
        # Test home position
        print("\n2. Testing HOME position (all joints at 180°)...")
        print("   Expected: Center position, all joints at middle")
        success = system.move_to_position('home')
        if not success:
            print("✗ Failed to move to home")
            return False
        print("✓ Home position reached")
        time.sleep(3)
        
        # Test retract position
        print("\n3. Testing RETRACT position...")
        print("   Expected: Shoulder=0°, Elbow=180°, Wrist=0°")
        success = system.move_to_position('retract')
        if not success:
            print("✗ Failed to move to retract")
            return False
        print("✓ Retract position reached")
        time.sleep(3)
        
        # Return to home
        print("\n4. Returning to HOME position...")
        success = system.move_to_position('home')
        if not success:
            print("✗ Failed to return to home")
            return False
        print("✓ Back to home position")
        
        print("\n" + "="*60)
        print("✓ ALL POSITION TESTS PASSED")
        print("="*60)
        return True
        
    except Exception as e:
        print(f"\n✗ Error during test: {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        system.shutdown()


def main():
    print("\n" + "="*60)
    print("POSITION TEST UTILITY")
    print("="*60)
    print("\nThis script will test the following positions:")
    print("  1. HOME - Center position (all joints at 180°)")
    print("  2. RETRACT - Fully retracted (shoulder=0°, elbow=180°, wrist=0°)")
    print("\nThe arm will move through each position with 3 second delays.")
    print()
    
    response = input("Continue with test? (y/N): ").strip().lower()
    if response != 'y':
        print("Test cancelled.")
        return
    
    success = test_positions()
    
    if success:
        print("\n✓ Position test completed successfully!")
    else:
        print("\n✗ Position test failed!")
        sys.exit(1)


if __name__ == '__main__':
    main()

