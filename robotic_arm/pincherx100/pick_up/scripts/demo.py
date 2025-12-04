#!/usr/bin/env python3
"""
Demo Script for Three Bottle Pick and Place System

This script demonstrates the basic functionality of the system
and can be used as a template for custom programs.

Usage:
    python demo.py
"""

import sys
import os
import time

# Add parent directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from scripts.pick_place import ThreeBottlePickPlace


def demo_detection():
    """Demo: Detect all bottles."""
    print("\n" + "="*60)
    print("DEMO 1: DETECTION")
    print("="*60)
    
    system = ThreeBottlePickPlace()
    
    try:
        print("\n1. Detecting all bottles...")
        results = system.detect_all_bottles(show_visualization=True)
        
        print(f"\nDetected {len(results)} bottles:")
        for bottle_name, data in results.items():
            pos = data['position_base']
            print(f"  {bottle_name.capitalize()}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m")
        
        print("\n✓ Detection demo complete")
        
    finally:
        system.shutdown()


def demo_pick_and_place():
    """Demo: Pick and place a single bottle."""
    print("\n" + "="*60)
    print("DEMO 2: PICK AND PLACE")
    print("="*60)
    
    system = ThreeBottlePickPlace()
    
    try:
        # Initialize arm
        print("\n1. Initializing arm...")
        if not system.arm_controller.initialize():
            print("Failed to initialize arm")
            return
        print("✓ Arm initialized")
        
        # Move to home
        print("\n2. Moving to home position...")
        system.move_home()
        time.sleep(2)
        
        # Pick orange bottle
        print("\n3. Picking orange bottle (Tag 0)...")
        success = system.pick_bottle('orange', show_visualization=True)
        if not success:
            print("Failed to pick orange bottle")
            return
        print("✓ Orange bottle picked")
        time.sleep(1)
        
        # Place at orange drop zone
        print("\n4. Placing at orange drop zone (Tag 10)...")
        success = system.place_at_zone('orange', show_visualization=True)
        if not success:
            print("Failed to place bottle")
            return
        print("✓ Bottle placed")
        
        # Return home
        print("\n5. Returning to home position...")
        system.move_home()
        
        print("\n✓ Pick and place demo complete")
        
    finally:
        system.shutdown()


def demo_move_sequence():
    """Demo: Move multiple bottles in sequence."""
    print("\n" + "="*60)
    print("DEMO 3: MOVE SEQUENCE")
    print("="*60)
    
    system = ThreeBottlePickPlace()
    
    try:
        # Initialize arm
        print("\n1. Initializing arm...")
        if not system.arm_controller.initialize():
            print("Failed to initialize arm")
            return
        print("✓ Arm initialized")
        
        # Move to home
        print("\n2. Moving to home position...")
        system.move_home()
        time.sleep(2)
        
        # Sequence 1: Orange to Apple zone
        print("\n3. Moving orange bottle to apple zone...")
        success = system.move_bottle('orange', 'apple', show_visualization=True)
        if not success:
            print("Failed to move orange bottle")
            return
        print("✓ Orange bottle moved to apple zone")
        time.sleep(2)
        
        # Sequence 2: Apple to Yogurt zone
        print("\n4. Moving apple bottle to yogurt zone...")
        success = system.move_bottle('apple', 'yogurt', show_visualization=True)
        if not success:
            print("Failed to move apple bottle")
            return
        print("✓ Apple bottle moved to yogurt zone")
        time.sleep(2)
        
        # Return home
        print("\n5. Returning to home position...")
        system.move_home()
        
        print("\n✓ Move sequence demo complete")
        
    finally:
        system.shutdown()


def demo_detection_only():
    """Demo: Detection without arm movement (safe test)."""
    print("\n" + "="*60)
    print("DEMO: DETECTION ONLY (SAFE TEST)")
    print("="*60)
    print("\nThis demo only detects bottles without moving the arm.")
    print("Safe for initial testing.")
    
    system = ThreeBottlePickPlace()
    
    try:
        print("\nDetecting bottles...")
        
        # Test detection for each bottle
        for bottle_name in ['orange', 'apple', 'yogurt']:
            print(f"\n--- {bottle_name.upper()} BOTTLE ---")
            detection = system.detect_bottle(bottle_name, show_visualization=False)
            
            if detection:
                pos = detection['position_base']
                quality = detection['quality']
                print(f"✓ Detected at: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m")
                print(f"  Quality: H={quality['hamming']}, M={quality['decision_margin']:.1f}")
            else:
                print(f"✗ Not detected")
        
        # Show all detections with visualization
        print("\n--- ALL BOTTLES ---")
        results = system.detect_all_bottles(show_visualization=True)
        print(f"\nTotal detected: {len(results)}/3 bottles")
        
        print("\n✓ Detection demo complete")
        
    finally:
        system.shutdown()


def main():
    """Main demo menu."""
    print("\n" + "="*60)
    print("THREE BOTTLE PICK AND PLACE SYSTEM - DEMO")
    print("="*60)
    print("\nAvailable demos:")
    print("  1. Detection only (safe, no arm movement)")
    print("  2. Detection with visualization")
    print("  3. Pick and place single bottle")
    print("  4. Move sequence (multiple bottles)")
    print("  0. Exit")
    print()
    
    choice = input("Select demo (0-4): ").strip()
    
    if choice == '1':
        demo_detection_only()
    elif choice == '2':
        demo_detection()
    elif choice == '3':
        demo_pick_and_place()
    elif choice == '4':
        demo_move_sequence()
    elif choice == '0':
        print("Exiting...")
        return
    else:
        print("Invalid choice")
        return
    
    print("\n" + "="*60)
    print("Demo finished!")
    print("="*60)


if __name__ == '__main__':
    main()

