#!/usr/bin/env python3
"""
Generate a checkerboard pattern as PDF for easy printing

This creates a high-quality PDF that can be printed directly without
worrying about scaling issues in the browser.

Usage:
    python generate_checkerboard_pdf.py [--size 8x6] [--square 25] [--output checkerboard.pdf]
"""

import numpy as np
import cv2
import argparse
from PIL import Image, ImageDraw, ImageFont
import os

def create_checkerboard_image(pattern_size=(9, 7), square_size_mm=25, dpi=300):
    """
    Create checkerboard pattern as image
    
    Args:
        pattern_size: (width, height) in squares (not inner corners!)
        square_size_mm: Size of each square in mm
        dpi: Resolution in dots per inch
    
    Returns:
        PIL Image
    """
    # Calculate dimensions
    # Convert mm to inches to pixels
    square_size_inches = square_size_mm / 25.4
    square_size_pixels = int(square_size_inches * dpi)
    
    width_squares, height_squares = pattern_size
    width_pixels = width_squares * square_size_pixels
    height_pixels = height_squares * square_size_pixels
    
    # Add margin for text
    margin_top = int(1.5 * dpi)  # 1.5 inches top margin
    margin_bottom = int(2 * dpi)  # 2 inches bottom margin
    margin_side = int(0.75 * dpi)  # 0.75 inches side margin
    
    # Create image
    total_width = width_pixels + 2 * margin_side
    total_height = height_pixels + margin_top + margin_bottom
    
    img = Image.new('RGB', (total_width, total_height), 'white')
    draw = ImageDraw.Draw(img)
    
    # Draw title
    try:
        title_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 60)
        text_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 40)
        small_font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 35)
    except:
        # Fallback to default font
        title_font = ImageFont.load_default()
        text_font = ImageFont.load_default()
        small_font = ImageFont.load_default()
    
    # Title
    title = "Camera Calibration Checkerboard"
    title_bbox = draw.textbbox((0, 0), title, font=title_font)
    title_width = title_bbox[2] - title_bbox[0]
    draw.text(((total_width - title_width) // 2, 50), title, fill='black', font=title_font)
    
    # Info text
    inner_corners = (width_squares - 1, height_squares - 1)
    info = f"{inner_corners[0]}x{inner_corners[1]} Inner Corners • {width_squares}x{height_squares} Squares • {square_size_mm}mm Square Size"
    info_bbox = draw.textbbox((0, 0), info, font=text_font)
    info_width = info_bbox[2] - info_bbox[0]
    draw.text(((total_width - info_width) // 2, 150), info, fill='black', font=text_font)
    
    # Warning
    warning = " IMPORTANT: Print at 100% scale (NO scaling) "
    warning_bbox = draw.textbbox((0, 0), warning, font=text_font)
    warning_width = warning_bbox[2] - warning_bbox[0]
    draw.text(((total_width - warning_width) // 2, 230), warning, fill='red', font=text_font)
    
    # Draw checkerboard
    y_offset = margin_top
    x_offset = margin_side
    
    for row in range(height_squares):
        for col in range(width_squares):
            # Alternate black and white
            if (row + col) % 2 == 0:
                color = 'black'
            else:
                color = 'white'
            
            x1 = x_offset + col * square_size_pixels
            y1 = y_offset + row * square_size_pixels
            x2 = x1 + square_size_pixels
            y2 = y1 + square_size_pixels
            
            draw.rectangle([x1, y1, x2, y2], fill=color, outline='black', width=2)
    
    # Instructions at bottom
    y_bottom = y_offset + height_pixels + 80
    
    instructions = [
        "PRINTING INSTRUCTIONS:",
        f"1. Print at 100% scale (actual size) - DO NOT scale to fit page",
        f"2. Verify: Each square should measure exactly {square_size_mm}mm ({square_size_mm/10:.1f}cm) with a ruler",
        f"3. Mount on rigid flat surface (foam board, cardboard, or wood)",
        f"4. Ensure pattern is perfectly flat - any warping affects calibration",
        f"5. Use good lighting for clear black/white contrast during calibration"
    ]
    
    for i, line in enumerate(instructions):
        if i == 0:
            draw.text((margin_side, y_bottom + i * 55), line, fill='black', font=text_font)
        else:
            draw.text((margin_side, y_bottom + i * 55), line, fill='black', font=small_font)
    
    return img

def main():
    parser = argparse.ArgumentParser(description='Generate checkerboard pattern PDF')
    parser.add_argument('--size', type=str, default='6x5',
                       help='Pattern size in INNER CORNERS (default: 6x5 = 7x6 squares, fits <10cm)')
    parser.add_argument('--square', type=float, default=15.0,
                       help='Square size in mm (default: 15mm for small pattern)')
    parser.add_argument('--output', type=str, default='checkerboard.pdf',
                       help='Output PDF file (default: checkerboard.pdf)')
    parser.add_argument('--dpi', type=int, default=300,
                       help='Resolution in DPI (default: 300)')
    args = parser.parse_args()
    
    # Parse size (inner corners)
    size_parts = args.size.lower().split('x')
    if len(size_parts) != 2:
        print(" Invalid size format. Use format like '8x6'")
        return
    
    inner_corners = (int(size_parts[0]), int(size_parts[1]))
    # Pattern size in squares = inner corners + 1
    pattern_size = (inner_corners[0] + 1, inner_corners[1] + 1)
    
    print("=" * 70)
    print(" Generating Checkerboard Pattern")
    print("=" * 70)
    print(f"Inner corners: {inner_corners[0]} x {inner_corners[1]}")
    print(f"Squares: {pattern_size[0]} x {pattern_size[1]}")
    print(f"Square size: {args.square}mm")
    print(f"DPI: {args.dpi}")
    print(f"Output: {args.output}")
    
    # Create checkerboard
    print("\nGenerating image...")
    img = create_checkerboard_image(pattern_size, args.square, args.dpi)
    
    # Save as PDF
    print(f"Saving to {args.output}...")
    img.save(args.output, "PDF", resolution=args.dpi, quality=100)
    
    # Also save as PNG for preview
    png_output = args.output.replace('.pdf', '.png')
    print(f"Saving preview to {png_output}...")
    img.save(png_output, "PNG", dpi=(args.dpi, args.dpi))
    
    print("\n" + "=" * 70)
    print("✓ Checkerboard pattern generated!")
    print("=" * 70)
    print(f"\nPDF file: {args.output}")
    print(f"Preview: {png_output}")
    print(f"\n Print the PDF at 100% scale (actual size)")
    print(f"   Each square should measure {args.square}mm when printed")
    print("=" * 70)

if __name__ == '__main__':
    main()

