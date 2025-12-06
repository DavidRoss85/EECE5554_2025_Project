#!/usr/bin/env python3
"""
Generate AprilTag PDFs for Three Bottle System

Generates 1-inch AprilTags for the three bottle pick and place system.

Default tags:
  - Tag 0: Orange Bottle
  - Tag 1: Apple Bottle
  - Tag 2: Yogurt Bottle
  - Tag 10: Orange Drop Zone
  - Tag 11: Apple Drop Zone
  - Tag 12: Yogurt Drop Zone

Usage:
    python generate_apriltag_pdf.py --id 0 1 2 10 11 12 --size 1.0 --multi
    python generate_apriltag_pdf.py --id 0 --size 1.0
    python generate_apriltag_pdf.py --all --size 1.0
"""

import argparse
import numpy as np
import cv2
from reportlab.lib.pagesizes import letter
from reportlab.lib.units import inch
from reportlab.pdfgen import canvas
from reportlab.lib import colors
import tempfile
import os


def generate_apriltag_image(tag_id, tag_size_pixels=500):
    """
    Generate AprilTag image using cv2.aruco.
    
    Args:
        tag_id: Tag ID (0-586 for 36h11 family)
        tag_size_pixels: Size of output image in pixels
    
    Returns:
        numpy array with tag image
    """
    # AprilTag 36h11 corresponds to DICT_APRILTAG_36h11 in cv2.aruco
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    
    # Generate tag
    tag_image = cv2.aruco.generateImageMarker(aruco_dict, tag_id, tag_size_pixels)
    
    return tag_image


class AprilTagGenerator:
    """Generate AprilTag images and PDFs."""
    
    def __init__(self, family='tag36h11'):
        """
        Initialize AprilTag generator.
        
        Args:
            family: AprilTag family (default: tag36h11)
        """
        self.family = family
    
    def generate_tag_image(self, tag_id, size_pixels=500):
        """
        Generate AprilTag image using OpenCV.
        
        Args:
            tag_id: Tag ID to generate
            size_pixels: Output image size in pixels
        
        Returns:
            numpy array with tag image (binary: 0 or 255)
        """
        return generate_apriltag_image(tag_id, size_pixels)
    
    def save_tag_image(self, tag_id, filename, size_pixels=500):
        """
        Save AprilTag image to file.
        
        Args:
            tag_id: Tag ID
            filename: Output filename (PNG)
            size_pixels: Image size in pixels
        """
        img = self.generate_tag_image(tag_id, size_pixels)
        cv2.imwrite(filename, img)
        print(f"Saved tag {tag_id} to {filename}")
    
    def create_pdf_single(self, tag_id, output_file, tag_size_inches=1.0, include_info=True):
        """
        Create PDF with single AprilTag.
        
        Args:
            tag_id: Tag ID to generate
            output_file: Output PDF filename
            tag_size_inches: Tag size in inches
            include_info: Include tag ID and size information on PDF
        """
        # Create canvas
        c = canvas.Canvas(output_file, pagesize=letter)
        page_width, page_height = letter
        
        # Generate tag image (high resolution for printing)
        tag_size_pixels = 1000
        img = self.generate_tag_image(tag_id, tag_size_pixels)
        
        # Save temporary image
        temp_file = tempfile.NamedTemporaryFile(suffix='.png', delete=False)
        cv2.imwrite(temp_file.name, img)
        
        # Calculate position (centered)
        tag_size = tag_size_inches * inch
        x = (page_width - tag_size) / 2
        y = (page_height - tag_size) / 2 + inch  # Offset up a bit for text below
        
        # Draw tag
        c.drawImage(temp_file.name, x, y, width=tag_size, height=tag_size)
        
        if include_info:
            # Add title
            c.setFont("Helvetica-Bold", 16)
            title_y = y + tag_size + 0.5 * inch
            c.drawCentredString(page_width / 2, title_y, f"AprilTag 36h11 - ID {tag_id}")
            
            # Add size information
            c.setFont("Helvetica", 12)
            info_y = y - 0.5 * inch
            c.drawCentredString(page_width / 2, info_y, f"Tag Size: {tag_size_inches:.2f} inches")
            c.drawCentredString(page_width / 2, info_y - 0.25 * inch, 
                              f"Measure and verify actual size after printing")
            
            # Add border measurements
            c.setFont("Helvetica", 10)
            c.setStrokeColorRGB(0.7, 0.7, 0.7)
            c.setLineWidth(0.5)
            
            # Horizontal measurement lines
            c.line(x - 0.25*inch, y, x, y)
            c.line(x - 0.25*inch, y + tag_size, x, y + tag_size)
            c.line(x - 0.25*inch, y, x - 0.25*inch, y + tag_size)
            
            # Size label
            label_x = x - 0.5 * inch
            label_y = y + tag_size / 2
            c.saveState()
            c.translate(label_x, label_y)
            c.rotate(90)
            c.drawCentredString(0, 0, f'{tag_size_inches}"')
            c.restoreState()
            
            # Add usage instructions at bottom
            c.setFont("Helvetica", 10)
            instructions = [
                "Instructions:",
                "1. Print this page without scaling (actual size)",
                "2. Verify tag size with ruler (should be exactly as specified)",
                "3. Cut out tag carefully along edges",
                "4. Attach to top of bottle with tag visible to camera",
                "5. Keep tag flat and unwrinkled for best detection"
            ]
            
            instruction_y = 1.5 * inch
            for i, instruction in enumerate(instructions):
                c.drawString(inch, instruction_y - i * 0.2 * inch, instruction)
        
        # Draw cut marks at corners
        c.setStrokeColorRGB(0.5, 0.5, 0.5)
        c.setLineWidth(0.5)
        mark_length = 0.2 * inch
        
        corners = [
            (x, y),  # Bottom-left
            (x + tag_size, y),  # Bottom-right
            (x, y + tag_size),  # Top-left
            (x + tag_size, y + tag_size)  # Top-right
        ]
        
        for cx, cy in corners:
            # Horizontal marks
            c.line(cx - mark_length, cy, cx, cy)
            c.line(cx, cy, cx + mark_length, cy)
            # Vertical marks
            c.line(cx, cy - mark_length, cx, cy)
            c.line(cx, cy, cx, cy + mark_length)
        
        # Save PDF
        c.save()
        
        # Clean up temp file
        os.unlink(temp_file.name)
        
        print(f"Created {output_file}")
        print(f"  Tag ID: {tag_id}")
        print(f"  Tag Size: {tag_size_inches} inches")
        print(f"  Family: 36h11")
    
    def create_pdf_multi(self, tag_ids, output_file, tag_size_inches=1.0, tags_per_row=2):
        """
        Create PDF with multiple AprilTags on one page.
        
        Args:
            tag_ids: List of tag IDs
            output_file: Output PDF filename
            tag_size_inches: Size of each tag in inches
            tags_per_row: Number of tags per row
        """
        c = canvas.Canvas(output_file, pagesize=letter)
        page_width, page_height = letter
        
        # Calculate layout
        tag_size_points = tag_size_inches * inch
        margin = 0.5 * inch
        spacing = 0.5 * inch
        
        # Starting position
        start_x = margin
        start_y = page_height - margin - tag_size_points
        
        row = 0
        col = 0
        
        for tag_id in tag_ids:
            # Generate tag
            tag_image = self.generate_tag_image(tag_id, 800)
            temp_file = tempfile.NamedTemporaryFile(suffix='.png', delete=False)
            cv2.imwrite(temp_file.name, tag_image)
            
            # Calculate position
            x = start_x + col * (tag_size_points + spacing)
            y = start_y - row * (tag_size_points + spacing)
            
            # Check if we need a new page
            if y < margin:
                c.showPage()
                row = 0
                y = start_y
            
            # Draw tag
            c.drawImage(temp_file.name, x, y, width=tag_size_points, height=tag_size_points)
            
            # Add label
            c.setFont("Helvetica", 10)
            
            # Get bottle name from tag ID
            tag_names = {
                0: "Orange Bottle",
                1: "Apple Bottle",
                2: "Yogurt Bottle",
                10: "Orange Drop Zone",
                11: "Apple Drop Zone",
                12: "Yogurt Drop Zone"
            }
            
            tag_name = tag_names.get(tag_id, f"Tag {tag_id}")
            c.drawCentredString(x + tag_size_points/2, y - 0.2*inch, f"ID {tag_id}: {tag_name}")
            
            # Clean up
            os.unlink(temp_file.name)
            
            # Update position
            col += 1
            if col >= tags_per_row:
                col = 0
                row += 1
        
        # Add title
        c.setFont("Helvetica-Bold", 14)
        c.drawCentredString(page_width / 2, page_height - 0.3*inch, "AprilTag 36h11 Sheet - Three Bottle System")
        c.setFont("Helvetica", 10)
        c.drawCentredString(page_width / 2, page_height - 0.5*inch, 
                           f"Tag Size: {tag_size_inches} inches each")
        
        c.save()
        print(f"Created {output_file} with {len(tag_ids)} tags")


def main():
    parser = argparse.ArgumentParser(description='Generate AprilTag PDFs for printing')
    parser.add_argument('--id', type=int, nargs='+', default=[0, 1, 2],
                       help='Tag ID(s) to generate (default: 0 1 2)')
    parser.add_argument('--all', action='store_true',
                       help='Generate all tags for three bottle system (0,1,2,10,11,12)')
    parser.add_argument('--size', type=float, default=1.0,
                       help='Tag size in inches (default: 1.0)')
    parser.add_argument('--output', type=str, default=None,
                       help='Output filename (default: apriltag_ID.pdf or apriltags_sheet.pdf)')
    parser.add_argument('--multi', action='store_true',
                       help='Generate all tags on one sheet')
    parser.add_argument('--tags-per-row', type=int, default=2,
                       help='Tags per row for multi-tag sheet (default: 2)')
    
    args = parser.parse_args()
    
    # Determine tag IDs
    if args.all:
        tag_ids = [0, 1, 2, 10, 11, 12]  # All tags for three bottle system
    elif args.id:
        tag_ids = args.id
    else:
        # Default: all tags
        tag_ids = [0, 1, 2, 10, 11, 12]
    
    print("AprilTag PDF Generator")
    print("=" * 60)
    print(f"  Family: 36h11")
    print(f"  Size: {args.size} inches ({args.size * 25.4:.1f} mm)")
    print(f"  Tag IDs: {tag_ids}")
    print()
    
    # Generate tags
    generator = AprilTagGenerator(family='tag36h11')
    
    if args.multi or len(tag_ids) > 1:
        # Generate multi-tag sheet
        output_file = args.output or 'apriltags_sheet.pdf'
        generator.create_pdf_multi(tag_ids, output_file, args.size, args.tags_per_row)
    else:
        # Generate individual PDFs
        for tag_id in tag_ids:
            output_file = args.output or f'apriltag_{tag_id:03d}.pdf'
            generator.create_pdf_single(tag_id, output_file, args.size)
    
    print("=" * 60)
    print("\nNext steps:")
    print("1. Print the PDF(s) at actual size (no scaling)")
    print("2. Measure the printed tag to verify size")
    print("3. Cut out carefully and attach to bottles")
    print(f"4. Update robot_config.yaml with tag size: {args.size * 0.0254:.4f} meters")
    print("\nTag ID mapping:")
    print("  - Tag 0: Orange bottle")
    print("  - Tag 1: Apple bottle")
    print("  - Tag 2: Yogurt bottle")
    print("  - Tag 10: Orange drop zone")
    print("  - Tag 11: Apple drop zone")
    print("  - Tag 12: Yogurt drop zone")


if __name__ == '__main__':
    main()

