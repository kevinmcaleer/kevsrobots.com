#!/usr/bin/env python3
"""
Image Optimization Script for KevsRobots Website
Optimizes images IN PLACE for web delivery while maintaining quality
Tracks optimized files to avoid re-processing
"""

import os
import sys
import json
import hashlib
from pathlib import Path
from PIL import Image
import argparse
from datetime import datetime

class ImageOptimizer:
    def __init__(self, image_dir, max_width=1920, max_height=1920,
                 jpeg_quality=85, webp_quality=85, convert_to_webp=False,
                 force=False):
        self.image_dir = Path(image_dir)
        self.max_width = max_width
        self.max_height = max_height
        self.jpeg_quality = jpeg_quality
        self.webp_quality = webp_quality
        self.convert_to_webp = convert_to_webp
        self.force = force

        # Tracking file to store optimization history
        self.tracking_file = self.image_dir / '.optimization_cache.json'
        self.tracking_data = self._load_tracking()

        self.stats = {
            'processed': 0,
            'skipped': 0,
            'already_optimized': 0,
            'original_size': 0,
            'optimized_size': 0,
            'errors': []
        }

    def _load_tracking(self):
        """Load optimization tracking data"""
        if self.tracking_file.exists():
            try:
                with open(self.tracking_file, 'r') as f:
                    return json.load(f)
            except:
                return {}
        return {}

    def _save_tracking(self):
        """Save optimization tracking data"""
        with open(self.tracking_file, 'w') as f:
            json.dump(self.tracking_data, f, indent=2)

    def _get_file_hash(self, file_path):
        """Get MD5 hash of file content"""
        hasher = hashlib.md5()
        with open(file_path, 'rb') as f:
            hasher.update(f.read())
        return hasher.hexdigest()

    def _is_already_optimized(self, image_path):
        """Check if image has already been optimized"""
        if self.force:
            return False

        rel_path = str(image_path.relative_to(self.image_dir))

        if rel_path not in self.tracking_data:
            return False

        # Check if file has changed since optimization
        current_hash = self._get_file_hash(image_path)
        tracked_hash = self.tracking_data[rel_path].get('optimized_hash')

        return current_hash == tracked_hash

    def _mark_optimized(self, image_path, original_size, optimized_size):
        """Mark image as optimized in tracking data"""
        rel_path = str(image_path.relative_to(self.image_dir))
        optimized_hash = self._get_file_hash(image_path)

        self.tracking_data[rel_path] = {
            'optimized_hash': optimized_hash,
            'optimized_date': datetime.now().isoformat(),
            'original_size': original_size,
            'optimized_size': optimized_size,
            'settings': {
                'max_width': self.max_width,
                'max_height': self.max_height,
                'quality': self.webp_quality if self.convert_to_webp else self.jpeg_quality,
                'format': 'webp' if self.convert_to_webp else 'original'
            }
        }

    def optimize_image(self, image_path):
        """Optimize a single image file IN PLACE"""
        try:
            # Check if already optimized
            if self._is_already_optimized(image_path):
                self.stats['already_optimized'] += 1
                print(f"⊘ {image_path.name} (already optimized)")
                return

            original_size = image_path.stat().st_size

            # Create temporary file for processing
            temp_path = image_path.with_suffix(image_path.suffix + '.tmp')

            # Open and process image
            with Image.open(image_path) as img:
                # Store original format
                original_format = img.format

                # Convert RGBA to RGB for JPEG/WebP
                if img.mode in ('RGBA', 'LA', 'P'):
                    background = Image.new('RGB', img.size, (255, 255, 255))
                    if img.mode == 'P':
                        img = img.convert('RGBA')
                    background.paste(img, mask=img.split()[-1] if img.mode in ('RGBA', 'LA') else None)
                    img = background
                elif img.mode != 'RGB':
                    img = img.convert('RGB')

                # Resize if needed
                needs_resize = img.width > self.max_width or img.height > self.max_height
                if needs_resize:
                    img.thumbnail((self.max_width, self.max_height), Image.Resampling.LANCZOS)

                # Determine output format and save to temp file
                if self.convert_to_webp:
                    temp_path = image_path.with_suffix('.webp.tmp')
                    img.save(temp_path, 'WEBP', quality=self.webp_quality, method=6)
                elif image_path.suffix.lower() in ['.jpg', '.jpeg']:
                    img.save(temp_path, 'JPEG', quality=self.jpeg_quality, optimize=True)
                elif image_path.suffix.lower() == '.png':
                    img.save(temp_path, 'PNG', optimize=True)
                else:
                    # Copy other formats as-is
                    img.save(temp_path)

            optimized_size = temp_path.stat().st_size

            # Only replace if we got a size reduction or format change
            if optimized_size < original_size or self.convert_to_webp or needs_resize:
                # Handle format change (e.g., jpg -> webp)
                if self.convert_to_webp and image_path.suffix.lower() != '.webp':
                    final_path = image_path.with_suffix('.webp')
                    image_path.unlink()  # Remove original
                    temp_path.rename(final_path)
                    image_path = final_path
                else:
                    image_path.unlink()
                    temp_path.rename(image_path)

                self.stats['original_size'] += original_size
                self.stats['optimized_size'] += optimized_size
                self.stats['processed'] += 1

                # Mark as optimized
                self._mark_optimized(image_path, original_size, optimized_size)

                reduction = ((original_size - optimized_size) / original_size * 100) if original_size > 0 else 0
                resize_msg = f" + resized from {img.size[0]}x{img.size[1]}" if needs_resize else ""
                print(f"✓ {image_path.name} ({self._format_size(original_size)} → {self._format_size(optimized_size)}, {reduction:.1f}% reduction{resize_msg})")
            else:
                # No improvement, keep original
                temp_path.unlink()
                self.stats['skipped'] += 1
                print(f"→ {image_path.name} (no improvement, kept original)")

        except Exception as e:
            self.stats['errors'].append(f"{image_path}: {str(e)}")
            self.stats['skipped'] += 1
            print(f"✗ Error processing {image_path.name}: {str(e)}")
            # Clean up temp file if it exists
            if temp_path.exists():
                temp_path.unlink()

    def _format_size(self, size_bytes):
        """Format bytes to human-readable size"""
        for unit in ['B', 'KB', 'MB', 'GB']:
            if size_bytes < 1024.0:
                return f"{size_bytes:.1f}{unit}"
            size_bytes /= 1024.0
        return f"{size_bytes:.1f}TB"

    def process_directory(self):
        """Process all images in directory"""
        if not self.image_dir.exists():
            print(f"Error: Directory {self.image_dir} does not exist")
            sys.exit(1)

        # Supported image formats
        image_extensions = {'.jpg', '.jpeg', '.png', '.gif', '.webp'}

        print(f"Scanning {self.image_dir}...\n")

        # Walk through directory tree
        image_files = []
        for root, dirs, files in os.walk(self.image_dir):
            for file in files:
                file_path = Path(root) / file
                if file_path.suffix.lower() in image_extensions:
                    image_files.append(file_path)

        print(f"Found {len(image_files)} images\n")

        # Process images
        for image_path in sorted(image_files):
            self.optimize_image(image_path)

        # Save tracking data
        self._save_tracking()

    def print_summary(self):
        """Print optimization summary"""
        print("\n" + "="*60)
        print("OPTIMIZATION SUMMARY")
        print("="*60)
        print(f"Images processed: {self.stats['processed']}")
        print(f"Already optimized (skipped): {self.stats['already_optimized']}")
        print(f"No improvement (kept original): {self.stats['skipped']}")

        if self.stats['processed'] > 0:
            print(f"\nOriginal size: {self._format_size(self.stats['original_size'])}")
            print(f"Optimized size: {self._format_size(self.stats['optimized_size'])}")

            if self.stats['original_size'] > 0:
                total_reduction = ((self.stats['original_size'] - self.stats['optimized_size']) /
                                 self.stats['original_size'] * 100)
                print(f"Total reduction: {total_reduction:.1f}%")

        if self.stats['errors']:
            print(f"\nErrors encountered: {len(self.stats['errors'])}")
            for error in self.stats['errors'][:10]:  # Show first 10 errors
                print(f"  - {error}")

        print(f"\nTracking file: {self.tracking_file}")


def main():
    parser = argparse.ArgumentParser(
        description='Optimize images IN PLACE for web delivery. Run multiple times safely.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('directory', help='Directory containing images to optimize')
    parser.add_argument('--max-width', type=int, default=1920, help='Maximum image width')
    parser.add_argument('--max-height', type=int, default=1920, help='Maximum image height')
    parser.add_argument('--jpeg-quality', type=int, default=85, help='JPEG quality (1-100)')
    parser.add_argument('--webp-quality', type=int, default=85, help='WebP quality (1-100)')
    parser.add_argument('--webp', action='store_true', help='Convert all images to WebP format')
    parser.add_argument('--force', action='store_true', help='Re-optimize all images (ignore cache)')

    args = parser.parse_args()

    optimizer = ImageOptimizer(
        image_dir=args.directory,
        max_width=args.max_width,
        max_height=args.max_height,
        jpeg_quality=args.jpeg_quality,
        webp_quality=args.webp_quality,
        convert_to_webp=args.webp,
        force=args.force
    )

    print(f"Image Optimization - IN PLACE MODE")
    print(f"Directory: {args.directory}")
    print(f"Max dimensions: {args.max_width}x{args.max_height}")
    print(f"JPEG quality: {args.jpeg_quality}")
    print(f"WebP quality: {args.webp_quality}")
    print(f"Convert to WebP: {args.webp}")
    print(f"Force re-optimization: {args.force}")
    print("="*60 + "\n")

    if not args.force:
        print("Note: Already optimized images will be skipped. Use --force to re-optimize all.\n")

    optimizer.process_directory()
    optimizer.print_summary()


if __name__ == '__main__':
    main()
