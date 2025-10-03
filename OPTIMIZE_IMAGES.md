# Image Optimization Guide

## Quick Start

Optimize all images in the website:

```bash
# Install Pillow if needed
pip install Pillow

# Run optimization (safe to run multiple times)
python3 optimize_images.py web/assets/img
```

## How It Works

The script optimizes images **IN PLACE** and tracks what's been optimized in `.optimization_cache.json`.

**Key features:**
- ✅ Safe to run multiple times (skips already optimized images)
- ✅ Only replaces images if they get smaller or are resized
- ✅ Tracks optimization history with MD5 hashes
- ✅ Keeps originals if optimization doesn't help
- ✅ Can be committed to git (optimized images replace originals)

## Usage Options

### Basic optimization (resize + quality)
```bash
python3 optimize_images.py web/assets/img
```
- Resizes images to max 1920x1920
- Compresses JPEGs to quality 85
- Optimizes PNGs
- Only replaces if smaller

### Convert to WebP (better compression)
```bash
python3 optimize_images.py web/assets/img --webp
```
- Converts all images to WebP format
- 60-80% smaller than JPEG
- Supported by all modern browsers

### Custom quality settings
```bash
python3 optimize_images.py web/assets/img --jpeg-quality 90 --webp-quality 90
```

### Force re-optimization
```bash
python3 optimize_images.py web/assets/img --force
```
Re-processes all images even if previously optimized.

## Quality vs Size

| Quality Setting | File Size | Visual Quality | Recommended For |
|----------------|-----------|----------------|-----------------|
| `--jpeg-quality 95` | Larger | Excellent | Hero images, photography |
| `--jpeg-quality 85` | Medium | Very good | **Default - recommended** |
| `--jpeg-quality 75` | Smaller | Good | Blog thumbnails |
| `--jpeg-quality 60` | Smallest | Acceptable | Backgrounds, decorative |

## Resolution Settings

Current images are often 4032x3024 (12MP camera):

| Setting | Description | Use Case |
|---------|-------------|----------|
| `--max-width 1920` | Full HD | **Default - recommended** |
| `--max-width 2560` | 2K displays | Larger files, better on 4K screens |
| `--max-width 1280` | HD | Faster loading, mobile-friendly |

## Workflow

### Initial Optimization

1. **Backup first** (optional):
   ```bash
   git commit -am "Backup before optimization"
   ```

2. **Run optimization**:
   ```bash
   python3 optimize_images.py web/assets/img
   ```

3. **Review results**:
   - Check the summary statistics
   - Spot-check a few images for quality
   - View the site locally to ensure images look good

4. **Commit optimized images**:
   ```bash
   git add web/assets/img
   git commit -m "Optimize images for web delivery

   - Resized to max 1920px
   - Compressed to quality 85
   - Reduced image directory from 1.1GB to ~300MB"
   ```

### Adding New Images

When you add new images to the site:

1. Add the images normally (full resolution from Canva/camera)
2. Run the optimizer again:
   ```bash
   python3 optimize_images.py web/assets/img
   ```
3. Only new images will be processed (existing optimized images skipped)
4. Commit the new optimized images

### Re-optimizing Everything

To change quality settings for all images:

```bash
# Higher quality (larger files)
python3 optimize_images.py web/assets/img --jpeg-quality 90 --force

# Lower quality (smaller files)
python3 optimize_images.py web/assets/img --jpeg-quality 75 --force
```

## Tracking File

The script creates `.optimization_cache.json` in the image directory to track:
- Which files have been optimized
- MD5 hash of optimized version
- Original vs optimized size
- Optimization settings used
- When it was optimized

**This file is git-ignored** - it's only used locally.

## Reverting to Originals

If you need the original high-res images:

```bash
# Revert to previous commit
git checkout HEAD~1 web/assets/img

# Or revert specific file
git checkout HEAD~1 web/assets/img/blog/specific-image.jpg
```

## WebP Conversion

WebP provides better compression (60-80% smaller than JPEG) and is supported by all modern browsers.

**To convert everything to WebP:**

```bash
python3 optimize_images.py web/assets/img --webp
```

**Note:** This will:
- Convert `.jpg` → `.webp`, `.png` → `.webp`
- Delete original JPG/PNG files
- Update file extensions (you may need to update image references in markdown)

**Browser support:**
- Chrome/Edge: ✓
- Firefox: ✓
- Safari: ✓ (v14+, 2020)
- Mobile: ✓

## Expected Results

Based on current stats (1.1GB images, 1,501 files):

- **Without WebP:** 1.1GB → ~300-400MB (65-75% reduction)
- **With WebP:** 1.1GB → ~150-250MB (75-85% reduction)
- **Processing time:** 5-10 minutes
- **Image quality:** Visually identical at quality 85

## Dockerfile Integration

The Dockerfile no longer optimizes images during build. Instead:

1. Optimize images manually using this script
2. Commit optimized images to git
3. Dockerfile clones repo with already-optimized images
4. Faster builds (no optimization step needed)

To use the enhanced nginx config with caching:

```bash
cd /Users/kev/Web/ClusteredPi/stacks/kevsrobots
# Copy the optimized nginx config
cp nginx-optimized.conf nginx.conf
# Or update dockerfile to use nginx-optimized.conf
```

## Troubleshooting

**"ModuleNotFoundError: No module named 'PIL'"**
```bash
pip install Pillow
```

**"Already optimized (skipped)" for all images**
The images were already optimized. Use `--force` to re-optimize.

**Images look too compressed**
Increase quality setting:
```bash
python3 optimize_images.py web/assets/img --jpeg-quality 90 --force
```

**Want to start fresh**
Delete the tracking cache:
```bash
rm web/assets/img/.optimization_cache.json
```

## Performance Impact

After optimization + nginx caching:

- **First visit:** 60-80% faster page loads
- **Return visits:** 90% faster (cached assets)
- **Bandwidth:** 70-85% reduction
- **Mobile:** Significantly faster on 4G/5G
- **SEO:** Better Core Web Vitals scores
