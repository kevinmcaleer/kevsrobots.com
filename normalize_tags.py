#!/usr/bin/env python3
"""
Normalize tags in Jekyll blog posts:
- Convert to lowercase
- Replace spaces with underscores
- Merge duplicate tags via mappings
"""

import os
import re
import glob

# Tag mappings for merging duplicates
TAG_MAPPINGS = {
    'raspberrypi': 'raspberry_pi',
    'raspberry pi': 'raspberry_pi',
    'raspberry_pi_pico_w': 'pico_w',
    '3dprinting': '3d_printing',
    '3d printing': '3d_printing',
    'robots': 'robot',
    'robotarms': 'robot_arms',
    'robot arms': 'robot_arms',
    'robotic arm': 'robot_arms',
    'robotic_arm': 'robot_arms',
    'micropyton': 'micropython',
    'lasercutting': 'laser_cutting',
    'laser cutting': 'laser_cutting',
}

def normalize_tag(tag):
    """Normalize a single tag"""
    if not tag or not isinstance(tag, str):
        return None

    tag = tag.strip().strip('"\'')
    if not tag:
        return None

    # Check mapping first (case insensitive)
    tag_lower = tag.lower()
    if tag_lower in TAG_MAPPINGS:
        return TAG_MAPPINGS[tag_lower]

    # Standard normalization
    tag = tag_lower
    tag = tag.replace(' ', '_')
    tag = tag.replace('-', '_')

    # Check mapping again after normalization
    if tag in TAG_MAPPINGS:
        return TAG_MAPPINGS[tag]

    # Remove double underscores
    while '__' in tag:
        tag = tag.replace('__', '_')

    tag = tag.strip('_')
    return tag if tag else None

def process_file(filepath):
    """Process a single markdown file - preserve structure, only change tag values"""
    with open(filepath, 'r', encoding='utf-8') as f:
        content = f.read()

    # Must start with frontmatter
    if not content.startswith('---'):
        return False

    lines = content.split('\n')

    # Find frontmatter boundaries
    fm_start = 0
    fm_end = None
    for i, line in enumerate(lines[1:], start=1):
        if line.strip() == '---':
            fm_end = i
            break

    if fm_end is None:
        return False

    # Process lines, only modifying tag values
    new_lines = lines.copy()
    in_tags = False
    modified = False

    for i in range(1, fm_end):
        line = lines[i]
        stripped = line.strip()

        if stripped.startswith('tags:'):
            in_tags = True
            # Check for inline tags
            after_colon = stripped[5:].strip()
            if after_colon:
                # Inline format like: tags: [tag1, tag2] or tags: tag1, tag2
                # Remove brackets
                after_colon = after_colon.strip('[]')
                original_tags = [t.strip().strip('"\'') for t in after_colon.split(',')]
                normalized = []
                for t in original_tags:
                    norm = normalize_tag(t)
                    if norm and norm not in normalized:
                        normalized.append(norm)

                if normalized != [t.lower().replace(' ', '_').replace('-', '_') for t in original_tags]:
                    modified = True

                # Preserve indentation
                indent = len(line) - len(line.lstrip())
                new_lines[i] = ' ' * indent + 'tags:'
                # Insert new tag lines after this one
                # We'll handle this by rebuilding the section
                in_tags = False
            continue

        if in_tags:
            if stripped.startswith('- '):
                # This is a tag item
                tag = stripped[2:].strip().strip('"\'')
                norm = normalize_tag(tag)
                if norm and norm != tag:
                    modified = True
                    # Preserve indentation
                    indent = len(line) - len(line.lstrip())
                    new_lines[i] = ' ' * indent + '- ' + norm
            elif stripped == '':
                continue
            elif not stripped.startswith('-'):
                # End of tags section
                in_tags = False

    if modified:
        new_content = '\n'.join(new_lines)
        with open(filepath, 'w', encoding='utf-8') as f:
            f.write(new_content)
        return True

    return False

def main():
    posts_dir = 'web/_posts'

    if not os.path.exists(posts_dir):
        print(f"Directory {posts_dir} not found.")
        return

    files = glob.glob(os.path.join(posts_dir, '*.md'))

    modified_count = 0
    for filepath in sorted(files):
        try:
            if process_file(filepath):
                print(f"Modified: {os.path.basename(filepath)}")
                modified_count += 1
        except Exception as e:
            print(f"Error processing {filepath}: {e}")

    print(f"\nDone! Modified {modified_count} files.")

if __name__ == '__main__':
    main()
