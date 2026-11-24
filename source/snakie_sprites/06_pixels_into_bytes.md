---
title: Pixels into Bytes
description: See how a row of pixels is packed into bytes and decode a sprite frame by hand in MicroPython
layout: lesson
type: page
cover: /learn/snakie_sprites/assets/banners/06_pixels_into_bytes.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to explain how a row of pixels is packed into bytes and decode a sprite frame by hand in MicroPython.

## What's in those strange lines?

The module you exported has lines like this in it:

```python
b'\x00\x00\x00\x00\x0f\xf0\x1f\xf8\x3f\xfc...'
```

That's your open eye. This lesson shows you how to read it. You don't have to do this by hand in real projects, but once you've seen how it works, sprites stop being magic and bugs get much easier to find.

---

## One row, two bytes

Take the third row of the eye, the first one with anything in it:

```
....########....
```

Write a 1 for each lit pixel and a 0 for each dark one:

```
0000111111110000
```

A byte holds eight bits, so split the row into two groups of eight:

```
00001111   11110000
```

Each group is one byte. Programmers usually write bytes in **hexadecimal**, where each digit stands for four bits:

| Bits | Hex | Bits | Hex |
|---|---|---|---|
| 0000 | 0 | 1000 | 8 |
| 0001 | 1 | 1001 | 9 |
| 0010 | 2 | 1010 | a |
| 0011 | 3 | 1011 | b |
| 0100 | 4 | 1100 | c |
| 0101 | 5 | 1101 | d |
| 0110 | 6 | 1110 | e |
| 0111 | 7 | 1111 | f |
{:class="table table-single"}

So `0000 1111` is `0f` and `1111 0000` is `f0`. Look at the exported line again: after four `\x00` bytes for the two blank rows, there they are, `\x0f\xf0`.

The next row down is `...##########...`, which is `0001 1111 1111 1000`, or `\x1f\xf8`. It's all there, row by row, top to bottom.

---

## The rules of the format

The layout has a name, `MONO_HLSB`, and three rules:

1. Rows are stored one after another, top row first
2. Within a byte, the **leftmost** pixel is the biggest bit
3. Every row starts on a fresh byte. A 12-pixel row still takes 2 bytes, and the spare 4 bits are left as 0

A 16×16 frame is therefore 16 rows of 2 bytes, 32 bytes in all.

---

## Decode it yourself

This box holds the open-eye frame and unpacks it using those rules:

{% capture snakie_code %}WIDTH = 16
HEIGHT = 16
FRAME = b'\x00\x00\x00\x00\x0f\xf0\x1f\xf8\x3f\xfc\x3f\xfc\x3c\x3c\x3c\x3c\x3c\x3c\x3c\x3c\x3f\xfc\x3f\xfc\x1f\xf8\x0f\xf0\x00\x00\x00\x00'

row_bytes = (WIDTH + 7) // 8

for y in range(HEIGHT):
    line = ""
    for x in range(WIDTH):
        byte = FRAME[y * row_bytes + x // 8]
        mask = 0x80 >> (x % 8)
        line += "#" if byte & mask else "."
    print(line)
{% endcapture %}
{% include snakie.html code=snakie_code name="decode.py" view="code" %}

Press **Run** and the eye appears.

Here's what the inner loop is doing:

- `y * row_bytes` skips to the start of the row
- `x // 8` picks the byte within the row: byte 0 for pixels 0 to 7, byte 1 for pixels 8 to 15
- `0x80 >> (x % 8)` makes a **mask** with a single 1 in the position of the pixel we want. `0x80` is `10000000`, and `>>` slides that 1 to the right
- `byte & mask` is zero if the pixel is off and non-zero if it's on

---

## Edit a sprite with a keyboard

In the box above, find the pupil rows. They're the four `\x3c\x3c` pairs: `0011 1100 0011 1100`.

Change the first pair to `\x3f\xfc` and run it again. You've just filled in the top row of the pupil without opening the editor.

Now you know why the format is so compact, and why it's still worth having a drawing tool.

---

## Try it Yourself

1. Change the code to print `██` for lit pixels and two spaces for dark ones. The eye will look squarer
2. Work out the two bytes for this row by hand, then check with the table: `..####....####..`
3. Make the eye wink: replace the right-hand byte of each pupil row so that side of the pupil is filled in
4. Flip the picture upside down by changing one thing in the `for y` line. (Hint: `range(HEIGHT - 1, -1, -1)`)

---

## Common Issues

**Problem**: `IndexError: bytes index out of range`.
**Solution**: Check the frame has exactly `row_bytes * HEIGHT` bytes, and that you didn't delete a `\x` pair while editing.
**Why**: The loop reads 32 bytes. A shorter frame runs out before the last row.

**Problem**: The picture comes out mirrored.
**Solution**: Use `0x80 >> (x % 8)`, not `1 << (x % 8)`.
**Why**: In `MONO_HLSB` the leftmost pixel is the biggest bit, so the mask has to start at the left and move right.

**Problem**: The picture is sheared, with each row shifted along.
**Solution**: Make sure `row_bytes` is `(WIDTH + 7) // 8`.
**Why**: Rows always start on a whole byte. Getting the row length wrong makes every row start in the wrong place.

---

## Check your understanding

{% quiz %}
? How many bytes does one row of a 16-pixel-wide sprite take?
- [ ] 1
- [x] 2
- [ ] 16
> Eight pixels fit in a byte, so 16 pixels need two.

? Which hex byte is the row 00001111?
- [x] 0f
- [ ] f0
- [ ] ff
> 0000 is 0 and 1111 is f.

? A sprite is 12 pixels wide. How many bytes is each row?
- [ ] 1.5
- [ ] 1
- [x] 2
> Every row is padded to a whole number of bytes, so 12 pixels take 2 bytes with 4 spare bits.
{% endquiz %}
