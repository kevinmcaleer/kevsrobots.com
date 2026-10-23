---
title: Attach Tracks
description: The tracks provide all terrain grip for the robot
layout: lesson
---

## Attach Tracks

To give the wheels more traction on different surfaces you can either attach the 8 tracks to each wheel, or attach 16 tracks to each side, in a caterpillar arrangement:

![caterpillar tracks](assets/tracks.png){:class="img-fluid w-50"}

The tracks are attached to each other by using a piece of filament.

![Tracks](assets/caterpillar_tracks.png){:class="img-fluid w-50"}

---

## Troubleshooting

You may find that the tracks are a really tight fit when connecting up in the caterpillar track formation. There are a couple of solutions to this:

* print another track at 110% - 120% scaling on the Y Axis
* use TPU / flexible filament, as this can stretch
* use a pre-sized piece from [Thingiverse](https://www.thingiverse.com/thing:3195991)

## Check your understanding

{% quiz %}
? A motor spins backwards when it should go forwards. What's the easiest fix?
- [x] Swap that motor's two wires in the motor shield connector
- [ ] Rebuild the chassis
- [ ] Use a bigger battery
> Swapping the wires reverses the motor. You could also fix it in code.

? Your caterpillar tracks are too tight. Which of these could help?
- [ ] Print the tracks at 50% infill
- [x] Print a track scaled to 110-120% on the Y axis, or use flexible TPU filament
- [ ] Remove the unpowered wheels
> A slightly longer track or a stretchy filament gives the loop the extra length it needs.
{% endquiz %}
