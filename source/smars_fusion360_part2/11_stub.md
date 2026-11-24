---
title: Stub wheel profile
description: Create the Stub wheel profile
layout: lesson
type: page
redirect_from: /learn/smars_fusion360/27_stub.html
cover: /learn/smars_fusion360_part2/assets/banners/11_stub.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to sketch a half-section profile that can be revolved into a round stub wheel.

## Half a part is enough

The stub wheel is created using a profile that is then revolved around an axis to form a rounded part. That is how a lathe works: you describe one slice of a round object and spin it all the way round. Because a round part is the same all the way round, you only draw one side of it.

This is a complex part to describe so please refer to the diagram below for dimensions.

![Chassis stub wheel profile Cad Drawing](assets/chassis_stub_wheel_profile.png){:class="img-fluid w-75 m-3"}

## How to approach a profile like this

Work in this order so you do not get lost:

1. Draw the axis line the profile will be spun around (the centre of the wheel hole).
2. Draw the outline on one side of the axis.
3. Close the shape, because a revolve needs a closed profile.
4. Add the dimensions from the diagram, then check that every line is black.

```mermaid
flowchart LR
    A[Axis at the wheel hole centre] --> B[Outline on one side]
    B --> C[Close the shape]
    C --> D[Dimension from the diagram]
```

The shape of the profile decides the shape of the stub. A step in the outline becomes a ring on the part, and a slanted line becomes a cone, so the steps in the diagram are what give the stub the shape that holds a wheel on.

**Check after this step:** the profile is closed, dimensioned from the diagram, and sits on one side of the axis.

## Try it Yourself

1. Sketch the outline and zoom in on each corner to check they all join.
2. Pick one step in the profile and predict what ring it will make once revolved.
3. Challenge: if the profile crossed the axis, what do you think would go wrong when revolving?

## Common Issues

- **Problem**: The revolve will not accept the profile.
- **Solution**: Zoom in on the corners and join any gaps so the outline is a closed shape.
- **Why**: A revolve needs a closed profile so that it can make a solid.

- **Problem**: The profile is partly on both sides of the axis.
- **Solution**: Move all of it to one side of the axis line.
- **Why**: A profile that crosses the axis overlaps itself when spun, so the solid cannot be built.

## Check your understanding

{% quiz %}
? How is a round stub wheel described in a sketch?
- [ ] By drawing a full circle only
- [x] By drawing one side of its outline to be revolved around an axis
- [ ] By drawing two cubes
> The profile is revolved to form a rounded part.

? What must be true about the profile before a revolve?
- [x] It must be a closed shape
- [ ] It must be a full circle
- [ ] It must be hidden
> A revolve needs a closed profile.

? Where do the dimensions come from for this profile?
- [ ] A table in the lesson
- [x] The diagram in the lesson
- [ ] The video only
> The lesson says to refer to the diagram.
{% endquiz %}
