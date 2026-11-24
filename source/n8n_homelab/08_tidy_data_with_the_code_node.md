---
title: Tidy Data with the Code Node
description: Use the n8n Code node and a little JavaScript to reshape sensor readings and flag a hot room
layout: lesson
type: page
cover: /learn/n8n_homelab/assets/banners/08_tidy_data_with_the_code_node.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

Stage 5: the butler thinks for itself. The nodes you've used so far handle most jobs, but sometimes you want to reshape data in a way no node offers. That's what the **Code** node is for, and we'll use JavaScript.

---

## Two modes

| Mode | Runs | Use it when |
|---|---|---|
| Run Once for All Items | One time, with every incoming item (the default) | Totals, sorting, building one summary message |
| Run Once for Each Item | Once per item | Changing every item in the same way |
{:class="table table-single"}

A Code node can use JavaScript or Python. We use JavaScript, which runs on Node.js. Built-in variables and methods start with a `$`, and `console.log` writes to your browser's developer console, which is handy for debugging.

## The rule you must remember

Data between nodes is an array of items, and each item keeps its data under a `json` key. Your code must return that shape:

```javascript
return [
  { json: { room: "lounge", temperature_c: 21.5 } },
  { json: { room: "shed", temperature_c: 4.0 } },
];
```

n8n adds a missing `json` key and a missing outer array for you, but returning the full shape yourself avoids surprises. The value of `json` must be an **object**, not an array.

## Tidy a Pico reading

Add a Code node after the Webhook in your sensor inbox workflow. Remember that the Pico's data arrives inside `body`. This code, in **Run Once for All Items** mode, turns each raw reading into something tidier and adds a verdict:

```javascript
// Run Once for All Items
const items = $input.all();

return items.map((item) => {
  const body = item.json.body;
  const temperature = Number(body.temperature_c);

  return {
    json: {
      device: body.device,
      temperature_c: temperature,
      temperature_f: Math.round((temperature * 9 / 5 + 32) * 10) / 10,
      status: temperature > 28 ? "too hot" : "ok",
      received_at: new Date().toISOString(),
    },
  };
});
```

Walk through it:

1. `$input.all()` gives every incoming item.
2. `.map()` builds one new item for each.
3. `Number(...)` makes sure the temperature is a number, in case it arrived as text.
4. The ternary `temperature > 28 ? "too hot" : "ok"` picks the status.
5. `received_at` stamps the time n8n received the reading, which matters because the Pico has no clock of its own.

## A hot-room alert

Add an **IF** node after the Code node. Set a **String** condition: `{{ $json.status }}` **is equal to** `too hot`. Send the true branch to a **Send Email** node, like lesson 5. The butler now tells you when the lounge overheats.

**Where does the data live?** n8n keeps each run's data in its executions, until they are pruned (lesson 10 covers that). For long-term history you would send readings to a database or a spreadsheet, using one of n8n's database nodes. That's a good next project.

**What the Code node can't do:** it can't read files or make web requests. Use the Read/Write File From Disk or HTTP Request nodes for those, then hand the data to your code.

## Try it Yourself

1. Change the threshold to 20 and watch the status flip.
2. Add a field `room` that is `"lounge"` when `device` is `"pico-lounge"`, and `"unknown"` otherwise.
3. Challenge: switch to **Run Once for Each Item** and rewrite the code. In that mode you read the current item with `$input.item` and return a single `{ json: ... }` object.

## Common Issues

**Problem**: "Code doesn't return items properly".
**Solution**: Return an array of objects, each with a `json` key.
**Why**: n8n passes items between nodes in that shape.

**Problem**: "A 'json' property isn't an object".
**Solution**: Put an object in `json`, never an array.
**Why**: Each item is one record with named fields.

**Problem**: `Cannot read properties of undefined (reading 'temperature_c')`.
**Solution**: Read `item.json.body`, not `item.json`.
**Why**: The Webhook node wraps the request in `body`, `headers`, `params` and `query`.

## Check your understanding

{% quiz %}
? A Pico POSTs `{"temperature_c": 21.5}` to a Webhook node. Where is the temperature in the item?
- [ ] `json.temperature_c`
- [x] `json.body.temperature_c`
- [ ] `json.query.temperature_c`
> The Webhook node puts the request body under `body`, next to `headers`, `params` and `query`.

? What must a Code node return?
- [ ] A plain string
- [x] An array of items, each with its data as an object under `json`
- [ ] Nothing, it edits the items in place
> Items flow between nodes in that shape, and `json` must hold an object.

? Which job can't the Code node do by itself?
- [ ] Do arithmetic
- [x] Make an HTTP request
- [ ] Build a message string
> Use the HTTP Request node for that, then pass the result to the Code node.
{% endquiz %}
