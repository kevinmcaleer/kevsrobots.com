---
title: Subqueries and Nested Queries
description: Put one SELECT inside another with scalar subqueries, IN, EXISTS and correlated subqueries, and avoid the NOT IN and NULL trap.
layout: lesson
type: page
cover: /learn/sql/assets/banners/12_subqueries_and_nested_queries.jpg
date_updated: 2026-11-24
---

![Course Cover Image]({{page.cover}}){:class="cover"}

---

By the end of this lesson you will be able to write a query inside another query, choose between `IN` and `EXISTS`, and explain why `NOT IN` can silently return nothing.

---

Some questions need two steps: "which sensors cost more than the **average** sensor?" First find the average, then compare. A **subquery** lets you do both in one statement, by putting a `SELECT` in brackets inside another.

```mermaid
flowchart LR
    A["Inner query runs: AVG(price) = 2.63"] --> B["Outer query uses the answer"]
    B --> C["WHERE price > 2.63"]
    C --> D["MPU6050, VL53L0X"]
```

## A subquery that returns one value

```sql
SELECT model, price
FROM sensors
WHERE price > (SELECT AVG(price) FROM sensors)
ORDER BY price;
```

```text
model    price
-------  -----
MPU6050  3.8
VL53L0X  6.0
```

The inner query gives a single number (about 2.63), and the outer query treats it like any other value. The same trick finds robots whose battery beats the average:

```sql
SELECT name, battery_mah
FROM robots
WHERE battery_mah > (SELECT AVG(battery_mah) FROM robots);
```

```text
name   battery_mah
-----  -----------
Smars  2000
```

## A subquery that returns a list: IN

If the inner query returns a column of values, test against it with `IN`. Which robots carry a distance sensor?

```sql
SELECT name
FROM robots
WHERE robot_id IN (SELECT robot_id FROM sensors WHERE kind = 'distance')
ORDER BY name;
```

```text
name
-----------
Pico Walker
Smars
```

You could write this as a join too. Subqueries often read more naturally when you only need to *test* for membership, and don't need columns from the second table.

## EXISTS: is there at least one?

`EXISTS` asks "does the inner query find any row?" It is true or false, and doesn't care what the row contains (so people write `SELECT 1`). Robots that have a line sensor:

```sql
SELECT name
FROM robots AS r
WHERE EXISTS (
    SELECT 1 FROM sensors AS s
    WHERE s.robot_id = r.robot_id AND s.kind = 'line'
)
ORDER BY name;
```

```text
name
-----------
Line Chaser
Smars
```

Notice the inner query mentions `r.robot_id` from the **outer** query. That makes it a **correlated subquery**: it is re-run for each robot in turn, with that robot's ID filled in. Put `NOT` in front, and you find robots with no sensors at all:

```sql
SELECT name
FROM robots AS r
WHERE NOT EXISTS (SELECT 1 FROM sensors AS s WHERE s.robot_id = r.robot_id)
ORDER BY name;
```

```text
name
-----------
Arm Buddy
Mystery Bot
```

## The NOT IN trap

It's tempting to write the same question with `NOT IN`:

```sql
SELECT name FROM robots
WHERE robot_id NOT IN (SELECT robot_id FROM sensors);
```

```text
name
----
```

**Nothing came back**, even though Arm Buddy and Mystery Bot clearly have no sensors. The culprit is our spare VL53L0X, whose `robot_id` is `NULL`. The list contains an unknown value, so "is robot 3 not in the list?" can never be answered with a firm yes. Either filter the `NULL` out:

```sql
SELECT name FROM robots
WHERE robot_id NOT IN (SELECT robot_id FROM sensors WHERE robot_id IS NOT NULL)
ORDER BY name;
```

or use `NOT EXISTS`, which has no such problem. Prefer `NOT EXISTS`.

## Subqueries in the SELECT list

A single-value subquery can sit in the column list. This counts each robot's sensors, one robot at a time:

```sql
SELECT name,
       (SELECT COUNT(*) FROM sensors AS s WHERE s.robot_id = r.robot_id) AS sensor_count
FROM robots AS r
ORDER BY name;
```

```text
name         sensor_count
-----------  ------------
Arm Buddy    0
Line Chaser  2
Mystery Bot  0
Pico Walker  2
Smars        2
```

A `LEFT JOIN` with `GROUP BY` (lesson 9) gives the same answer, often faster on big tables.

## Subqueries in FROM

You can also treat a query's result as a temporary table, as long as you give it an alias. Which kinds of sensor cost more than 2 on average?

```sql
SELECT kind, ROUND(avg_price, 2) AS avg_price
FROM (
    SELECT kind, AVG(price) AS avg_price
    FROM sensors
    GROUP BY kind
) AS per_kind
WHERE avg_price > 2
ORDER BY kind;
```

```text
kind      avg_price
--------  ---------
distance  3.67
motion    3.8
```

## Try it Yourself

1. List the readings that are above the overall average reading, highest first.
2. Use `IN` with a subquery on `builds` to find the robots built by `Sam`. (You'll need a second, nested subquery on `builders`.)
3. Challenge: find sensors that cost more than the average **for their own kind**. Hint: a correlated subquery with `WHERE kind = s.kind`. Only the VL53L0X should appear.

## Common Issues

- **Problem**: `NOT IN` returns no rows, although I know there are matches.
- **Solution**: Switch to `NOT EXISTS`, or add `WHERE column IS NOT NULL` inside the subquery.
- **Why**: If the list contains `NULL`, the comparison is unknown for every row.

- **Problem**: My `= (SELECT ...)` subquery returns more than one row, but SQLite doesn't complain and I only see one answer.
- **Solution**: Use `IN` when the inner query can return several rows.
- **Why**: SQLite quietly uses the first row of the inner query. MySQL and PostgreSQL stop with an error, which is the safer behaviour.

- **Problem**: `no such column` when the subquery refers to the outer table.
- **Solution**: Check that the outer table has an alias (like `r`) and you spelt it the same way inside.
- **Why**: A correlated subquery finds the outer row through that alias.

## Check your understanding

{% quiz %}
? What makes a subquery "correlated"?
- [ ] It returns exactly one row
- [x] It refers to a column from the outer query, so it is evaluated for each outer row
- [ ] It uses `JOIN` inside
> In our `EXISTS` example, `s.robot_id = r.robot_id` links the inner query to the current outer robot.

? Why did `NOT IN (SELECT robot_id FROM sensors)` return nothing?
- [ ] No robot is missing a sensor
- [x] One `robot_id` in the list is `NULL`, which makes every `NOT IN` check unknown
- [ ] `NOT IN` is not allowed with subqueries
> The spare VL53L0X has a `NULL` robot. `NOT EXISTS` avoids the problem.

? When must a subquery in `FROM` be given an alias?
- [x] Always
- [ ] Only when it contains a `GROUP BY`
- [ ] Never
> The temporary result needs a name so the outer query can refer to its columns, like `per_kind` above.
{% endquiz %}

Now let's make our favourite queries reusable, and make them fast.
