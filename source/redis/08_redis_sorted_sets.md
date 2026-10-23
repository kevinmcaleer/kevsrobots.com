---
layout: lesson
title: Redis Sorted Sets
type: page
description: Learn about Redis sorted sets and how to use them with Python.
---

<!-- ![Cover photo of Redis sorted sets](assets/redis-sorted-sets.jpg){:class="cover"} -->

### Overview

In this lesson, we will cover Redis sorted sets and how to use them with Python. Redis sorted sets are a data type that can be used to store a collection of unique elements with an associated score. We will cover how to add, retrieve, and manipulate sorted set elements in Redis, and how to use sorted sets in Python.

---

### Lesson Content

In this lesson, we will cover:

* [Redis sorted sets](#redis-sorted-sets)
* [Adding and retrieving sorted set elements](#adding-and-retrieving-sorted-set-elements)
* [Manipulating sorted set elements](#manipulating-sorted-set-elements)
* [Using Redis sorted sets with Python](#using-redis-sorted-sets-with-python)

---

### Redis Sorted Sets

Redis sorted sets are a data type that can be used to store a collection of unique elements with an associated score. Sorted sets in Redis are implemented as a combination of a hash table and a skip list, which means that adding, removing, and retrieving sorted set elements is very fast.

---

### Adding and Retrieving Sorted Set Elements

To add an element with an associated score to a sorted set in Redis, you can use the `zadd` command. To retrieve the score of an element in a sorted set, you can use the `zscore` command. Here are some examples of adding and retrieving elements from a Redis sorted set:

```redis
zadd myzset 1 "element1"
(integer) 1
zadd myzset 2 "element2"
(integer) 1
zscore myzset "element1"
"1"
```

---

### Manipulating Sorted Set Elements

In addition to adding and retrieving sorted set elements, Redis provides several commands for manipulating sorted set elements. Here are some examples of manipulating sorted set elements in Redis:

* `zrem`: Remove an element from a sorted set
* `zrange`: Get a range of elements by rank (their position in score order, lowest first)
* `zrevrange`: The same, highest score first. Newer Redis versions prefer `zrange` with the `REV` option
* `zcard`: Get the number of elements in a sorted set

---

### Using Redis Sorted Sets with Python

To use Redis sorted sets in Python, you can use the `sortedset` methods of the Redis Python library. Here's an example of how to use Redis sorted sets in Python:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)

# Add elements with associated scores to a sorted set
redis_client.zadd('myzset', {'element1': 1, 'element2': 2})

# Get the score of an element in a sorted set
score = redis_client.zscore('myzset', 'element1')

# Remove an element from a sorted set
redis_client.zrem('myzset', 'element2')

# Get the first two elements, lowest score first
elements = redis_client.zrange('myzset', 0, 1)

# Get the first two elements, highest score first
elements_rev = redis_client.zrange('myzset', 0, 1, desc=True)

# Get the number of elements in a sorted set
count = redis_client.zcard('myzset')
```

In this example, we use the Redis Python library to add elements with associated scores to a sorted set using the `zadd` method, retrieve the score of an element in a sorted set using the `zscore` method, remove an element from a sorted set using the `zrem` method, get a range of elements in score order using the `zrange` method (and in reverse order with `desc=True`), and get the number of elements in a sorted set using the `zcard` method.

---

### Example: a leaderboard

```python
redis_client.zadd('leaderboard', {'alice': 120, 'bob': 95, 'carol': 150})
redis_client.zincrby('leaderboard', 40, 'bob')   # bob scores 40 more

top3 = redis_client.zrange('leaderboard', 0, 2, desc=True, withscores=True)
for rank, (name, score) in enumerate(top3, start=1):
    print(rank, name, int(score))
```

`zincrby()` adds to a player's score, and Redis keeps everyone in order automatically.

---

{% quiz %}
? In a sorted set, what must be unique?
- [x] The elements
- [ ] The scores
- [ ] Both
> Two players can have the same score, but each player appears only once.

? How do you get the highest scores first in Python?
- [ ] `zrange('leaderboard', 0, 2)`
- [x] `zrange('leaderboard', 0, 2, desc=True)`
> `desc=True` reverses the order.
{% endquiz %}

---

### Key Takeaways

After completing this lesson, you should be able to:

* Describe Redis sorted sets and their implementation in Redis
* Add and retrieve elements with associated scores in Redis sorted sets
* Manipulate elements in Redis sorted sets
* Use Redis sorted sets in Python

---

### Conclusion

In this lesson, we covered Redis sorted sets and how to use them with Python. We described Redis sorted sets and their implementation in Redis, and demonstrated how to add, retrieve, and manipulate elements with associated scores in Redis sorted sets, and how to use Redis sorted sets in Python using the Redis Python library. In the next lesson, we will cover Redis pub/sub.

---
