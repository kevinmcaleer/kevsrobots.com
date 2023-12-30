---
layout: lesson
title: Redis Sets
author: Kevin McAleer
type: page
cover: /learn/redis/assets/redis-cover.jpg
date: 2023-04-12
previous: 05_redis_lists.html
next: 07_redis_hashes.html
description: Learn about Redis sets and how to use them with Python.
percent: 60
duration: 3
navigation:
- name: Using Redis with Python
- content:
  - section: Introduction to Redis
    content:
    - name: Introduction to Redis
      link: 01_introduction.html
    - name: Redis Basics
      link: 02_redis_basics.html
  - section: Working with data
    content:
    - name: Redis Data Types
      link: 03_data_types.html
    - name: Redis Commands and Transactions
      link: 04_commands_and_transactions.html
    - name: Redis Lists
      link: 05_redis_lists.html
  - section: Sets, Hashes and Sorted Sets
    content:
    - name: Redis Sets
      link: 06_redis_sets.html
    - name: Redis Hashes
      link: 07_redis_hashes.html
    - name: Redis Sorted Sets
      link: 08_redis_sorted_sets.html
  - section: Publish and Subscribe
    content:
    - name: Redis Pub/Sub
      link: 09_redis_pub_sub.html
  - section: Conclusion
    content:
    - name: Conclusion and Additional Resources
      link: 10_summary.html
---


<!-- ![Cover photo of Redis sets](assets/redis-sets.jpg){:class="cover"} -->

### Overview

In this lesson, we will cover Redis sets and how to use them with Python. Redis sets are a data type that can be used to store an unordered collection of unique strings. We will cover how to add, retrieve, and manipulate set elements in Redis, and how to use sets in Python.

---

### Lesson Content

In this lesson, we will cover:

* [Redis sets](#redis-sets)
* [Adding and retrieving set elements](#adding-and-retrieving-set-elements)
* [Manipulating set elements](#manipulating-set-elements)
* [Using Redis sets with Python](#using-redis-sets-with-python)

---

### Redis Sets

Redis sets are a data type that can be used to store an unordered collection of unique strings. Sets in Redis are implemented as hash tables, which means that adding, removing, and checking for membership of elements in a set is very fast.

---

### Adding and Retrieving Set Elements

To add an element to a set in Redis, you can use the `sadd` command. To check if an element is a member of a set, you can use the `sismember` command. Here are some examples of adding and retrieving elements from a Redis set:

```redis
sadd myset "element1"
(integer) 1
sadd myset "element2"
(integer) 1
sismember myset "element1"
(integer) 1
```

---

### Manipulating Set Elements

In addition to adding and retrieving set elements, Redis provides several commands for manipulating set elements. Here are some examples of manipulating set elements in Redis:

* `srem`: Remove an element from a set
* `spop`: Remove and return a random element from a set
* `smembers`: Get all members of a set

---

### Using Redis Sets with Python

To use Redis sets in Python, you can use the `set` methods of the Redis Python library. Here's an example of how to use Redis sets in Python:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379)

# Add elements to a set
redis_client.sadd('myset', 'element1', 'element2')

# Check if an element is a member of a set
is_member = redis_client.sismember('myset', 'element1')

# Remove an element from a set
redis_client.srem('myset', 'element2')

# Get all members of a set
members = redis_client.smembers('myset')
```

In this example, we use the Redis Python library to add elements to a set using the `sadd` method, check if an element is a member of a set using the `sismember` method, remove an element from a set using the `srem` method, and get all members of a set using the `smembers` method.

---

### Key Takeaways

After completing this lesson, you should be able to:

* Describe Redis sets and their implementation in Redis
* Add and retrieve elements from Redis sets
* Manipulate elements in Redis sets
* Use Redis sets in Python

---

### Conclusion

In this lesson, we covered Redis sets and how to use them with Python. We described Redis sets and their implementation in Redis, and demonstrated how to add, retrieve, and manipulate set elements in Redis, and how to use Redis sets in Python using the Redis Python library. In the next lesson, we will cover Redis hashes.

---
