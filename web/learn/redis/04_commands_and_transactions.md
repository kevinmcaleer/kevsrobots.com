---
layout: lesson
title: Redis Commands and Transactions
author: Kevin McAleer
type: page
cover: /learn/redis/assets/redis-cover.jpg
date: 2023-04-12
previous: 03_data_types.html
next: 05_redis_lists.html
description: Learn about Redis commands and transactions, and how to use them in Python.
percent: 40
duration: 4
quiz_total: 14
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


<!-- ![Cover photo of Redis commands and transactions](assets/redis-commands.jpg){:class="cover"} -->

### Overview

In this lesson, we will cover Redis commands and transactions, and how to use them in Python. Redis provides a rich set of commands for working with data, and transactions for executing multiple commands atomically. We will cover how to use Redis commands and transactions in Python, and how to handle errors and exceptions.

---

### Lesson Content

In this lesson, we will cover:

* [Redis commands and transactions](#redis-commands-and-transactions)
* [Redis transactions in Python](#redis-transactions-in-python)
* [Handling errors and exceptions](#handling-errors-and-exceptions)

---

### Redis Commands and Transactions

Redis provides a rich set of commands for working with data, including commands for manipulating strings, hashes, lists, sets, and sorted sets, as well as more advanced commands for working with data structures such as bitmaps and hyperloglogs. Redis also provides transactions for executing multiple commands atomically, which is useful for ensuring data consistency and integrity.

---

### Redis Transactions in Python

In Redis, a transaction is a group of commands sent between `MULTI` and `EXEC`. Redis runs them all in one go, and no other client's commands can sneak in between them.

In Python, you use a `pipeline`, which wraps its commands in a transaction by default:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)

with redis_client.pipeline() as pipe:
    pipe.set('mykey1', 'myvalue1')
    pipe.set('mykey2', 'myvalue2')
    pipe.incr('visits')
    results = pipe.execute()

print(results)   # [True, True, 1]
```

- Commands added to `pipe` aren't sent straight away. They're queued up
- `execute()` sends them all at once, inside `MULTI` and `EXEC`, and returns a list with one result per command
- Sending commands together is also much faster than sending them one at a time, because there's only one round trip to the server

> ## Atomic, but no rollback
>
> A Redis transaction makes sure the commands run together, but it doesn't undo earlier commands if a later one fails. That's different from transactions in SQL databases.

---

### Handling Errors and Exceptions

When working with Redis and Python, it's important to handle errors and exceptions properly. Redis and the Redis Python library provide several types of exceptions for handling errors, including redis.exceptions.RedisError for general Redis errors, redis.exceptions.ConnectionError for connection errors, and redis.exceptions.ResponseError for errors related to Redis responses.

Here's an example of how to handle a Redis error in Python:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)

try:
    redis_client.ping()
    print("Connected!")
except redis.exceptions.ConnectionError:
    print("Can't reach Redis. Is the server running?")
except redis.exceptions.RedisError as e:
    print('Error:', e)
```

In this example, we use a try/except block to catch Redis errors. `ConnectionError` is caught first, so we can give a helpful message when the server isn't running. Any other Redis error is caught by the more general `RedisError`.

---

{% quiz %}
? What does `pipe.execute()` return?
- [ ] Nothing
- [x] A list with one result for each queued command
- [ ] Only the result of the last command
> Each command in the pipeline gets its own entry in the results list.

? If the second command in a Redis transaction fails, what happens to the first?
- [ ] It's rolled back
- [x] It stays done; Redis transactions don't roll back
> Redis runs the commands together but doesn't undo them.
{% endquiz %}

---

### Key Takeaways

After completing this lesson, you should be able to:

* Describe the main Redis commands and transactions and their use cases
* Use Redis transactions in Python
* Handle Redis errors and exceptions in Python

---

### Conclusion

In this lesson, we covered Redis commands and transactions, and how to use them in Python. We described the main Redis commands and transactions, and demonstrated how to use transactions in Python using the Redis Python library. We also covered how to handle Redis errors and exceptions in Python. In the next lesson, we will look at Redis lists in more depth.

---
