---
layout: lesson
title: Redis Basics
author: Kevin McAleer
type: page
cover: /learn/redis/assets/redis-cover.jpg
date: 2023-04-12
previous: 01_introduction.html
next: 03_data_types.html
description: Learn the basics of Redis and how to use it with Python.
percent: 20
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


<!-- ![Cover photo of Redis basics](assets/redis-basics.jpg){:class="cover"} -->

### Overview

In this lesson, we will cover the basics of `Redis` and how to use it with Python. Redis is an in-memory key-value store that can be used for caching, real-time analytics, message queues, and more. We will cover how to install and configure Redis, and how to use it with Python.

---

### Lesson Content

In this lesson, we will cover:

* [Redis basics](#redis-basics)
* [Installing and configuring Redis](#installing-and-configuring-redis)
* [Using Redis with Python](#using-redis-with-python)

---

### Redis Basics

Redis is an in-memory key-value store that can be used for caching, real-time analytics, message queues, and more. Redis stores data in memory, which makes it very fast, but also means that it is limited by the amount of available memory.

Redis data is organized into key-value pairs, where the key is a string and the value can be any Redis data type, such as a string, hash, list, set, or sorted set. Redis provides a rich set of commands for working with data, including commands for manipulating strings, hashes, lists, sets, and sorted sets, as well as more advanced commands for working with data structures such as bitmaps and hyperloglogs.

---

### Installing and Configuring Redis

Pick whichever option suits your computer:

**Raspberry Pi OS, Debian or Ubuntu**

```bash
sudo apt update
sudo apt install redis-server
```

The server starts automatically and runs in the background.

**macOS** (with [Homebrew](https://brew.sh))

```bash
brew install redis
brew services start redis
```

**Any computer with Docker** (including Windows)

```bash
docker run -d --name redis -p 6379:6379 redis
```

See the [Docker course](/learn/docker/) if you're new to Docker.

To check Redis is running, use the `redis-cli` command-line tool (inside Docker, use `docker exec -it redis redis-cli`):

```bash
redis-cli ping
```

If you see `PONG`, you're ready. You can type any Redis command into `redis-cli` to try it out, which is handy for the command examples in later lessons.

By default, Redis listens on port `6379`, but you can change this in the Redis configuration file.

---

### Install the Python library

```bash
pip install redis
```

On Raspberry Pi OS, create and activate a virtual environment first (`python3 -m venv ~/redis-env` then `source ~/redis-env/bin/activate`).

---

### Using Redis with Python

To use Redis with Python, you can use the Redis Python library, which provides a Pythonic interface to Redis. Here's an example of how to use Redis with Python:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)

# Set a string value
redis_client.set('mykey', 'myvalue')

# Get a string value
value = redis_client.get('mykey')
print(value)   # myvalue

# Delete a key
redis_client.delete('mykey')
```

In this example, we create a Redis client with `redis.Redis()`, which connects to the Redis server on localhost at port 6379. We then use `set()` to store a string value for the key `mykey`, and `get()` to retrieve it. Finally, `delete()` removes the key and its value.

> ## Bytes or strings?
>
> Redis stores everything as bytes. Without `decode_responses=True`, `get()` returns `b'myvalue'` instead of `'myvalue'`. We use `decode_responses=True` throughout this course so you get normal Python strings back.

---

### Keys that expire

One of Redis's handiest tricks is that keys can delete themselves after a set time. That's perfect for caching:

```python
redis_client.set('weather', 'sunny', ex=60)   # expires in 60 seconds
print(redis_client.ttl('weather'))            # seconds left to live
```

---

{% quiz %}
? Which port does Redis listen on by default?
- [ ] 80
- [ ] 5432
- [x] 6379
> 6379 is Redis's default port. You can change it in the configuration file.

? Why do we pass `decode_responses=True` when connecting?
- [ ] It makes Redis faster
- [x] So results come back as Python strings instead of bytes
- [ ] It's required to connect to localhost
> Redis returns bytes by default, like `b'myvalue'`.
{% endquiz %}

---

### Key Takeaways

After completing this lesson, you should be able to:

* Describe the basics of Redis and its data model
* Install and configure Redis
* Use Redis with Python

---

### Conclusion

In this lesson, we covered the basics of Redis and how to use it with Python. We described Redis and its data model, and demonstrated how to install and configure Redis, and how to use Redis with Python using the Redis Python library. In the next lesson, we will cover Redis data types.

---
