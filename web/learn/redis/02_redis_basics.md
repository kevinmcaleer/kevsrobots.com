---
layout: lesson
title: Redis Basics
author: Kevin McAleer
type: page
previous: 01_introduction.html
next: 03_data_types.html
description: Learn the basics of Redis and how to use it with Python.
percent: 20
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

To install Redis, you can download it from the Redis website and follow the installation instructions for your operating system. Once Redis is installed, you can start the Redis server using the `redis-server` command. By default, Redis listens on port 6379, but you can change this in the Redis configuration file.

---

### Using Redis with Python

To use Redis with Python, you can use the Redis Python library, which provides a Pythonic interface to Redis. Here's an example of how to use Redis with Python:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379)

# Set a string value
redis_client.set('mykey', 'myvalue')

# Get a string value
value = redis_client.get('mykey')

# Delete a key
redis_client.delete('mykey')
```

In this example, we create a Redis client using the redis.Redis method, which connects to the Redis server on localhost at port 6379. We then use the set method to set a string value for the key mykey, and the get method to retrieve the value. We also use the delete method to delete the key and its value.

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
