---
layout: lesson
title: Redis Lists
author: Kevin McAleer
type: page
previous: 04_commands_and_transactions.html
next: 06_redis_sets.html
description: Learn about Redis lists and how to use them with Python.
percent: 50
duration: 4
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


<!-- ![Cover photo of Redis lists](assets/redis-lists.jpg){:class="cover"} -->

### Overview

In this lesson, we will cover Redis lists and how to use them with Python. Redis lists are a data type that can be used to store an ordered collection of strings. We will cover how to add, retrieve, and manipulate list elements in Redis, and how to use lists in Python.

---

## Lesson Content

In this lesson, we will cover:

* [Redis lists](#redis-lists)
* [Adding and retrieving list elements](#adding-and-retrieving-list-elements)
* [Manipulating list elements](#manipulating-list-elements)
* [Using Redis lists with Python](#using-redis-lists-with-python)

---

### Redis Lists

Redis lists are a data type that can be used to store an ordered collection of strings. Lists in Redis are implemented as linked lists, which means that adding or removing elements from the beginning or end of a list is very fast, but accessing elements in the middle of a list can be slow.

---

### Adding and Retrieving List Elements

To add an element to the end of a list in Redis, you can use the `rpush` command. To add an element to the beginning of a list, you can use the `lpush` command. Here are some examples of adding elements to a Redis list:

```redis
rpush mylist "element1"
(integer) 1
rpush mylist "element2"
(integer) 2
lpush mylist "element0"
(integer) 3
```

To retrieve elements from a list, you can use the `lrange` command. The `lrange` command returns a range of elements from the list, starting at the specified index and ending at the specified index. Here's an example of retrieving elements from a Redis list:

```redis
lrange mylist 0 -1

"element0"
"element1"
"element2"
```

---

### Manipulating List Elements

In addition to adding and retrieving list elements, Redis provides several commands for manipulating list elements. Here are some examples of manipulating list elements in Redis:

* `lpop`: Remove and return the first element of a list
* `rpop`: Remove and return the last element of a list
* `ltrim`: Trim a list to the specified range of elements

---

### Using Redis Lists with Python

To use Redis lists in Python, you can use the `list` methods of the Redis Python library. Here's an example of how to use Redis lists in Python:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379)

# Add elements to the end of a list
redis_client.rpush('mylist', 'element1', 'element2')

# Add elements to the beginning of a list
redis_client.lpush('mylist', 'element0')

# Get a range of elements from a list
elements = redis_client.lrange('mylist', 0, -1)

# Remove and return the first element of a list
first_element = redis_client.lpop('mylist')

# Remove and return the last element of a list
last_element = redis_client.rpop('mylist')
```

In this example, we use the Redis Python library to add elements to a list using the rpush and lpush methods, retrieve a range of elements from the list using the lrange method, and remove and return elements from the list using the lpop and rpop methods.

---

### Key Takeaways

After completing this lesson, you should be able to:

* Describe Redis lists and their implementation in Redis
* Add and retrieve elements from Redis lists
* Manipulate elements in Redis lists
* Use Redis lists in Python

---

### Conclusion

In this lesson, we covered Redis lists and how to use them with Python. We described Redis lists and their implementation in Redis, and demonstrated how to add, retrieve, and manipulate list elements in Redis, and how to use Redis lists in Python using the Redis Python library. In the next lesson, we will cover Redis sets.

---
