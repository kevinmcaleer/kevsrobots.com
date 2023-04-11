---
layout: lesson
title: Redis Data Types
type: page
description: Learn about Redis data types and how to use them in Python.
---

<!-- ![Cover photo of Redis data types](assets/redis-data-types.jpg){:class="cover"} -->

### Overview

In this lesson, we will cover Redis data types and how to use them in Python. Redis supports several data types, including strings, hashes, lists, sets, and sorted sets. We will cover how to create and manipulate each data type in Redis, and how to interact with them using the Redis Python library.

---

### Lesson Content

In this lesson, we will cover:

* [Redis data types](#redis-data-types)
* [String data type](#string-data-type)
* [Hash data type](#hash-data-type)
* [List data type](#list-data-type)
* [Set data type](#set-data-type)
* [Sorted set data type](#sorted-set-data-type)
* [Using Redis data types with Python](#sorted-set-data-type)

---

### Redis Data Types

Redis supports several data types, each with its own set of commands and operations. Here are the main data types in Redis:

* Strings: used to store text or binary data.
* Hashes: used to store key-value pairs, where the keys and values are both strings.
* Lists: used to store a collection of ordered elements.
* Sets: used to store a collection of unordered elements with no duplicates.
* Sorted sets: used to store a collection of ordered elements with scores, where each element has a unique score.

---

### String Data Type

The string data type is the most basic data type in Redis. It is used to store text or binary data. Here are some examples of string operations in Redis:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379)

# Set a string value
redis_client.set('mykey', 'myvalue')

# Get a string value
value = redis_client.get('mykey')

# Append a string to an existing value
redis_client.append('mykey', 'appended value')
```

---

### Hash Data Type

The hash data type is used to store key-value pairs, where the keys and values are both strings. Here are some examples of hash operations in Redis:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379)

# Set a hash value
redis_client.hset('myhash', 'key1', 'value1')

# Get a hash value by key
value = redis_client.hget('myhash', 'key1')

# Get all hash values
all_values = redis_client.hgetall('myhash')
```

---

### List Data Type

The list data type is used to store a collection of ordered elements. Here are some examples of list operations in Redis:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379)

# Add an element to a list
redis_client.rpush('mylist', 'element1')

# Get an element by index
element = redis_client.lindex('mylist', 0)

# Get all elements in a list
all_elements = redis_client.lrange('mylist', 0, -1)

```

---

### Set Data Type

The set data type is used to store a collection of unordered elements with no duplicates. Here are some examples of set operations in Redis:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379)

# Add an element to a set
redis_client.sadd('myset', 'element1')

# Check if an element is already in a set
element_exists = redis_client.sismember('myset', 'element1')

```

---

### Get all elements in a set

```python
all_elements = redis_client.smembers('myset')

```

---

### Sorted Set Data Type

The sorted set data type is used to store a collection of ordered elements with scores, where each element has a unique score. Here are some examples of sorted set operations in Redis:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379)

# Add an element to a sorted set
redis_client.zadd('mysortedset', {'element1': 1})

# Get an element by rank
element = redis_client.zrange('mysortedset', 0, 0)

# Get all elements in a sorted set with scores
all_elements = redis_client.zrange('mysortedset', 0, -1, withscores=True)

```

---

### Using Redis Data Types with Python

To use Redis data types in Python, you can use the corresponding methods of the Redis Python library. Here's an example of how to use a Redis hash in Python:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379)

# Set a hash value
redis_client.hset('myhash', 'key1', 'value1')

# Get a hash value by key
value = redis_client.hget('myhash', 'key1')

```

---

### Key Takeaways

After completing this lesson, you should be able to:

Describe the main Redis data types and their use cases
Perform basic operations on Redis data types using the Redis Python library

---

### Conclusion

In this lesson, we covered Redis data types and how to use them in Python. We described the main data types in Redis, including strings, hashes, lists, sets, and sorted sets, and demonstrated how to perform basic operations on each data type using the Redis Python library. In the next lesson, we will cover Redis commands and transactions.

---
