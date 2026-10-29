---
layout: lesson
title: Redis Hashes
type: page
description: Learn about Redis hashes and how to use them with Python.
---

<!-- ![Cover photo of Redis hashes](assets/redis-hashes.jpg){:class="cover"} -->

### Overview

In this lesson, we will cover Redis hashes and how to use them with Python. Redis hashes are a data type that can be used to store key-value pairs. We will cover how to add, retrieve, and manipulate hash elements in Redis, and how to use hashes in Python.

---

## Lesson Content

In this lesson, we will cover:

* [Redis hashes](#redis-hashes)
* [Adding and retrieving hash elements](#adding-and-retrieving-hash-elements)
* [Manipulating hash elements](#manipulating-hash-elements)
* [Using Redis hashes with Python](#using-redis-hashes-with-python)

---

### Redis Hashes

Redis hashes are a data type that can be used to store key-value pairs. Hashes in Redis are implemented as a hash table, which means that adding, removing, and retrieving hash elements is very fast.

---

### Adding and Retrieving Hash Elements

To add a key-value pair to a hash in Redis, you can use the `hset` command. To retrieve the value of a key in a hash, you can use the `hget` command. Here are some examples of adding and retrieving elements from a Redis hash:

```redis
hset myhash key1 "value1"
(integer) 1
hset myhash key2 "value2"
(integer) 1
hget myhash key1
"value1"
```

---

### Manipulating Hash Elements

In addition to adding and retrieving hash elements, Redis provides several commands for manipulating hash elements. Here are some examples of manipulating hash elements in Redis:

* `hdel`: Delete one or more keys from a hash
* `hexists`: Check if a key exists in a hash
* `hkeys`: Get all keys in a hash
* `hvals`: Get all values in a hash
* `hgetall`: Get every key and value
* `hincrby`: Add a number to a value, handy for counters

---

### Using Redis Hashes with Python

To use Redis hashes in Python, you can use the hash methods of the Redis Python library, which all start with `h`. Here's an example of how to use Redis hashes in Python:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)

# Add key-value pairs to a hash, one at a time or several at once
redis_client.hset('myhash', 'key1', 'value1')
redis_client.hset('myhash', mapping={'key2': 'value2', 'key3': 'value3'})

# Get the value of a key in a hash
value = redis_client.hget('myhash', 'key1')

# Delete one or more keys from a hash
redis_client.hdel('myhash', 'key2')

# Get all keys in a hash
keys = redis_client.hkeys('myhash')

# Get all values in a hash
values = redis_client.hvals('myhash')
```

In this example, we use the Redis Python library to add key-value pairs to a hash using the `hset` method, retrieve the value of a key in a hash using the `hget` method, delete one or more keys from a hash using the `hdel` method, get all keys in a hash using the `hkeys` method, and get all values in a hash using the `hvals` method.

---

{% quiz %}
? Which method returns a whole hash as a Python dictionary?
- [ ] `hget()`
- [x] `hgetall()`
- [ ] `hkeys()`
> `hgetall()` returns every field and value. `hkeys()` only returns the field names.
{% endquiz %}

---

### Key Takeaways

After completing this lesson, you should be able to:

* Describe Redis hashes and their implementation in Redis
* Add and retrieve key-value pairs in Redis hashes
* Manipulate key-value pairs in Redis hashes
* Use Redis hashes in Python

---

### Conclusion

In this lesson, we covered Redis hashes and how to use them with Python. We described Redis hashes and their implementation in Redis, and demonstrated how to add, retrieve, and manipulate key-value pairs in Redis hashes, and how to use Redis hashes in Python using the Redis Python library. In the next lesson, we will cover Redis sorted sets.

---
