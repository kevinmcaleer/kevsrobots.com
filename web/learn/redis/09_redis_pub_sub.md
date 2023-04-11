---
layout: lesson
title: Redis Pub/Sub
author: Kevin McAleer
type: page
previous: 08_redis_sorted_sets.html
next: 10_summary.html
description: Learn about Redis pub/sub and how to use it with Python.
percent: 90
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


<!-- ![Cover photo of Redis pub/sub](assets/redis-pubsub.jpg){:class="cover"} -->

### Overview

In this lesson, we will cover Redis pub/sub and how to use it with Python. Redis pub/sub is a messaging system that allows for publish/subscribe messaging between clients. We will cover how to publish and subscribe to channels in Redis, and how to use pub/sub in Python.

---

### Lesson Content

In this lesson, we will cover:

* [Redis pub/sub](#redis-pubsub)
* [Publishing messages to channels](#publishing-messages-to-channels)
* [Subscribing to channels](#subscribing-to-channels)
* [Using Redis pub/sub with Python](#using-redis-pubsub-with-python)

---

### Redis Pub/Sub

Redis pub/sub is a messaging system that allows for publish/subscribe messaging between clients. In pub/sub, there are publishers and subscribers. Publishers send messages to channels, and subscribers receive messages from channels.

---

### Publishing Messages to Channels

To publish a message to a channel in Redis, you can use the `publish` command. Here's an example of how to publish a message to a channel:

```redis
publish mychannel "Hello, world!"
(integer) 1
```

---

### Subscribing to Channels

To subscribe to a channel in Redis, you can use the `subscribe` command. Here's an example of how to subscribe to a channel:

```bash
subscribe mychannel
Reading messages... (press Ctrl-C to quit)

1. "subscribe"
2. "mychannel"
3. (integer) 1
```

Once you have subscribed to a channel, you will receive any messages that are published to that channel.

---

### Using Redis Pub/Sub with Python

To use Redis pub/sub in Python, you can use the `pubsub` methods of the Redis Python library. Here's an example of how to use Redis pub/sub in Python:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379)

# Publish a message to a channel
redis_client.publish('mychannel', 'Hello, world!')

# Subscribe to a channel
pubsub = redis_client.pubsub()
pubsub.subscribe('mychannel')

# Receive messages from the subscribed channel
for message in pubsub.listen():
    print(message)
```

In this example, we use the Redis Python library to publish a message to a channel using the publish method, subscribe to a channel using the subscribe method, and receive messages from the subscribed channel using a for loop.

---

### Key Takeaways

After completing this lesson, you should be able to:

* Describe Redis pub/sub and its implementation in Redis
* Publish messages to channels in Redis
* Subscribe to channels in Redis and receive messages
* Use Redis pub/sub in Python

---

### Conclusion

In this lesson, we covered Redis pub/sub and how to use it with Python. We described Redis pub/sub and its implementation in Redis, and demonstrated how to publish messages to channels and subscribe to channels in Redis, and how to use Redis pub/sub in Python using the Redis Python library. In the final lesson, we will review what we have learned and provide some additional resources for further study.

---
