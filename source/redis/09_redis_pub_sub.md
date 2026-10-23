---
layout: lesson
title: Redis Pub/Sub
type: page
description: Learn about Redis pub/sub and how to use it with Python.
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

Pub/sub needs two programs: one listening and one sending. Messages aren't stored, so the subscriber must be running *before* the message is published, or it will miss it.

Save this as `subscriber.py` and run it first:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)

pubsub = redis_client.pubsub()
pubsub.subscribe('mychannel')

print("Listening...")
for message in pubsub.listen():
    if message['type'] == 'message':
        print("Received:", message['data'])
```

Then, in a second terminal, run `publisher.py`:

```python
import redis

redis_client = redis.Redis(host='localhost', port=6379, decode_responses=True)

listeners = redis_client.publish('mychannel', 'Hello, world!')
print("Delivered to", listeners, "subscriber(s)")
```

- `pubsub.subscribe()` starts listening on a channel
- `pubsub.listen()` waits for messages and hands each one to the loop as a dictionary
- The first message you get is a confirmation with `type` set to `'subscribe'`, which is why we only print messages whose `type` is `'message'`
- `publish()` returns how many subscribers received the message

> ## A robot example
>
> Pub/sub is a neat way to connect parts of a robot project. A sensor script can publish readings on a `sensors` channel, while a dashboard and a logger each subscribe to it, without knowing about each other.

---

{% quiz %}
? A message is published before any subscriber is listening. What happens to it?
- [ ] It's stored until someone subscribes
- [x] It's lost
- [ ] Redis returns an error
> Pub/sub doesn't store messages. Use a list as a queue if messages must not be missed.
{% endquiz %}

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
