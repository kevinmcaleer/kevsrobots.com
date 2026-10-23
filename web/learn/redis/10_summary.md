---
layout: lesson
title: Conclusion and Additional Resources
author: Kevin McAleer
type: page
cover: /learn/redis/assets/redis-cover.jpg
date: 2023-04-12
previous: 09_redis_pub_sub.html
description: Review what we have learned and find additional resources for further
  study.
percent: 100
duration: 3
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


<!-- ![Cover photo of Redis and Python](assets/redis-python.jpg){:class="cover"} -->

### Overview

In this final lesson, we will review what we have learned in this course and provide some additional resources for further study.

---

### Course Content

In this course, we covered:

* [Redis basics](02_redis_basics.html#redis-basics)
* [Strings and key-value pairs](03_data_types.html#string-data-type)
* [Lists](05_redis_lists.html#adding-and-retrieving-list-elements)
* [Sets](06_redis_sets.html#adding-and-retrieving-set-elements)
* [Hashes](07_redis_hashes.html#adding-and-retrieving-hash-elements)
* [Sorted sets](08_redis_sorted_sets.html#adding-and-retrieving-sorted-set-elements)
* [Pub/sub messaging](09_redis_pub_sub.html#publishing-messages-to-channels)
* [Transactions and pipelines](04_commands_and_transactions.html#redis-transactions-in-python)
* [Using Redis with Python](02_redis_basics.html#using-redis-with-python)

---

### Review

Redis is a powerful and versatile in-memory data store that can be used for a wide range of applications, from caching and session storage to message queuing and real-time analytics. In this course, we covered the basics of Redis and how to use it with Python.

We started with simple key-value pairs and strings, and then moved on to more advanced data types like lists, sets, hashes, and sorted sets. We also covered pub/sub messaging and how to use Redis with Python.

By the end of this course, you should have a solid understanding of the basics of Redis and how to use it with Python. You should be able to use Redis to solve a wide range of data storage and messaging problems in your own applications.

---

### Additional Resources

If you want to learn more about Redis and how to use it with Python, here are some additional resources to check out:

* Redis documentation: <https://redis.io/docs/latest/>
* Redis command reference: <https://redis.io/docs/latest/commands/>
* Redis Python library documentation: <https://redis-py.readthedocs.io/en/stable/>
* Redis University (free courses): <https://university.redis.io/>
* Valkey, the open source fork: <https://valkey.io>

---

{% quiz %}
? You want to cache a web page for five minutes. Which call fits best?
- [x] `set('page', html, ex=300)`
- [ ] `lpush('page', html)`
- [ ] `publish('page', html)`
> `ex=300` makes the key expire after 300 seconds, so stale pages clear themselves.

? Which Redis feature would you use to send live sensor readings to several programs at once?
- [ ] Sorted sets
- [ ] Transactions
- [x] Pub/sub
> Every subscriber to a channel receives each published message.
{% endquiz %}

---

### What's next?

* [Create Databases with Python and SQLite3](/learn/sqlite3/) - for data that needs to be kept on disk and queried with SQL
* [Docker](/learn/docker/) - run Redis alongside your own apps in containers

---

### Conclusion

In this course, we covered Redis and how to use it with Python. We covered a range of topics, from simple key-value pairs to more advanced data types like lists, sets, hashes, and sorted sets, as well as pub/sub messaging. By the end of this course, you should have a solid understanding of Redis and how to use it with Python.

Thank you for taking this course, and we hope you found it useful and informative!

---
