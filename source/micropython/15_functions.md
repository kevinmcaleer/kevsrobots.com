---
title: Functions
description: Learn how to make reusable blocks of code
layout: lesson
---

![Photo of a laptop and notepad on a desk](assets//functions.jpg){:.cover}

At some point you will want to make a block of code that you will use many times, perhaps with different values. This is not only essential to save repeating the same block of code over and over again, its a better technique to improve reliability and readability.

## Defining a function

In MicroPython we use the `def` keyword to define the code that follows as a function. 

Functions can also have parameters of there own.

Lets create a simple function that will print a message when its called:

##### Example

```python
def say_hello():
    # This function prints the message hello
    print('hello')


say_hello()
```

The example above defines (thats why its the `def` keyword) the function called `say_hello`, notice that we also need to provide the `()` brackets as functions need this, we also need to use the `:` colon symbol to tell MicroPython that the following block of code relates to this function.

The function code is also indented.

---

## Defining a function with Parameters

We often want our function to take in an input variable so that we can use it in the function code. Lets extend our `say_hello` example so that it can include a name and age.

##### Example

```python
def say_hello(name, age):
    # This function prints the message hello
    print('hello', name, 'how does',age,'feel?')


say_hello('Kev', 47)
```

In the example above we've added two parameters `name` and `age`. Our functions can use these parameters within its block of code, in this case we use it to print out the message `hello Kev how does 47 feel?`.
Notice how Python accepts the two different types of variables - the `name` is a string (`Str`) of text, and `age` is an integer (a whole number with no decimal points).

---

We can take this a step further and pass variables to the function and it will work just the same:

##### Example

```python
def say_hello(name, age):
    # This function prints the message hello
    print('hello', name, 'how does',age,'feel?')

my_name = 'Kev'
my_age = 47

say_hello(my_name, my_age)
```

Functions can also call other functions, though on a MicroController you may find that the device crashes if it runs out of memory, if you nest too many functions, but that will take quite a few levels.

---

## Returning values

Functions can do more than just run blocks of code, they can return values too. Lets make a new function that takes a value and adds `1` to it.

##### Example

```python
def add_one(number):
    number += 1
    return number

a = add_one(1)
print(a)
```

In the example above we define the function `add_one` which has one parameter `number`. It then adds `1` to the number and `return`s the number to the caller.

We assign `a` to the result of running `add_one(1)`, which should return the value `2`. Therefore the variable `a` should contain the value `2`. Run the code and try passing different values and changing the `number += 1` to different values.

---

> ## Challenge
>
> Write your own function that takes a value, multiplies it by `10`, and then returns that value.
>
{:.bg-blue}

---
