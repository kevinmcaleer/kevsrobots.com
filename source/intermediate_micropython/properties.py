class Person:
    def __init__(self, name, age):
        self._name = name
        self._age = age

    @property
    def name(self):
        """The name property."""
        return self._name

    @name.setter
    def name(self, value):
        if not isinstance(value, str):
            raise ValueError("Name must be a string")
        self._name = value

    @property
    def age(self):
        """The age property."""
        return self._age

    @age.setter
    def age(self, value):
        if not isinstance(value, int):
            raise ValueError("Age must be an integer")
        if value < 0:
            raise ValueError("Age cannot be negative")
        self._age = value

# Example usage
person = Person("John Doe", 30)

# Accessing the properties
print(person.name)  # Output: John Doe
print(person.age)   # Output: 30

# Modifying the properties using setters
person.name = "Jane Doe"
person.age = 25

print(person.name)  # Output: Jane Doe
print(person.age)   # Output: 25

# Attempting to set invalid values
try:
    person.name = 123  # Raises ValueError
except ValueError as e:
    print(e)  # Output: Name must be a string

try:
    person.age = -5  # Raises ValueError
except ValueError as e:
    print(e)  # Output: Age cannot be negative