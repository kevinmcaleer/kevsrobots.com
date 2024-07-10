class Animal:
    def __init__(self, name):
        self.name = name

    def speak(self):
        print("Animal speaking")

class Dog(Animal):
    def __init__(self, name, breed):
        super().__init__(name)
        self.breed = breed

    def speak(self):
        print("Dog barking")

animal = Animal("Animal")
dog = Dog("Dog", "Labrador")
dog.speak()