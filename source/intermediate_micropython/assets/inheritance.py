# Inheritance

class Mammal():
    backbone = True
    live_birth = True
    
    def is_warmblooded(self):
        return True
    

class Dog(Mammal):
    barks = True
    
    def no_of_legs(self):
        return 4
    
    def has_legs(self):
        return True
    
class Dolphin(Mammal):
    barks = False
    
    def has_legs(self):
        return False
    
    def no_of_legs(self):
        return 0
    
flipper = Dolphin()
lassy = Dog()

print(f"flipper has {flipper.no_of_legs()} legs")
print(f"Lassy has {lassy.no_of_legs()} legs")
print(f"It's {flipper.live_birth} that dolhpins have live births.")
