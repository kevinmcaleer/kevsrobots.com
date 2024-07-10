# Basic Class

class Robot():
    
    name = "Robby"
    
    def __init__(self, name=None):
        if name is not None:    
            self.name = name
        
    def __str__(self):
        return f"My name is {self.name}"
    

bot = Robot()
bot02 = Robot("Cassie")

print(bot)

print(bot02)