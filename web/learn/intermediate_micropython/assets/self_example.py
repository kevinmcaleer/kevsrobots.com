# Self Example

# Code without using class properties

# left_foot_toes = 3
# right_foot_toes = 4
# left_foot_smelly = True
# right_foot_smelly = False
# left_foot_big_toe = True
# right_foot_big_toe = False

# You could use a list to store all the variables for each foot

# left_foot = [2, True, False]

# but this would get complicated if you later remove or change the order of variables

# You could use a list, but this is inefficient as the keys would be stored with each value
# doubling the space needed

class Foot():
    # models a foot
    
    toes = 4
    big_toe = True
    smelly = False
    
    def total_toes(self)->int:
        # Returns the number of toes plus big toe if present
        return self.toes + self.big_toe
    
    def has_big_toe(self)->bool:
        # Returns the presence of absense of a big toe
        return self.big_toe

left_foot = Foot()
right_foot = Foot()

my_toes = left_foot.total_toes() + right_foot.total_toes()

print(f"You have {my_toes} toes.")