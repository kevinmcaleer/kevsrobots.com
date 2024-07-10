class Car():

    _fuel_type = "petrol"

    def __init__(self, fuel:str=None):
        if fuel is not None:
            self._fuel_type = fuel

    def __str__(self):
        return f"Fuel type is {self._fuel_type}"
    
gas_guzzler = Car()

zoe = Car(fuel="Electric")
print(zoe)

print(gas_guzzler)