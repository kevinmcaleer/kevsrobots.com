# wrapper function

def hello(message):
    # this function wraps another
    
    def posh_hello(message):
        print(f"Well Hello")
    
    posh_hello(f"{message}")
        
hello("Hey Robot Makers")

posh_hello("this doesnt work; its not in scope")