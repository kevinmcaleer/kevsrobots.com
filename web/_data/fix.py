BROKEN_FILE = 'web/_data/fix.txt'

with open(BROKEN_FILE) as f:
    data = f.read()
    for line in data:

        replaced = line.replace('code', ' - code')
        