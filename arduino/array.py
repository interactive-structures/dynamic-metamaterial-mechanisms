fileName = 'active.csv'

def readFile(path):
    with open(path, 'rt') as f:
        return f.read()

def createArray(file):
    res = []
    i = 0
    for line in readFile(fileName).splitlines():
        if i != 0:
            temp = []
            for val in line.split(','):
                res.append(float(val))
        i += 1
    return res

print(createArray(fileName))