fileName = 'multipath.csv'

def readFile(path):
    with open(path, 'rt') as f:
        return f.read()

def createArray(file):
    res = ''
    i = 0
    for line in readFile(fileName).splitlines():
        temp = '{'
        if i != 0:
            j = 0
            for item in line.split(','):
                temp += item
                if j != 1:
                    temp += ','
                j += 1
            temp += '},\n'
        res += temp
        if i == 50:
            res += '\n\n\n'
        i += 1
    return res + '}'

print(createArray(fileName))