import sys
import numpy

def mat_from_file(file_name):
    f = open(file_name,'r')
    lines = f.readlines()
    data = []
    for s in lines:
        parts = s.split()
        row = []
        for p in parts:
            row.append(float(p))
        data.append(row)
    return numpy.array(data)

if __name__ == '__main__':
    data = mat_from_file(sys.argv[1])
    print data
