import time

secs = time.time()
tt = time.localtime(secs)
t = time.asctime(tt)

file_name = 'bhanumati ' + str(t) + '.csv'

x = 1.0
y = 2.0
z = 1.5

p = time.time()
q = 9.0
r = 999.123456789

a = str(x) + ',' + str(y) + ',' + str(z) + '\n'
b = str(p) + ',' + str(q) + ',' + str(r) + '\n'

file = open(file_name,'a')
file.write(a)
file.write(b)
