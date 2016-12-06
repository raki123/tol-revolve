import numpy.random


for f in range(20):
	ints = numpy.random.randint(1,5,25)
	count = [0,0,0,0]
	for i in ints:
		count[i-1]+=1
	print(count)
