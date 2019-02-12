import time
import random

then = time.time() #Time before the operations start

#DO YOUR OPERATIONS HERE
i = 0
while (i < 10000000):
    h = random.uniform(0.0, 1300.0)
    i += 1

now = time.time() #Time after it finished

print("It took: ", now-then, " seconds")