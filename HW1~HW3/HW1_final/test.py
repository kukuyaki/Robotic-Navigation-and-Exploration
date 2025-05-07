a=[1,5,1,6,3,8,2,8,25,6,36,1135,37,246,23]
import heapq
while a:
    print(a)
    print(heapq.heappop(a))