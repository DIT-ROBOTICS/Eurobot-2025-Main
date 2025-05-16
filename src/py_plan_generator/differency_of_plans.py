import math

pts = []
distance = []
num = int(input('input num: '))
print('input pts: ')
for i in range(0, num):
    a = map(float, input().split(","))
    pts.append(list(a))
print(pts)

for pt0 in pts:
    dist_row = []
    for pt1 in pts:
        dist_row.append(math.dist(pt0, pt1))
    distance.append(dist_row)
print(distance)

value = 0
plan0 = []
plan1 = []
plan0 = list(map(int, input().split()))
plan1 = list(map(int, input().split()))

for i in range(0, min(len(plan0), len(plan1))):
    value += distance[plan0[i]][plan1[i]]
print(value)