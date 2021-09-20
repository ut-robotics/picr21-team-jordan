x = {1, 2, 3}
y = {4, 5, 6}
z = {7, 8, 9}


print(x | (y & z) == (x | y) & (x | z))
print(x | (y - z) == (x | y) - (x | z))
print(x | (y | z) == (x | y) | (x | z))
