import json

import matplotlib.pyplot as plt

with open("../build/tests/samples.json", "r") as file:
    samples = json.load(file)

print(samples)

figure, axes = plt.subplots(1,1)
axes.scatter([sample[0] for sample in samples], [sample[1] for sample in samples])
plt.show()