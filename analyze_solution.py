import pickle
from body_parts import *

import sys

genome_file = sys.argv[1]
print(genome_file)

with open(genome_file, "rb") as file:
    print(pickle.load(genome_file))