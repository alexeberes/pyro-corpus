import os
from simulation import Simulation

def simulate():
    simulation = Simulation("GUI", 0)
    simulation.run()

if __name__ == '__main__':
    simulate()