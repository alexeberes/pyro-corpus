import numpy as np


def mutate(network_weights, random_number_generator, network_shape, mutation_rate, mutation_magnitude) -> None:
    mutation_on_off = random_number_generator.uniform(size=network_shape) < mutation_rate
    mutation_magnitude_multiplier = np.multiply(random_number_generator.choice([-1, 1]), random_number_generator.exponential(scale=mutation_magnitude, size=network_shape))
    mutations = np.multiply(mutation_on_off, mutation_magnitude_multiplier)
    return network_weights.get_weights() + mutations