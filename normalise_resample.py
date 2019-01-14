import random

# Normalisation
particle_set = []
particle_weights = []
num_particles = 100

# Add up all the weights in the unnormalised set, and then divide the weight of each by this total
sum_set = sum(particle_weights)
normalised_weights = [i / sum_set for i in particle_weights]

cum_weight_array = []
for i, w in enumerate(normalised_weights):
  if i == 0:
    cum_weight_array[i] = normalised_weights[i]
  else:
    cum_weight_array[i] = normalised_weights[i] + normalised_weights[i-1]

new_particle_set = []
for i in range(num_particles):
  new_particle_set[i] = particle_set[choose_particle()]

particle_set = new_particle_set[:]

def choose_particle():
  # Get random number between 0 and 1
  rand_num = random.uniform(0, 1)
  for i, val in enumerate(cum_weight_array):
    if rand_num < val :
        return i
    
