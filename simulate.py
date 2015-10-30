# simulate the effects of resolved rates

# TODO: make this optimize the path or whatever
def makeTrajectory(start, end, timesteps):
    for t in range(0, timesteps):
        yield start + (end - start) * ((t + 1) / timesteps)
