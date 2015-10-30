# simulate the effects of resolved rates

def makeTrajectory(start, end, timesteps):
    for t in range(0, timesteps):
        yield start + (end - start) * ((t + 1) / timesteps)
