import holoocean
import numpy as np

if __name__ == "__main__":
    env = holoocean.make("PierHarbor-Hovering")

    # The hovering AUV takes a command for each thruster
    command = np.array([10,10,10,10,0,0,0,0])

    env.act("auv0", command)

    for _ in range(200):
        # state = env.step(command)
        state = env.tick()

        print(state.keys())