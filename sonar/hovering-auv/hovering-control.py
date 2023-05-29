import holoocean
import matplotlib.pyplot as plt
import numpy as np
import json
from pynput import keyboard
import math

#### GET SONAR CONFIG
cfg = None
with open('scenario.json', 'r') as f:
    cfg = json.load(f)

env = holoocean.make(scenario_cfg=cfg)
config = cfg['agents'][0]['sensors'][-1]["configuration"]
azi = config['Azimuth']
minR = config['RangeMin']
maxR = config['RangeMax']
binsR = config['RangeBins']
binsA = config['AzimuthBins']

#### GET PLOT READY
plt.ion()
fig, ax = plt.subplots(subplot_kw=dict(projection='polar'), figsize=(8,5))
ax.set_theta_zero_location("N")
ax.set_thetamin(-azi/2)
ax.set_thetamax(azi/2)

theta = np.linspace(-azi/2, azi/2, binsA)*np.pi/180
r = np.linspace(minR, maxR, binsR)
T, R = np.meshgrid(theta, r)
z = np.zeros_like(T)

plt.grid(False)
plot = ax.pcolormesh(T, R, z, cmap='gray', shading='auto', vmin=0, vmax=1)
plt.tight_layout()
fig.canvas.flush_events()

pressed_keys = list()
force = 25

def on_press(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.append(key.char)
        pressed_keys = list(set(pressed_keys))

def on_release(key):
    global pressed_keys
    if hasattr(key, 'char'):
        pressed_keys.remove(key.char)

listener = keyboard.Listener(
    on_press=on_press,
    on_release=on_release)
listener.start()

def parse_keys(keys, val):
    command = np.zeros(8)
    if 'i' in keys:
        command[0:4] += val
    if 'k' in keys:
        command[0:4] -= val
    if 'j' in keys:
        command[[4,7]] += val
        command[[5,6]] -= val
    if 'l' in keys:
        command[[4,7]] -= val
        command[[5,6]] += val

    if 'u' in keys:
        command[[0,1]] += val
        command[[2,3]] -= val
    if 'o' in keys:
        command[[0,1]] -= val
        command[[2,3]] += val

    if 'w' in keys:
        command[4:8] += val
    if 's' in keys:
        command[4:8] -= val
    if 'a' in keys:
        command[[4,6]] += val
        command[[5,7]] -= val
    if 'd' in keys:
        command[[4,6]] -= val
        command[[5,7]] += val

    return command


#### RUN SIMULATION
while True:
    if 'q' in pressed_keys:
        break

    command = parse_keys(pressed_keys, force)

    print(pressed_keys)

    env.act("auv0", command)
    state = env.tick()

    if 'ProfilingSonar' in state:
        s = state['ProfilingSonar']
        plot.set_array(s.ravel())

        fig.canvas.draw()
        fig.canvas.flush_events()

print("Finished Simulation!")
plt.ioff()
plt.show() 