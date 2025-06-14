import pickle
import numpy as np
import utils


def compute_delta_incremental(tick, next_tick):
    if np.abs(next_tick-tick) < 2**31:
            return next_tick-tick
    elif next_tick-tick < 2**31:
            return next_tick+ 2**32 - tick
    else:
        return -(tick + 2**32 - next_tick)
    
def compute_steering(tick, next_tick):
    delta = next_tick - tick
    if delta > 8192 / 2:
        delta -= 8192
    elif delta < -8192 / 2:
        delta += 8192
    return tick + delta

def compute_relative_isometries(subsampled_poses):
    iso_list = [utils.v2t(pose) for pose in subsampled_poses]
    relative_iso = [np.dot(utils.invert_isometry(iso_list[i]), iso_list[i+1]) for i in range(len(iso_list)-1)]
    return relative_iso

def subsample_dataset(lines):
    ticks = []
    poses = []
    for l in lines[9:]:
        tokens = l.split(":")
        tick = tokens[2].strip()
        tick = tick.split(" ")
        tick = [float(tick[0]), float(tick[1])]
        pose = tokens[4].strip().split(" ")
        pose = [float(pose[0]), float(pose[1]), float(pose[2])]
        ticks.append(tick)
        poses.append(pose)
    incremental = 0
    subsampled_ticks = [ticks[0]]
    subsampled_poses = [poses[0]]
    incremental_ticks = []
    for i in range(len(ticks)-1):
        incremental += compute_delta_incremental(ticks[i][1], ticks[i+1][1])
        if incremental > 100:
            subsampled_ticks.append(ticks[i])
            subsampled_poses.append(poses[i])
            incremental_ticks.append(incremental)
            incremental = 0
    return subsampled_ticks, subsampled_poses, incremental_ticks

def unwrap_steering(subsampled_ticks):
    unwrapped_steering = [subsampled_ticks[0][0]]
    for i in range(1, len(subsampled_ticks)):
        unwrapped_steering.append(compute_steering(unwrapped_steering[-1], subsampled_ticks[i][0]))
    return unwrapped_steering
    


if __name__=='__main__':

    with open('dataset.txt', 'r') as f:
        lines = f.read().splitlines()
    #subsample ticks by selecting transitions in incremental encoders larger than 100 ticks
    subsampled_ticks, subsampled_poses, incremental_ticks = subsample_dataset(lines)
    #unwrap absolute encoder values and take average values for consecutive terms
    unwrapped_steering = unwrap_steering(subsampled_ticks)
    steering_avgs = [np.mean([unwrapped_steering[i], unwrapped_steering[i+1]]) for i in range(len(unwrapped_steering)-1)]

    with open("dataset/unwrapped.txt", "w") as f:
        for elemento1 in unwrapped_steering:
            f.write(f"{elemento1} \n")


    #Now, take subsampled poses and compute relative changes by computing relative isometry and then back to vector representation
    relative_iso_matrices = compute_relative_isometries(subsampled_poses)
    relative_iso_vectors = [utils.t2v(pose) for pose in relative_iso_matrices]



    assert len(incremental_ticks) == len(steering_avgs) == len(relative_iso_vectors)

    with open("dataset/ticks_subsampled.txt", "w") as f:
        for el1, el2, el3 in zip(steering_avgs, incremental_ticks, relative_iso_vectors):
            line = " ".join(map(str, [el1, el2] + list(el3)))
            f.write(line + "\n")




