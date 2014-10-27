import os
import os.path
import matplotlib.pyplot as plt
import math

def get_foot_contact(footZ, height_threshold=20):
    return footZ[footZ < height_threshold].index

def get_left_support(dataframe):
    return dataframe[dataframe != 2].index

def get_right_support(dataframe):
    return dataframe[dataframe != 1].index

def compute_intervals(index_array):
    intervals = []
    last_start = index_array[0]
    last_value = index_array[0]
    for i in index_array:
        if i - last_value > 0.01:
            intervals.append((last_start, last_value))
            last_start = i
        last_value = i
    intervals.append((last_start, last_value))
    return intervals

def get_newest(log_dir, path):
    return get_nth_newest(log_dir, path, 1)

def get_nth_newest(log_dir, path, n):
    files = os.listdir(log_dir)
    matches = [f for f in files if f.endswith(path)]
    return os.path.join(log_dir, sorted(matches)[-n])

# Computes support foot intervals and adds them to a plot as background
class SupportPhaseInfo:
    def __init__(self, leftFootZ, rightFootZ, supportphase):
        self._left_contact  = compute_intervals(get_foot_contact(leftFootZ))
        self._right_contact = compute_intervals(get_foot_contact(rightFootZ))
        self._left_support  = compute_intervals(get_left_support(supportphase))
        self._right_support = compute_intervals(get_right_support(supportphase))

    def add_contact_phase_spans(self, colors=("red", "orange")):
        for start, end in self._left_contact:
            plt.axvspan(start, end, color=colors[0], alpha=0.1)
        for start, end in self._right_contact:
            plt.axvspan(start, end, color=colors[1], alpha=0.1)

    def add_support_phase_spans(self, colors=("green", "blue")):
        for start, end in self._left_support:
            plt.axvspan(start, end, color=colors[0], alpha=0.1)
        for start, end in self._right_support:
            plt.axvspan(start, end, color=colors[1], alpha=0.1)

class CollisionInfo:
    def __init__(self, contact_times, force):
        self._line_widths = [math.log(f+1) for f in force]
        self._contact_times = contact_times

    def add_collision_info_lines(self, color="red"):
        for i, (t, w) in enumerate(zip(self._contact_times, self._line_widths)):
            if i > 10:
                break
            plt.axvline(t, linewidth=w, color=color)
