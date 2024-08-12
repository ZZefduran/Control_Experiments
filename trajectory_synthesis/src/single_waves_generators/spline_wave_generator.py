import numpy as np
from typing import List
# import matplotlib.pyplot as plt

class State:
    def __init__(self, pos, vel, acc, jerk, dt):
        self.pos = pos
        self.vel = vel
        self.acc = acc
        self.jerk = jerk
        self.dt = dt

    def __repr__(self) -> str:
        return f"pos: {self.pos}, vel:{self.vel}, acc: {self.acc}, jerk: {self.jerk}"


class StateGen:
    def __init__(self, dof_limit):
        self.min_pos = dof_limit.min_pos
        self.max_pos = dof_limit.max_pos
        self.max_vel = dof_limit.max_vel
        self.max_acc = dof_limit.max_acc
        self.max_jerk = dof_limit.max_jerk
        self.peak_vel = dof_limit.peak_vel
        self.a_acc_clip = -self.max_acc/(self.max_vel - self.peak_vel)
        self.b_acc_clip = -self.a_acc_clip * self.max_vel
    
    def __repr__(self) -> str:
        return f"min pos: {self.min_pos}, max pos: {self.max_pos}, peak vel: {self.peak_vel}, max vel: {self.max_vel}, max acc: {self.max_acc}, max jerk: {self.max_jerk}"


    def validate_state(self, state: State):
        epsilon = 1.0e-6
        err = ""
        if self.max_pos is not None and state.pos - self.max_pos > epsilon:
            err += f"\nMax pos exceeded {state.pos} > {self.max_pos}"
        if self.min_pos is not None and state.pos - self.min_pos < epsilon:
            err += f"\nMin pos exceeded {state.pos} < {self.min_pos}"
        if state.vel - self.max_vel > epsilon:
            err += f"\nMax vel exceeded {state.vel} > {self.max_vel}"
        if state.vel + self.max_vel < epsilon:
            err += f"\nMin vel exceeded {state.vel} < -{self.max_vel}"
        if state.acc - self.max_acc > epsilon:
            err += f"\nMax acc exceeded {state.acc} > {self.max_acc}"
        if state.acc + self.max_acc < epsilon:
            err += f"\nMin acc exceeded {state.acc} < -{self.max_acc}"
        if abs(state.vel) < self.peak_vel:
            if state.jerk - self.max_jerk > epsilon:
                err += f"\nMax jerk exceeded {state.jerk} > {self.max_jerk}"
            if state.jerk + self.max_jerk < epsilon:
                err += f"\nMin jerk exceeded {state.jerk} < -{self.max_jerk}"
        if len(err) > 0:
            raise Exception(err)

    def get_valid_state(self, state: State, desired_pos) -> State:
        pos_min_limit, pos_max_limit = self.calculate_pos_limits(state)
        next_pos = np.clip(desired_pos, pos_min_limit, pos_max_limit)
        # print(f"next pos: {next_pos}")
        next_state = self.pos_to_state(state, next_pos)
        # print(f"next state: {next_state}")
        return next_state

    def pos_to_state(self, state: State, pos) -> State:
        vel = (pos - state.pos) / state.dt
        acc = (vel - state.vel) / state.dt
        jerk = (acc - state.acc) / state.dt
        return State(pos, vel, acc, jerk, state.dt)

    def calculate_pos_limits(self, state: State):
        # Pos limits
        if self.min_pos is not None and self.max_pos is not None:
            pos_interval = (self.min_pos, self.max_pos)
        else:
            pos_interval = None

        # Velocity limits
        vel_max = state.pos + self.max_vel * state.dt
        vel_min = state.pos - self.max_vel * state.dt
        vel_interval = (vel_min, vel_max)

        # Acc limits
        acc_min_limit, acc_max_limit = self.get_acc_limits(state.vel)
        # if acc_max_limit != self.max_acc:
        #     print(
        #         f"*** acc max limit has changed due to velocity: {acc_max_limit} at vel {state.vel} (min: {acc_min_limit})")
        # if acc_min_limit != -self.max_acc:
        #     print(
        #         f"*** acc min limit has changed due to velocity: {acc_min_limit} at vel {state.vel}, max: {acc_max_limit}")
        acc_max = state.pos + state.vel * state.dt + acc_max_limit * (state.dt ** 2)
        acc_min = state.pos + state.vel * state.dt + acc_min_limit * (state.dt ** 2)
        acc_interval = (acc_min, acc_max)

        # Jerk limits
        jerk_min_limit = -self.max_jerk
        jerk_max_limit = self.max_jerk
        jerk_max = state.pos + state.vel * state.dt + state.acc * \
            (state.dt ** 2) + jerk_max_limit * (state.dt ** 3)
        jerk_min = state.pos + state.vel * state.dt + state.acc * \
            (state.dt ** 2) + jerk_min_limit * (state.dt ** 3)
        jerk_interval = (jerk_min, jerk_max)
        if abs(state.vel) >= self.peak_vel:
            jerk_interval = (-1e10, 1e10)

        # print(
        #     f" limitations: jerk: {jerk_interval}, acc: {acc_interval}, vel: {vel_interval}, pos: {pos_interval}")
        
        intersection = self.intersect([vel_interval, acc_interval, jerk_interval])
        if pos_interval is not None and intersection is not None:
            intersection = self.intersect([intersection, pos_interval])
        if intersection is None:
            print(f"current state: {state}")
            raise Exception(
                f"cannot satisfy all limitations: jerk: {jerk_interval}, acc: {acc_interval}, vel: {vel_interval}, pos: {pos_interval}")

        intersection_min, intersection_max = intersection
        # print(f"intersection limits: {intersection_min} - {intersection_max}")
        return intersection_min, intersection_max

    def intersect(self, intervals):
        intersection = intervals[0]
        for interval in intervals[1:]:
            if interval[0] > intersection[1] or interval[1] < intersection[0]:
                return None
            intersection = (max(intersection[0], interval[0]), min(intersection[1], interval[1]))
        return intersection

    def get_acc_limits(self, vel):
        max_acc_limits = (-self.max_acc, self.max_acc)
        clipped_acc_limits = (self.a_acc_clip * vel - self.b_acc_clip,
                              self.a_acc_clip * vel + self.b_acc_clip)
        return self.intersect([max_acc_limits, clipped_acc_limits])


def apply_second_half(states: List[State], state_gen: StateGen):
    second_half = []
    cur_state = states[-1]
    for state in reversed(states[:-1]):
        next_state = state_gen.pos_to_state(cur_state, 2*states[-1].pos - state.pos)
        second_half.append(next_state)
        cur_state = next_state
    states.extend(second_half)


# def plot(pos, dt, state_gen: StateGen):
#     t = [dt * i for i in range(len(pos))]
#     vel_diff = np.insert(np.diff(pos) / dt, 0, 0)
#     acc_diff = np.insert(np.diff(vel_diff) / dt, 0, 0)
#     jerk_diff = np.insert(np.diff(acc_diff) / dt, 0, 0)
#     fig, axs = plt.subplots(4, 1, figsize=(8, 12), sharex=True)

#     axs[0].plot(t, pos, 'o-r', label="actual")
#     axs[0].hlines(y=state_gen.max_pos, xmin=t[0], xmax=t[-1], linewidth=1, color='y')
#     axs[0].hlines(y=state_gen.min_pos, xmin=t[0], xmax=t[-1], linewidth=1, color='y')
#     axs[0].set_xlabel('Time')
#     axs[0].set_ylabel('Position')

#     axs[1].plot(t, vel_diff, 'o-r', label="diff")
#     axs[1].hlines(y=state_gen.max_vel, xmin=t[0], xmax=t[-1], linewidth=1, color='y')
#     axs[1].hlines(y=-state_gen.max_vel, xmin=t[0], xmax=t[-1], linewidth=1, color='y')
#     axs[1].set_xlabel('Time')
#     axs[1].set_ylabel('Velocity')

#     axs[2].plot(t, acc_diff, 'o-r', label="diff")
#     axs[2].hlines(y=state_gen.max_acc, xmin=t[0], xmax=t[-1], linewidth=1, color='y')
#     axs[2].hlines(y=-state_gen.max_acc, xmin=t[0], xmax=t[-1], linewidth=1, color='y')
#     axs[2].set_xlabel('Time')
#     axs[2].set_ylabel('Acceleration')

#     axs[3].plot(t, jerk_diff, 'o-r', label="diff")
#     axs[3].hlines(y=state_gen.max_jerk, xmin=t[0], xmax=t[-1], linewidth=1, color='y')
#     axs[3].hlines(y=-state_gen.max_jerk, xmin=t[0], xmax=t[-1], linewidth=1, color='y')
#     axs[3].set_xlabel('Time')
#     axs[3].set_ylabel('Jerk')

#     plt.tight_layout()
#     plt.show()

def generate_states(state_gen: StateGen, init_state: State, pos_f):
    states = []
    cur_state = init_state
    while abs(cur_state.pos - init_state.pos) < abs(cur_state.pos - pos_f):
        # print(cur_state.pos)
        next_state = state_gen.get_valid_state(cur_state, 0.5*pos_f)
        states.append(next_state)
        cur_state = next_state
    states = states[:-1]
    if len(states) == 0:
        raise ValueError(f"Cannot create trajectory with {state_gen}")
    apply_second_half(states, state_gen)

    return states

def generate_spline(sample_freq, amplitude, repeat, dof_limit, show: bool, plateau_num = 0):
    print(dof_limit)
    dt = 1/sample_freq
    state_gen = StateGen(dof_limit)
    zero_state = State(0.0, 0.0, 0.0, 0.0, dt)
    spline_zero_to_max = generate_states(state_gen, zero_state, amplitude)
    spline_max_to_min = generate_states(state_gen, spline_zero_to_max[-1], -spline_zero_to_max[-1].pos)
    spline_min_to_zero = generate_states(state_gen, spline_max_to_min[-1], 0)
    spline_min_to_max = spline_max_to_min[::-1]

    poses = [0.0] * 10
    poses.extend([state.pos for state in spline_zero_to_max])
    poses.extend([poses[-1]] * plateau_num)
    for _ in range(repeat-1):
        poses.extend([state.pos for state in spline_max_to_min])
        poses.extend([poses[-1]] * plateau_num)
        poses.extend([state.pos for state in spline_min_to_max])
        poses.extend([poses[-1]] * plateau_num)
    poses.extend([state.pos for state in spline_max_to_min])
    poses.extend([poses[-1]] * plateau_num)
    poses.extend([state.pos for state in spline_min_to_zero])
    poses.extend([poses[-1]] * 10)
    # if show:
    #     plot(poses, dt, state_gen)

    return np.array(poses)

