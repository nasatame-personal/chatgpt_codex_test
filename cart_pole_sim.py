import math
from typing import Callable, List


def simulate_cart_pole(
    controller: Callable[[List[float]], float],
    dt: float = 0.02,
    steps: int = 1000,
    initial_state: List[float] = None,
) -> List[List[float]]:
    """Simulate an inverted pendulum on a cart.

    Args:
        controller: Function that takes the current state [x, x_dot, theta, theta_dot]
            and returns a force applied to the cart.
        dt: Time step for integration.
        steps: Number of simulation steps.
        initial_state: Optional initial state. Defaults to near upright at rest.

    Returns:
        List of states over time, including the initial state.
    """
    # Physical constants
    gravity = 9.8
    masscart = 1.0
    masspole = 0.1
    total_mass = masspole + masscart
    length = 0.5  # Half the pole's length
    polemass_length = masspole * length

    if initial_state is None:
        state = [0.0, 0.0, 0.05, 0.0]  # x, x_dot, theta (rad), theta_dot
    else:
        state = initial_state[:]  # copy

    history = [state.copy()]

    for _ in range(steps):
        x, x_dot, theta, theta_dot = state
        force = controller(state)

        costheta = math.cos(theta)
        sintheta = math.sin(theta)

        temp = (force + polemass_length * theta_dot ** 2 * sintheta) / total_mass
        thetaacc = (
            gravity * sintheta - costheta * temp
        ) / (length * (4.0 / 3.0 - masspole * costheta ** 2 / total_mass))
        xacc = temp - polemass_length * thetaacc * costheta / total_mass

        x = x + dt * x_dot
        x_dot = x_dot + dt * xacc
        theta = theta + dt * theta_dot
        theta_dot = theta_dot + dt * thetaacc

        state = [x, x_dot, theta, theta_dot]
        history.append(state.copy())

    return history


if __name__ == "__main__":
    # Example controller: simple PD control to keep the pole upright.
    Kp = 10.0
    Kd = 1.0
    controller = lambda s: -Kp * s[2] - Kd * s[3]

    states = simulate_cart_pole(controller, steps=200)

    # Print final state for a quick check
    print(states[-1])
