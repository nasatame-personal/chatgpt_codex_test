# chatgpt_codex_test

This repository contains a simple simulation of a cart-pole (inverted pendulum on a cart).

The simulation is implemented in `cart_pole_sim.py` and exposes a function
`simulate_cart_pole` that accepts a controller function. The controller can be
provided using a lambda expression to quickly experiment with different control
strategies.

Example usage:

```python
from cart_pole_sim import simulate_cart_pole

controller = lambda s: -10.0 * s[2] - s[3]  # simple PD control
states = simulate_cart_pole(controller, steps=200)
print(states[-1])
```

Running the script directly will execute this example controller.
