import numpy as np
import random
import copy
import vamp
from pathlib import Path
from fire import Fire
import pandas as pd

# Starting and goal configurations
a = [0., -0.785, 0., -2.356, 0., 1.571, 0.785]
b = [2.35, 1., 0., -0.8, 0, 2.5, 0.785]

# Problem specification (sphere centers)
problem = [
    [0.55, 0, 0.25], [0.35, 0.35, 0.25], [0, 0.55, 0.25], [-0.55, 0, 0.25],
    [-0.35, -0.35, 0.25], [0, -0.55, 0.25], [0.35, -0.35, 0.25], [0.35, 0.35, 0.8],
    [0, 0.55, 0.8], [-0.35, 0.35, 0.8], [-0.55, 0, 0.8], [-0.35, -0.35, 0.8],
    [0, -0.55, 0.8], [0.35, -0.35, 0.8]
]

def test_set_joint_limits():
    lower_limits = [-2.5, -2.0, -2.5, -2.8, -2.5, -0.1, -2.5]
    upper_limits = [2.5, 2.0, 2.5, 2.8, 2.5, 0.1, 2.5]

    # Set joint limits
    # breakpoint()
    vamp.panda.set_joint_limits(lower_limits, upper_limits)
    print("Joint limits set successfully.")

    # Configure robot and planner
    vamp_module, planner_func, plan_settings, simp_settings = vamp.configure_robot_and_planner_with_kwargs(
        "panda", "rrtc"
    )

    # Setup sampler
    sampler = vamp_module.halton()
    sampler.skip(10)

    # Setup environment with spheres
    environment = vamp.Environment()
    for sphere in problem:
        environment.add_sphere(vamp.Sphere(sphere, 0.2))

    # Validate the start and goal configuration
    # result_a = vamp.panda.validate(a, environment)
    # result_b = vamp.panda.validate(b, environment)

    # print(f"Validation of start config: {result_a}")
    # print(f"Validation of goal config: {result_b}")

    # # Plan and simplify
    # if result_a and result_b:
    result = planner_func(a, b, environment, plan_settings, sampler)
    simplified = vamp_module.simplify(result.path, environment, simp_settings, sampler)

    # print("Path found:")
    # for point in simplified.path:
    #     print(point)

    # Convert to DataFrame
    results = {
        "planning_time": result.nanoseconds,
        "simplification_time": simplified.nanoseconds,
        "initial_path_cost": result.path.cost(),
        "simplified_path_cost": simplified.path.cost(),
        "planning_iterations": result.iterations
    }

    df = pd.DataFrame([results])
    print(df.describe())

# Run the test
if __name__ == "__main__":
    test_set_joint_limits()
