import numpy as np

def scale_controller_input(x:float) -> float :
    if abs(x) <= 0.05:
        return 0.0

    abs_x = abs(x)
    result = np.sign(x) * ((1.2 * np.power(1.0356, abs_x * 100.0)) - 1.2 + (0.2 * abs_x * 100.0))

    # Normalize curve output back to [-1, 1].
    max_result = (1.2 * np.power(1.0356, 100.0)) - 1.2 + (0.2 * 100.0)
    if max_result <= 0:
        return float(x)
    return float(np.clip(result / max_result, -1.0, 1.0))