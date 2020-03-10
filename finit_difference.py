from utils import *
from config import *

def gradient_decent_step(seed_delta_ts, n_samples=3, sampling_radius=0.01, lr=0.0005):

    X, y = sample_cost(seed_delta_ts, n_samples=n_samples, sampling_radius = sampling_radius)
    A = np.vstack([X, np.ones(len(X))]).T
    print(np.linalg.lstsq(A, y, rcond=None))
    grad = np.linalg.lstsq(A, y, rcond=None)[0][1:]
    seed_delta_ts -= lr*grad
    print('grad: {} seed_delta_ts {}'.format(grad, seed_delta_ts))
    return seed_delta_ts.tolist()
def finit_difference(seed_delta_ts, n_iterations=2, n_samples=3, sampling_radius=0.01, lr=0.0025):
    for _ in range(n_iterations):
        seed_delta_ts = gradient_decent_step(seed_delta_ts,
                                             n_samples=n_samples,
                                             sampling_radius=sampling_radius,
                                             lr=lr)
        print('new seed: {}'.format(seed_delta_ts))

if __name__ == '__main__':
    seed_delta_ts = [0.2190, 0.0865, 0.0205]
    finit_difference(seed_delta_ts, n_iterations=2, n_samples=3, sampling_radius=0.01, lr=0.0025)
