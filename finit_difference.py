from utils import *
from config import *
# input: policy parameterization θh
# for i=1 to  I do
#     generate policy variation Δθi
#     estimate J^i≈J(θh+Δθi)=⟨∑Hk=0akrk⟩ from roll-out
#     estimate J^ref , e.g., J^ref=J(θh−Δθi) from roll-out
#     compute ΔJ^i≈J(θh+Δθi)−Jref
# end for
# return gradient estimate gFD=(ΔΘTΔΘ)−1ΔΘTΔJ^
def gradient_decent_step(seed_delta_ts, n_samples=3, sampling_radius=0.01, lr=0.0005):


    Delta_J_plus, Delta_J_minus, epsilon = sample_cost(seed_delta_ts, n_samples=n_samples, sampling_radius = sampling_radius)
    Delta_J = Delta_J_plus - Delta_J_minus
    grad = np.linalg.inv(epsilon.T @ epsilon) @ epsilon.T @ Delta_J
    print(grad)
    seed_delta_ts += lr*grad
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
    finit_difference(seed_delta_ts, n_iterations=2, n_samples=15, sampling_radius=0.01, lr=0.0025)
