"""
Image-Based Visual Servoing (IBVS) with image moments, following

  [P1] M. Hegazy, R. A. Boby, A. Klimchik, "Localization of a Drone for Landing using Image Based
       Visual Servoing with Image Moments".

Modules
  moments.py      discrete image moments m_ij, mu_ij                       [P1 Eq.(2), p.3]
  features.py     s = [x_n, y_n, a_n, alpha], desired s*, error e          [P1 p.4-5]
  interaction.py  4x4 interaction matrix and its pseudo-inverse estimate   [P1 Eq.(10), p.5]
  controller.py   v_c = -lambda * L_hat^+ (s - s*), adaptive gain, stages  [P1 Eq.(1) p.3, p.5, Alg. 1 p.7]
  frames.py       eye-to-hand conversion and drone body frame (hardware adaptation, see docs/06)
  sources.py      camera / video-replay observations (reuses common/ camera + ArUco code)
"""
