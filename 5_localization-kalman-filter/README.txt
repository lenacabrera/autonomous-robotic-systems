

TODO
Lena
DONE - beacon -> tuple / array?
DONE - change spider robot to bubble robot (omnidirectional sensor instead of sensors at certain angles)
DONE - check intersection robot sensor with landmark / beacon, add green line between robot and feature

Kathrin
DONE - draw path / trajectory (solid line) -> for line, connect all robot positions
- robot control -> kalman filter
    - initialize matrices
        - A is identity matrix I, A_t = I -> State transitions
        - B           -> Actions
        - C           -> Sensor model, map state to observation, C_t = I (always)
        - Sigma       -> Initially set to zeros?
        - R / Epsilon -> Motion Noise
        - Q / delta   -> Sensor Noise
    - prediction part
    - correction part
- initialize covariance matrices (Sigma, R, Q) -> recording ?!

- Velocity-based motion model robot,
  Control robot with key board (W=increment 𝑣, S=decrement 𝑣, A=decrement 𝜔, D=increment 𝜔, X=stop)

Additional TODO
- position estimate: triangulation with visible landmarks, if several take average (assume no bearing info -> use 3 landmarks)
    -> measure distances, add noise afterwards to mimic sensor noise
    -> z is estimation of position based on sensor info/distances


- later/after filter: draw path / dotted line and ellipsis

- in the end, write about work distribution
