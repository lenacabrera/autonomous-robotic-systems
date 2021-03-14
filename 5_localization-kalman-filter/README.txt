

TODO
Lena
DONE - beacon -> tuple / array?
DONE - change spider robot to bubble robot (omnidirectional sensor instead of sensors at certain angles)
DONE - check intersection robot sensor with landmark / beacon, add green line between robot and feature

Kathrin
- draw path / trajectory (solid line) -> for line, connect all robot positions
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


- later/after filter: draw path / dotted line and ellipsis

- in the end, write about work distribution
