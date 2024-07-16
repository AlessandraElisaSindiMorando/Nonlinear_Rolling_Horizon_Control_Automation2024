
# Scenario 1 : Circle with One Obstacle
The first scenario lasts $ 2\,\text{min}$ and considers a circular trajectory centered in the origin with radius of $2\,\text{m}$, described by the following equations

$$
    \begin{cases}
        x_{Ref}(t) = 2 \cos \left( \frac{2\pi}{60}\,t \right)\\
        y_{Ref}(t) = 2 \sin \left( \frac{2\pi}{60}\,t \right)\\
        \theta_{Ref}(t) = \frac{2\pi}{60}\,t + \frac{\pi}{2}\\
        v_{Ref}(t) = 2 \, \frac{2\pi}{60}\\
    \end{cases}
$$

A static obstacle is placed along the reference trajectory in $(x_{Obs},y_{Obs}) = (-2\,\unit{\meter},0\,\unit{\meter})$.
While the UGV should track the reference, the drone should follow the leader keeping an altitude defined as 

$$
    z_{1,Ref}(t) = \begin{cases}
        2 \, \text{m} \quad \text{if}  \quad t < 1\, \text{min} \\
        3 \, \text{m} \quad \text{if}  \quad t \geq 1\,\text{min}
    \end{cases}
$$

The aim is to demonstrate that even if the drone must increase its altitude to avoid an obstacle, the controller effectively manages this change in trajectory.
