# Scenario 4 : Lemniscate of Bernoulli with two Obstacles
The last case study is similar to the third one, but a second obstacle is added.
Both are in the reference positions that the robot should ideally pass at $t = 24\ \text{s}$ and $t = 54 \ \text{s}$.

To complicate the test, the constant height is replaced by a decreasing trajectory.

$$
    z_{1,Ref}(t) = \begin{cases}
        2 \ \text{m} \quad \text{if}  \quad t < 30\ \text{s} \\
        2 - (1.8 \, (t-30)/30)) \ \text{m} \quad \text{if}  \quad t \geq 30\ \text{s}
    \end{cases}
$$