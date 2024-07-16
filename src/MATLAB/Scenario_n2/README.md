# Scenario 2 : Circle with Two Obstacles
The second case study maintains the circular trajectory of the first scenario, but considers two obstacles, placed in 

$$(x_{Obs1},y_{Obs1}) = (0\ \text{m},2\ \text{m})$$

and

$$(x_{Obs2},y_{Obs2}) = (0\ \text{m},2\ \text{m})$$

The aircraft should adhere to a specified take-off reference trajectory, as delineated below

$$
    z_{1,Ref}(t) = \begin{cases}
        (1 + (t/30)) \ \text{m} \quad \text{if}  \quad t < 30\,\text{s}\\
        2 \ \text{m} \quad \text{if}  \quad t \geq 30\,\text{s}
    \end{cases}
$$

The simulation time is also reduced to one minute.