# Scenario 3 : Circle with Two Obstacles
For the third simulation, Bernoulli's Lemniscate is chosen as the desired trajectory, described by the following equations

$$
    \begin{cases}
        x_{Ref}(t) &= 2 \, c(t) \, \cos \left( t \, \frac{2\pi}{60}  \right) \\
        y_{Ref}(t) &= \, c(t) \, \sin \left( 2t \, \frac{2\pi}{60} \right) \\
        \theta_{Ref}(t) &= \text{arctan2}(v_{y,Ref}(t),v_{x,Ref}(t))\\
        v_{Ref}(t) &= \sqrt{v_{x,Ref}(t)^2 + v_{y,Ref}(t)^2}
    \end{cases}
$$

with 

$$
    c(t) = 2\, / \,\left( 3 - \cos \left(  2t  \, \frac{2\pi}{60} \right) \right)
$$

$$
    v_{x,Ref}(t) = \left.\frac{d  x_{Ref}(\tau) }{d\tau}\right\vert_{\tau = t} \quad
    v_{y,Ref}(t) = \left.\frac{d  y_{Ref}(\tau) }{d\tau}\right\vert_{\tau = t}
$$

To get an analytic expression of the reference forward velocity and orientation, the script **ComputeReferenceBernoulli** has been run.

and the only obstacle is in the origin. The reference altitude of the UAV is kept constant at 

$$z_{1,Ref}(t) = 2 \ \text{m}$$