# Conventions

These conventions apply to every module. Mixing conventions is the most common source of sign errors in attitude software, so any deviation must be explicit and converted at the interface.

## Units

SI units internally (m, s, kg, N, T). Angles in radians. Degrees and arcseconds appear only in configuration comments and plots.

## Reference frames

| Frame | Symbol | Definition |
|---|---|---|
| Inertial | ECI | GCRF / J2000 |
| Earth-fixed | ECEF | ITRF |
| Local orbital | LVLH | z: nadir (−r̂), y: negative orbit normal (−ĥ), x: completes the triad (≈ velocity) |
| Body | B | Fixed to spacecraft structure, origin at CoM |

## Quaternions

**Layout (scalar-last):**

$$
q = \begin{bmatrix} \mathbf{q}_v \\ q_4 \end{bmatrix}, \qquad
\mathbf{q}_v = \hat{e}\sin\frac{\theta}{2}, \quad q_4 = \cos\frac{\theta}{2}
$$

**Attitude matrix** (maps reference-frame vectors into the body frame, $\mathbf{v}_B = A(q)\,\mathbf{v}_I$):

$$
A(q) = \left(q_4^2 - \|\mathbf{q}_v\|^2\right) I_3 + 2\,\mathbf{q}_v\mathbf{q}_v^T - 2\,q_4\,[\mathbf{q}_v\times]
$$

**Product (Shuster convention)**, chosen so that quaternions compose in the same order as attitude matrices:

$$
q \otimes p = \begin{bmatrix} q_4\,\mathbf{p}_v + p_4\,\mathbf{q}_v - \mathbf{q}_v \times \mathbf{p}_v \\ q_4 p_4 - \mathbf{q}_v \cdot \mathbf{p}_v \end{bmatrix},
\qquad A(q \otimes p) = A(q)\,A(p)
$$

**Kinematics** ($\boldsymbol{\omega}$ = body rate w.r.t. inertial, in body):

$$
\dot{q} = \tfrac{1}{2}\,\Omega(\boldsymbol{\omega})\,q, \qquad
\Omega(\boldsymbol{\omega}) = \begin{bmatrix} -[\boldsymbol{\omega}\times] & \boldsymbol{\omega} \\ -\boldsymbol{\omega}^T & 0 \end{bmatrix},
\qquad \dot{A} = -[\boldsymbol{\omega}\times]A
$$

**Discrete propagation** (exact for constant $\boldsymbol{\omega}$ over $\Delta t$):

$$
q_{k+1} = \bar{\Omega}\,q_k, \quad
\bar{\Omega} = \begin{bmatrix} \cos\!\left(\tfrac{\|\boldsymbol{\omega}\|\Delta t}{2}\right) I_3 - [\boldsymbol{\psi}\times] & \boldsymbol{\psi} \\ -\boldsymbol{\psi}^T & \cos\!\left(\tfrac{\|\boldsymbol{\omega}\|\Delta t}{2}\right) \end{bmatrix}, \quad
\boldsymbol{\psi} = \sin\!\left(\tfrac{\|\boldsymbol{\omega}\|\Delta t}{2}\right)\frac{\boldsymbol{\omega}}{\|\boldsymbol{\omega}\|}
$$

**Attitude error** (used by the MEKF): $\delta q = q \otimes \hat{q}^{-1}$, with small-angle approximation $\delta\boldsymbol{\theta} \approx 2\,\delta\mathbf{q}_v$.

## Interoperability with Aerospace Toolbox

Aerospace Toolbox functions (`quat2dcm`, `quatmultiply`, …) use **scalar-first** row vectors and the **Hamilton** product.

- Component order: convert with `att.toScalarFirst` / `att.fromScalarFirst`. After reordering, `quat2dcm` returns the same matrix as `att.toDCM`.
- Multiplication order is reversed: `q ⊗ p` (Shuster) equals `quatmultiply(p, q)` (Hamilton, after reordering).

Both facts are checked by `QuaternionTest/aerospaceToolboxConsistency`, which runs automatically when the toolbox is installed. Never call `quatmultiply` inside `src/fsw/`; use `att.multiply`.
