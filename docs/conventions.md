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

Same convention as B5G-AOCS-EGSE (`QMul`, `QInv`, `M2Q`, `FrameA2B`) and Aerospace Toolbox (`quatmultiply`, `quat2dcm`), so results can be compared without conversion.

**Layout (scalar-first):**

$$
q = \begin{bmatrix} q_s \\ \mathbf{q}_v \end{bmatrix}, \qquad
q_s = \cos\frac{\theta}{2}, \quad \mathbf{q}_v = \hat{e}\sin\frac{\theta}{2}
$$

**Attitude matrix** (maps reference-frame vectors into the body frame, $\mathbf{v}_B = A(q)\,\mathbf{v}_I = q^{-1} \otimes \mathbf{v}_I \otimes q$):

$$
A(q) = \left(q_s^2 - \|\mathbf{q}_v\|^2\right) I_3 + 2\,\mathbf{q}_v\mathbf{q}_v^T - 2\,q_s\,[\mathbf{q}_v\times]
$$

**Product (Hamilton convention):**

$$
q \otimes p = \begin{bmatrix} q_s p_s - \mathbf{q}_v \cdot \mathbf{p}_v \\ q_s\,\mathbf{p}_v + p_s\,\mathbf{q}_v + \mathbf{q}_v \times \mathbf{p}_v \end{bmatrix}
$$

Frame rotations compose left to right: $q_{a2c} = q_{a2b} \otimes q_{b2c}$, hence $A(q \otimes p) = A(p)\,A(q)$.

**Kinematics** ($\boldsymbol{\omega}$ = body rate w.r.t. inertial, in body):

$$
\dot{q} = \tfrac{1}{2}\, q \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}, \qquad \dot{A} = -[\boldsymbol{\omega}\times]A
$$

**Discrete propagation** (exact for constant $\boldsymbol{\omega}$ over $\Delta t$):

$$
q_{k+1} = q_k \otimes \begin{bmatrix} \cos\!\left(\tfrac{\|\boldsymbol{\omega}\|\Delta t}{2}\right) \\ \boldsymbol{\psi} \end{bmatrix}, \quad
\boldsymbol{\psi} = \sin\!\left(\tfrac{\|\boldsymbol{\omega}\|\Delta t}{2}\right)\frac{\boldsymbol{\omega}}{\|\boldsymbol{\omega}\|}
$$

**Sign:** `att.fromDCM` returns $q_s \ge 0$ (same as EGSE `M2Q`).

**Attitude error:** `att.errorAngle` returns the total angle of $\delta q = q_A \otimes q_B^{-1}$ in rad. EGSE `QErrAng` instead returns the 3-axis small-angle vector $2\,\delta\mathbf{q}_v$ in arcsec.

## Interoperability with Aerospace Toolbox

`quatmultiply` and `quat2dcm` use the same layout and product but take 1x4 **row** vectors; `att.*` uses 4x1 **columns**. Pass `q'`. Checked by `QuaternionTest/aerospaceToolboxConsistency` when the toolbox is installed. Inside `src/fsw/` use `att.*` only.
