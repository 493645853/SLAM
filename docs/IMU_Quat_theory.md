# IMU Quaternion Estimation Theory

[TOC]

## 1. Quaternion Algebra

### Definition

The quaternion is defined as
$$
\overline{q} = q_4 + q_1 i+ q_2j+q_3 k
$$
where $i,j,k$ are hyperimaginary numbers satisfying
$$
i^2=-1,\ j^2=-1,\ k^2=-1 \\
-ij=ji=k,\ -jk = kj = i,\ -ki=ik=j
$$
The quaternion can also be written in a 4-dimension vector form, as
$$
\mathbf{\overline{q}}=\begin{bmatrix} \mathbf{q}\\q_4 \end{bmatrix} = \begin{bmatrix} q_1, q_2, q_3, q_4 \end{bmatrix}^T
$$
where the imaginary part $\mathbf{q}=[k_x,k_y,k_z]^T\sin(\theta/2)$ and $q_4=\cos(\theta/2)$. The quaternion of rotation should be a unit vector, which implies
$$
|\overline{q}|=\sqrt{\overline{q}^T\overline{q}}=1
$$

### Multiplication

The quaternion multiplication is defined as
$$
\begin{align}
\mathbf{\overline{q}}\otimes\mathbf{\overline{p}}&=(q_4 + q_1 i+ q_2j+q_3 k)(p_4 + p_1 i+ p_2j+p_3 k)\\
&=\begin{bmatrix}
q_4p_1+q_3p_2-q_2p_3+q_1p_4\\
-q_3p_1+q_4p_2+q_1p_3+q_2p_4\\
q_2p_1-q_1p_2+q_4p_3+q_3p_4\\
-q_1p_1-q_2p_2-q_3p_3+q_4p_4
\end{bmatrix}
\end{align}
$$
Above operation can be written in the matrix multiplication form. In order to do this, we need to firstly introduce the skew-symmetric matrix operator $(\cdot)^\wedge$, which is for representing the inner product. For two 3D vectors $\mathbf{q}$ and $\mathbf{p}$, their inner product is defined as
$$
\mathbf{q}\cross\mathbf{p}=\left| \begin{matrix}
i&j&k\\
q_1&q_2&q_3\\
p_1&p_2&p_3
\end{matrix}\right| =
\begin{bmatrix}
0&-q_3&q_2\\
q_3&0&-q_1\\
-q_2&q_1&0
\end{bmatrix}\begin{bmatrix}p_1\\p_2\\p_3\end{bmatrix}=\mathbf{q}^\wedge\mathbf{p}
$$
and then the quaternion multiplication can be expressed as
$$
\begin{align}
\mathbf{\overline{q}}\otimes\mathbf{\overline{p}} &= \mathcal{L}(\mathbf{\overline{q}})\mathbf{\overline{p}}\\
&=\begin{bmatrix}
q_4 & q_3 & -q_2 & q_1\\
-q_3 & q_4 & q_1 & q_2 \\
q_2 & -q_1 & q_4 & q_3 \\
-q_1 & -q_2 & -q_3 & q_4
\end{bmatrix}\begin{bmatrix}p_1\\p_2\\p_3\\p_4\end{bmatrix}\\
&=\begin{bmatrix}
q_4 \mathbf{I}_{3\times3}-\mathbf{q}^\wedge & \mathbf{q}\\
-\mathbf{q}^T & q_4
\end{bmatrix}\begin{bmatrix}p_1\\p_2\\p_3\\p_4\end{bmatrix}
\end{align}
$$
Since the multiplication operation satisfy the commutation law, we can also express it as $\mathbf{\overline{q}}\otimes\mathbf{\overline{p}}=\mathcal{R}(\overline{p})\overline{q}$ . The operation $\mathcal{L}$ and $\mathcal{R}$ are important and they can be expressed as
$$
\begin{align}
\mathcal{L} &= [\Psi(\mathbf{\overline{q}}) \ \ \mathbf{\overline{q}}]\\
\mathcal{R} &= [\Xi(\mathbf{\overline{p}}) \ \ \mathbf{\overline{p}}]
\end{align}
$$
where $\Psi$ and $\Xi$ are defined  as
$$
\begin{align}
\Psi&=
\begin{bmatrix}
q_4 \mathbf{I}_{3\times3}-\mathbf{q}^\wedge \\
-\mathbf{q}^T 
\end{bmatrix}\\
\Xi&=
\begin{bmatrix}
p_4 \mathbf{I}_{3\times3}-\mathbf{p}^\wedge \\
-\mathbf{p}^T 
\end{bmatrix}
\end{align}
$$

### Quaternion & Rotation matrix



### Quaternion Time Derivative

$$
^{L(t)}_{G}\mathbf{\dot{\overline{q}}}(t)
=\lim_{\Delta t\rarr0}\frac{^{L(t+\Delta t)}_{G}\mathbf{\overline{q}}-^{L(t)}_{G}\mathbf{\overline{q}}}{\Delta t}
$$

where $^{L(t+\Delta t)}_{G}\mathbf{\overline{q}}=^{L(t+\Delta t)}_{L(t)}\mathbf{\overline{q}}\otimes^{L(t)}_{G}\mathbf{\overline{q}}$, which describes how the rotation propagates over the time. For $\Delta t\rarr 0$, we can approximate the term $^{L(t+\Delta t)}_{L(t)}\mathbf{\overline{q}}$ using only the first term of its Taylor expansion, as
$$
^{L(t+\Delta t)}_{L(t)}\mathbf{\overline{q}}=\begin{bmatrix}
\hat{\mathbf{k}}\sin(\theta/2)\\
\cos(\theta/2)
\end{bmatrix}\approx
\begin{bmatrix}
\hat{\mathbf{k}}\frac{\theta}{2}\\
1
\end{bmatrix}=
\begin{bmatrix}
\frac{\delta\boldsymbol{\theta}}{2}\\
1
\end{bmatrix}
$$
where the vector $\delta\boldsymbol{\theta}\in\mathbb{R}^3$ is in the rotation axis-angle representation. Then, by the definition of the rotational velocity, we have
$$
\boldsymbol\omega = \lim_{\Delta t\rarr 0}\frac{\delta\boldsymbol{\theta}}{\Delta t}
$$
where $\boldsymbol\omega\in\mathbb{R}^3$ denotes the rotational velocity vector in rad/s. Following above simplification, the final expression for the time derivative of quaternion can be derived as
$$
\begin{align}
^{L(t)}_{G}\mathbf{\dot{\overline{q}}}(t)
&=\lim_{\Delta t\rarr0}\frac{^{L(t+\Delta t)}_{G}\mathbf{\overline{q}}-^{L(t)}_{G}\mathbf{\overline{q}}}{\Delta t}\\

&=\lim_{\Delta t\rarr0}\frac{^{L(t+\Delta t)}_{L(t)}\mathbf{\overline{q}}\otimes^{L(t)}_{G}\mathbf{\overline{q}}-\begin{bmatrix}\mathbf{0}\\1\end{bmatrix}\otimes {^{L(t)}_{G}\mathbf{\overline{q}}}}{\Delta t}\\

&\approx\lim_{\Delta t\rarr0}\frac{\left(\begin{bmatrix}
\frac{\delta\boldsymbol{\theta}}{2}\\1\end{bmatrix}-\begin{bmatrix}\mathbf{0}\\1\end{bmatrix}\right)\otimes^{L(t)}_{G}\mathbf{\overline{q}}}{\Delta t}\\

&=\frac{1}{2}\begin{bmatrix}\boldsymbol \omega\\0\end{bmatrix} \otimes^{L(t)}_{G}\mathbf{\overline{q}}\\

&= \frac{1}{2}\begin{bmatrix}
-\boldsymbol \omega^\wedge & \boldsymbol\omega\\
-\boldsymbol\omega^T & 0
\end{bmatrix} \otimes^{L(t)}_{G}\mathbf{\overline{q}}\\

&= \frac{1}{2}\Omega(\boldsymbol\omega) ^{L(t)}_{G}\mathbf{\overline{q}}
\end{align}
$$

### Quaternion Integration

To solve the differential equation
$$
^{L(t)}_{G}\mathbf{\dot{\overline{q}}}(t)=\frac{1}{2}\Omega(\boldsymbol\omega) ^{L(t)}_{G}\mathbf{\overline{q}}(t)
$$
We have the general form of the solution
$$
^{L(t)}_{G}\mathbf{\overline{q}}(t)=\Theta(t,t_k)\underbrace{^{L(t)}_{G}\mathbf{\overline{q}}(t_k)}_{\text{initial }^{L(t_k)}_{G}\mathbf{\overline{q}}}
$$
Differentiating and reordering above expression yields
$$
\begin{align}
\dot{\Theta}(t,t_k) &= ^{L(t)}_{G}\mathbf{\dot{\overline{q}}}(t)^{L(t)}_{G}\mathbf{\overline{q}}^{-1}(t_k) \\

&=\frac{1}{2}\Omega(\boldsymbol\omega(t)) ^{L(t)}_{G}\mathbf{\overline{q}}(t)\mathbf{\overline{q}}^{-1}(t_k)\\

&=\frac{1}{2}\Omega(\boldsymbol\omega(t)) \Theta(t,t_k)
\end{align}
$$
where we assume the initial $\dot{\Theta}(t_k,t_k)=\mathbf{I}_{4\times4}$.

#### Zero-th Order Quaternion Integrator

If $\boldsymbol\omega(t)$ is constant over the time period $\Delta t=t_{k+1}-t_k$, the matrix $\Omega(\boldsymbol\omega)$ can be assumed with the constant entries and therefore the differential equation can be solved as
$$
\Theta(t_{k+1},t_k) = \Theta(\Delta t)=\exp\left(\frac{1}{2}\Omega(\boldsymbol\omega)\Delta t\right)
$$
which can be further linearized by using the Taylor series expansion
$$

$$


## 2. Attitude Propagation

### Gyroscope Signal Model



## 3. Update

### Accelerometer Signal Model

