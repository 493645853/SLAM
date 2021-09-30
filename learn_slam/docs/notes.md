# <center>**SLAM十四讲笔记** 
<center> Xin Li </center>

# Contents 
[TOC]

# Notations
Notation|Meaning
---|:--:
$a$|Scalar
$\mathbf{a}$|Vector
$\mathbf{A}$|Matrix
$(\cdot)^T$|Matrix transpose
$(\cdot)^{-1}$|Matrix inverse
$\mathcal{E}\{\cdot\}$|Expectation
$\|\mathbf{a}\|$|Euclidean norm of vector $\mathbf{a}$
$\|\mathbf{A}\|_F$|Frobenius norm of matrix $\mathbf{A}$
$\mathbb{R}$|Set of real numbers
$\mathbb{C}$|Set of complex numbers

<!-- pagebreak -->

# Matrix Transformation

## Coordinates & Basis 
A point in the real Cartesian space $\mathbb{R}^3$ can be described by the basis $(\mathbf{e}_1,\mathbf{e}_2,\mathbf{e}_3)$, as
$$
a = [\mathbf{e}_1,\mathbf{e}_2,\mathbf{e}_3]\begin{bmatrix}
        a_1 \\ a_2 \\ a_3
    \end{bmatrix}=a_1\mathbf{e}_1+a_2\mathbf{e}_2+a_3\mathbf{e}_3.
$$

## Inner/Outer Product 
For two vectors $\mathbf{a},\mathbf{b}\in \mathbb{R}^3$, their inner product is defined as
$$
\mathbf{a}\cdot\mathbf{b}=|\mathbf{a}||\mathbf{b}|\cos\left<\mathbf{a},\mathbf{b}\right>
$$
and their outer product is
$$
\begin{aligned}
    \mathbf{a}\times\mathbf{b}&=
\begin{bmatrix}
    i & j & k\\
    a_1 & a_2 & a_3\\
    b_1 & b_2 & b_3
\end{bmatrix}=
\begin{bmatrix}
    a_2b_3-a_3b_2\\
    a_3b_1-a_1b_3\\
    a_1b_2-a_2b_1
\end{bmatrix}\\&=
\begin{bmatrix}
    0 & -a_3 & a_2 \\
    a_3 & 0 & -a_1\\
    -a_2 & a_1 & 0
\end{bmatrix}\mathbf{b}
\\&\triangleq \mathbf{a}\wedge\mathbf{b}
\end{aligned}
$$

which is orthogonal to the vectors $\mathbf{a}$ and $\mathbf{b}$. Here, $\mathbf{a}\wedge$ is a "Skew-symmetric" (or anti-symmetric) matrix.

## Euclidean Transformation 
Suppose the world coordinates $(x_w,y_w,z_w)$ are stationary while the robot can be indicated by a moving coordinates $(x_c,y_c,z_c)$. Consider a vector $\mathbf{p}\in\mathbb{R}^3$ in the figure below: 
![](/home/lx6/MyWork/SLAM/learn_slam/docs/figures/coordinate_transform.png)
We can represent the vector using those two different coordinates. Assume the world coordinates are described by the basis $(\mathbf{e}_1,\mathbf{e}_2,\mathbf{e}_3)$ and the robot coordinates are described by the basis $(\mathbf{e}'_1,\mathbf{e}'_2,\mathbf{e}'_3)$, the vector $\mathbf{p}$ will not change using the representations from those two bases, i.e.,
$$
[\mathbf{e}_1,\mathbf{e}_2,\mathbf{e}_3]\begin{bmatrix}
        a_1 \\ a_2 \\ a_3
    \end{bmatrix}=
[\mathbf{e}'_1,\mathbf{e}'_2,\mathbf{e}'_3]\begin{bmatrix}
        a'_1 \\ a'_2 \\ a'_3
    \end{bmatrix}.
$$
Then, multiply the two sides with $[\mathbf{e}_1,\mathbf{e}_2,\mathbf{e}_3]^T$,
$$
\begin{aligned}
    \begin{bmatrix}
        a_1 \\ a_2 \\ a_3
    \end{bmatrix}&=
[\mathbf{e}_1,\mathbf{e}_2,\mathbf{e}_3]^T
[\mathbf{e}'_1,\mathbf{e}'_2,\mathbf{e}'_3]
\begin{bmatrix}
        a'_1 \\ a'_2 \\ a'_3
    \end{bmatrix}\\
    &\triangleq\mathbf{R}\mathbf{a}'.
\end{aligned}
$$
where $\mathbf{R}$ denotes the ***rotation matrix***, which contains certain special properties. We can define the set of the rotation matrix as
$$
SO(n)=\{\mathbf{R}\in\mathbb{R}^{n\times n}\lvert\mathbf{R}\mathbf{R}^T=\mathbf{I},\det(\mathbf{R})=1\}
$$
where $SO(n)$ stands for "***Special Orthogonal Group***". Basically, the rotation matrix can describe the rotation of certain object. We can also perform the inverse operation (the reversed rotation) by $\mathbf{R}^{-1}\mathbf{a}$ (equivalent to $\mathbf{R}^{T}\mathbf{a}$ since $\mathbf{R}$ is symmetric) to obtain the vector $\mathbf{a}'$.

Moreover, the Euclidean transformation also includes the translation of the robot's location, and thus we also need the translation vector $\mathbf{t}\in\mathbb{R}^3$ to the original expression, i.e.,
$$
\mathbf{a}'=\mathbf{R}\mathbf{a}+\mathbf{t}.
$$

## Overall Transform Matrix
We can represent above Euclidean transformation into a ***homogeneous coordinates*** form, as
$$
\begin{bmatrix}
    \mathbf{a'}\\1
\end{bmatrix}=
\begin{bmatrix}
    \mathbf{R} & \mathbf{t}\\
    \mathbf{0}^T & 1
\end{bmatrix}
\begin{bmatrix}
    \mathbf{a} \\ 1
\end{bmatrix}\triangleq
\mathbf{T}
\begin{bmatrix}
    \mathbf{a}\\1
\end{bmatrix}
$$
which allows us to represent the overall transform in a linear form, and in the rest of the notes, I will implicitly represent the homogeneous coordinates $[\mathbf{a},1]^T$ by $\mathbf{a}$ for simplicity. In addition, the set of the transform matrix $\mathbf{T}$ can be defined as
$$
SE(3)=\left\{\mathbf{T}=
\begin{bmatrix}
    \mathbf{R} & \mathbf{t}\\
    \mathbf{0}^T & 1
\end{bmatrix}\in \mathbb{R}^{4\times4}\lvert
\mathbf{R}\in\mathbb{R}^{3\times3},
\mathbf{t}\in\mathbb{R}^{3}
\right\}
$$
where $SE(3)$ denotes the ***special Euclidean group***.

## Rotation Vector & Euler Angles

In $SE(3)$, we employ totally 16 entries to describe the Euclidean transform of the object, and in $SO(3)$ we employ 9 entries to describe the rotation with 3 DoFs, which can be redundant. A more compact representation of the rotation transform can be

- Rotation axis + rotation angle
- Euler angles

which will be introduced as follows.

### Rotation Vector

Suppose the rotation axis is given by the vector $\mathbf{n}$, and the rotation angle is $\theta$, the rotation vector can be represented by $\theta\mathbf{n}$, which relates the rotation matrix using the Rodrigues’s Formula[^Rodrigue], as
$$
\mathbf{R}=\cos\theta\mathbf{I}+(1-\cos\theta)\mathbf{n}\mathbf{n}^T+\sin\theta\mathbf{n}\wedge
$$
where $\wedge$ is the operator that transform the vector to the anti-symmetric matrix. From this relation, we can also retrieve the rotation angle by
$$
\begin{align}
\tr(\mathbf{R})&=\cos\theta\tr(\mathbf{I})+(1-\cos\theta)\tr(\mathbf{n}\mathbf{n}^T)+\sin\theta\tr(\mathbf{n}\wedge)\notag\\
&=3\cos\theta+(1-\cos\theta)\notag\\
&=1+2\cos\theta
\end{align}
$$
and therefore we can obtain the rotation angle by
$$
\theta=\cos^{-1}\left\{\frac{\tr(\mathbf{R})-1}{2}\right\}
$$
In addition, if we rotate the rotation axis $\mathbf{n}$ by the matrix $\mathbf{R}$, the result are still $\mathbf{R}\mathbf{n}=\mathbf{n}$. Therefore, $\mathbf{n}$ is the eigenvector of the matrix $\mathbf{R}$ which corresponds to the eigenvalue 1. We can use eigen decomposition to find the rotation axis.

[^Rodrigue]: Refer to https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula

### Euler Angles

Euler angles can be defined in many different ways. Here we employ the "yaw-pitch-roll" in the UAV area, which is equivalent to "Z-Y-X" rotation, i.e.,

- rotate about Z axis, we have **yaw angle**
- rotate about the Y axis after the <u>first rotation</u>, we have **pitch angle**
- rotate about the X axis after the <u>second rotation</u>, we have **roll angle**

and we can also use $[r,p,y]^T$ vector to describe the Euler angles.

However, the Euler angles suffer from the well-known ***Gimbal Lock*** problem: 

![](/home/lx6/MyWork/SLAM/learn_slam/docs/figures/gimbalLock.png)

As illustrated in the graph above, if we rotate in this way, the third rotation will cause ambiguity: That is, rotate about X axis will be equivalent to the case if we rotate Z axis at the first step. This will lead to a decrease in the DoFs of the system (from DoF=3 to DoF=2). Therefore, Euler angles are usually used for the visualization purpose.


## Quaternions

In order to avoid ambiguity, we need to extend the number of variables to express the 3D rotation, which is called "quaternions".

A quaternion $\mathbf{q}$ contains 1 real part & 3 imaginary parts, i.e.,
$$
\mathbf{q}=q_0+q_1i+q_2j+q_3k,
$$
where
$$
\begin{align}
i^2=j^2=k^2=-1\\
ij=k,ji=-k,\\
jk=i,kj=-i,\\
ki=j,ik=-j
\end{align}
$$
and thus we can express the quaternions as a vector $\mathbf{q}=[s,\mathbf{v}^T]^T$, where $\mathbf{v}=[q_1,q_2,q_3]^T$  and $s=q_0$.



## C++ Eigen 3 Implementation

```c++
#include<iostream>
#include<Eigen/Eigen>
#include<Eigen/Core>
#include<Eigen/Geometry>
#include<cmath>
/**
 * @brief SLAM 14讲 第三章
 * 
 * 复习eigen库有关知识: 向量变换
 * 
 * @return int 
 */

int main()
{
    // create a matrix
    Eigen::MatrixXd mat = Eigen::MatrixXd::Identity(5,5);
    Eigen::MatrixXd covMat = mat*mat.transpose();

    // Eigendecomposition
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigen_solver(covMat);
    
    //std::cout.precision(3);
    std::cout<<"Covariance Matrix = \n"<<covMat<<std::endl;
    std::cout<<"Eigenvalues = \n"<<eigen_solver.eigenvalues()<<std::endl;
    std::cout<<"Eigenvectors = \n"<<eigen_solver.eigenvectors()<<std::endl;

    /**
     * @brief 
     * The code below is about the Eigen/Geometryw
     * 
     */
    // For the rotation matrix, we directly apply the matrix3d (or matrixXd)
    Eigen::Matrix3d rotation_mat = Eigen::Matrix3d::Identity(3,3);
    // Angle axis rotation
    Eigen::AngleAxisd rot_vec(EIGEN_PI/4, Eigen::Vector3d(0,0,1));
    // Euler angles (from rotation matrix)
    Eigen::Vector3d eulerAngles = rotation_mat.eulerAngles(2,1,0); // ZYX order


    // To rotation matrix
    std::cout<<"From axis angle to the rotation matrix\n"<<rot_vec.toRotationMatrix()<<std::endl;
    std::cout<<"From the rotation matrix to the Euler angles\n"<<eulerAngles.transpose()<<std::endl;

    // Euclidean transform matrix
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); // 4x4 matrix
    T.rotate(rot_vec);
    T.pretranslate(Eigen::Vector3d(1,3,4));// pretranslate = 提前位移, 坐标系依然是世界坐标系
    std::cout<<"Euclidean tranformation matrix\n"<<T.matrix()<<std::endl;

    //Quaternions
    Eigen::Quaterniond q1(1,0,0,0); // 直接赋值
    std::cout<<"Quaternion from the direct values\n"<<q1.coeffs()<<std::endl;
    Eigen::Quaterniond q2(rot_vec); // 从angleAxis赋值
    std::cout<<"Quaternion from the angleAxis\n"<<q2.coeffs()<<std::endl;

    //rotate other vector
    std::cout<<"(1,0,0) after rotation = "<<(q2*Eigen::Vector3d(1,0,0)).transpose()<<std::endl;

    return 0;
}
```

# Lie Group & Lie Algebra

In the previous chapter, we introduced the concepts of the $SO(3)$ and $SE(3)$:
$$
SO(n)=\{\mathbf{R}\in\mathbb{R}^{n\times n}\lvert\mathbf{R}\mathbf{R}^T=\mathbf{I},\det(\mathbf{R})=1\}
$$

$$
SE(3)=\left\{\mathbf{T}=
\begin{bmatrix}
    \mathbf{R} & \mathbf{t}\\
    \mathbf{0}^T & 1
\end{bmatrix}\in \mathbb{R}^{4\times4}\lvert
\mathbf{R}\in\mathbb{R}^{3\times3},
\mathbf{t}\in\mathbb{R}^{3}
\right\}
$$

where we can find that for two arbitrary rotation matrices $\mathbf{R}_1$ amd $\mathbf{R}_2$:
$$
\mathbf{R}_1+\mathbf{R}_2\notin SO(3)
$$

$$
\mathbf{R}_1\mathbf{R}_2\in SO(3)
$$

which implies that their addition operation is not closed but the multiplication operation is closed. Actually, we call the set with only 1 type of operation as "group". Next we'll explain ***group*** in details.

## Definition of Group

A group is an algebraic structure of one set plus one operator. If we denote a set $\mathbb{A}$ and an operator $(\cdot)$, the group can be defined as $G=(\mathbb{A},\cdot)$. We say $G$ is a group if it satisfies the following conditions:

1. Closure: $\forall a_1,a_2\in\mathbb{A},a_1\cdot a_2\in\mathbb{A}$.
2. Combination: $\forall a_1,a_2,a_3\in\mathbb{A},(a_1\cdot a_2)\cdot a_3=a_1\cdot (a_2\cdot a_3)$.
3. Unit element: $\exist a_0\in\mathbb{A}, \text{s.t.} \forall a\in\mathbb{A}, a_0\cdot a=a\cdot a_0=a$.
4. Inverse element: $\forall a\in\mathbb{A},\exist a^{-1}\in\mathbb{A},\text{s.t.}a\cdot a^{-1}=a_0$.

## Lie Algebra

Consider an arbitrary rotation matrix $\mathbf{R}$, it satisfies
$$
\mathbf{R}\mathbf{R}^T=\mathbf{I}
$$
Now, assume we want to describe the rotation of a moving object, the rotation matrix should be a continuous-time function $\mathbf{R}(t)$, and thus we have
$$
\mathbf{R}(t)\mathbf{R}^T(t)=\mathbf{I}
$$
Take the derivative of this expression on both sides with respect to $t$, as
$$
\dot{\mathbf{R}}(t)\mathbf{R}^T(t)+\mathbf{R}(t)\dot{\mathbf{R}}^T(t)=\mathbf{0}
$$
which can be simplified as
$$
\dot{\mathbf{R}}(t)\mathbf{R}^T(t)=-(\dot{\mathbf{R}}(t)\mathbf{R}^T(t))^T
$$
where we can find that the matrix $\dot{\mathbf{R}}(t)\mathbf{R}^T(t)$ is anti-symmetric. Recall that when we introduce the concepts of the inner product, an anti-symmetric matrix can be obtained by taking $(\cdot )^\wedge$ of a vector.  Therefore, we denote the anti-symmetric matrix as
$$
\phi(t)^\wedge \triangleq \dot{\mathbf{R}}(t)\mathbf{R}^T(t)
$$

Then, we can multiply $\mathbf{R}(t)$ on the both sides of the above equation, and we can obtain that
$$
\dot{\mathbf{R}}(t)=\phi(t)^\wedge \mathbf{R}(t)
$$
where we can find that taking the dirivative of the rotation matrix is equivalent to left multiply $\phi(t)^\wedge$. To find the relation between the rot

# Camera & Images

In the previous chapters, we introduced how to express the robot's motion. This chapter will mainly focus on how the robot observe the outside world using the stereo camera, which is basically related to the image projection.

## Pinhole Camera Models

Consider the pinhole model as shown in below:

![](/home/lx6/MyWork/SLAM/learn_slam/docs/figures/pinHoleModel.png)

A point $P=[X,Y,Z]^T$ in the camera's coordinate projects its image into the image plane, producing the pixel $P'=[X',Y',Z']$ in the camera's coordinate. Their coordinates are related as
$$
\frac{Z}{f}=-\frac{X}{X'}=-\frac{Y}{Y'}
$$
but the final image will automatically delete the negative sign and thus the coordinates can be written as
$$
X'=f\frac{X}{Z}, Y'=f\frac{Y}{Z}
$$
However, here $X',Y'$ are in the real world and should be converted into the pixel coordinates $u,v$, which can be obtained by
$$
\begin{align}
u&=\alpha X' + c_x\\
v&=\beta Y' + c_y
\end{align}
$$
or in a more specific form,
$$
\begin{align}
u&=f_x \frac{X}{Z} + c_x\\
v&=f_y \frac{Y}{Z} + c_y
\end{align}
$$
and if we write it into the matrix form,
$$
\begin{bmatrix}
u\\v\\1
\end{bmatrix}=\frac{1}{Z}
\begin{bmatrix}
f_x & 0 & c_x\\
0 & f_y & c_y\\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
X\\Y\\Z
\end{bmatrix}\triangleq \frac{1}{Z} \mathbf{K}\mathbf{P}
$$
where $\mathbf{K}$ is called the the *camera intrinsic* matrix. 

In addition to the camera intrinsic matrix, we also have its counterpart$-$ camera extrinsic matrix. Recall that the point $\mathbf{P}$ is in the camera's coordinate, and the camera keeps moving and thus we can express the point $\mathbf{P}$ by $[\mathbf{R}|\mathbf{t}][X_w,Y_w,Z_w,1]^T\triangleq \mathbf{T}_{nonHomo}\mathbf{P}_w$, where the extrinsic matrix $\mathbf{T}_{nonHomo}$ is not homogeneous and describe how <u>the world coordinates transform to the camera's coordinate</u>. Overall, we have the following relationship:
$$
\begin{bmatrix}
u\\v\\1
\end{bmatrix}=\frac{1}{Z}
\begin{bmatrix}
f_x & 0 & c_x\\
0 & f_y & c_y\\
0 & 0 & 1
\end{bmatrix}
\begin{bmatrix}
X\\Y\\Z
\end{bmatrix}\triangleq \frac{1}{Z} \mathbf{K}[\mathbf{R}|\mathbf{t}]
\begin{bmatrix}
X_w\\Y_w\\Z_w\\1
\end{bmatrix}
$$
where we know that the depth of the point is deleted because of $\frac{1}{Z}$.



## Camera Calibration

TODO

# Nonlinear Optimization

In this section, we'll introduce some nonlinear optimization techniques to tackle the issues in the next few chapters.

## State Estimation Problem

The SLAM problem can be concluded into the following two equations:
$$
\begin{align}
\mathbf{x}_{k}=f(\mathbf{x}_{k-1},\mathbf{u}_{k})+\mathbf{w}_{k}\\
\mathbf{z}_{k,j}=f(\mathbf{y}_{j},\mathbf{x}_{k})+\mathbf{v}_{k,j}
\end{align}
$$
where $\mathbf{x}_{k}$ is the camera's poses that can be described by $SE(3)$, $\mathbf{u}_{k}$ is the control vector.  $\mathbf{z}_{k,j}$ is the key point pixel captured by the camera at $\mathbf{x}_{k}$ for the landmark $\mathbf{y}_{j}$. We can formulate $f(\cdot)$ as
$$
s\mathbf{z}_{k,j}=\mathbf{K}(\mathbf{R}_k \mathbf{y}_{j}+\mathbf{t}_k)
$$
where $\mathbf{K}$ is the camera intrinsic matrix and $s$ is the z-coordinates of $(\mathbf{R}_k \mathbf{y}_{j}+\mathbf{t}_k)$. Moreover, we assume the noise terms $\mathbf{w}_{k}$ and $\mathbf{v}_{k,j}$ are gaussian distributed, 
$$
\begin{align}
\mathbf{w}_{k} \sim \mathcal{N}(0,\mathbf{W}_k)\\
\mathbf{v}_{k,j} \sim \mathcal{N}(0,\mathbf{V}_{k,j})
\end{align}
$$
where $\mathbf{W}_k$ and $\mathbf{V}_{k,j}$ are the covariance matrices.

One way to solve the state estimation problem is to use the *incremental* method, or the filtering method. For example, we can process the data at time instant $k$ using the extended Kalman filter. Another way is to use the batch method to process the data from $k$ to $k+\delta k$, which generally can be better than the incremental method. Next, we'll formulate the optimization problem for the state estimation.

Consider we have gathered the data with the time instants from 1 to $N$ and there're $M$ landmarks. We respectively define the time-sequence of the robot's poses and the landmarks as
$$
\mathbf{x}=\{\mathbf{x}_1,\cdots,\mathbf{x}_N\}, \mathbf{y}=\{\mathbf{y}_1,\cdots,\mathbf{y}_N\}
$$
According to the Bayesian equation, given the control vector $\mathbf{u}$ and the observation pixels $\mathbf{z}$, we have
$$
P(\mathbf{x},\mathbf{y}|\mathbf{z},\mathbf{u})=\frac{P(\mathbf{u},\mathbf{z}|\mathbf{x},\mathbf{y})P(\mathbf{x},\mathbf{y})}{P(\mathbf{z},\mathbf{u})}\propto P(\mathbf{u},\mathbf{z}|\mathbf{x},\mathbf{y})P(\mathbf{x},\mathbf{y})
$$
We can possibly use MAP estimation to find the estimates of $\mathbf{x},\mathbf{y}$, as
$$
(\mathbf{x},\mathbf{y})^*_{MAP}=\arg \max \{\underbrace{P(\mathbf{u},\mathbf{z}|\mathbf{x},\mathbf{y})}_{\text{likelihood}}\underbrace{P(\mathbf{x},\mathbf{y})}_{\text{prior}}\}
$$
However, we do not have any prior knowledge about the distribution of the robot's poses, and therefore we can only use the Maximum likelihood estimation, as
$$
(\mathbf{x},\mathbf{y})^*_{ML}=\arg \max \{P(\mathbf{u},\mathbf{z}|\mathbf{x},\mathbf{y})\}
$$


## Least Squares Problem

Now, consider the observation equation $\mathbf{z}_{k,j}=f(\mathbf{y}_{j},\mathbf{x}_{k})+\mathbf{v}_{k,j}$, we know that if $(\mathbf{y}_{j},\mathbf{x}_{k})$ are given, the observed pixel vector $\mathbf{z}_{k,j}$ can be modelled with the Gaussian distribution as
$$
\begin{align}
P(\mathbf{z}_{k,j}|\mathbf{y}_{j},\mathbf{x}_{k})&=\mathcal{N}(f(\mathbf{y}_{j},\mathbf{x}_{k}), \mathbf{V}_{k,j})\notag\\
&=\frac{1}{\sqrt{\det\{2\pi\mathbf{V}_{k,j}\}}}\exp\left(-\frac{1}{2} (\mathbf{z}_{k,j}-f(\mathbf{y}_{j},\mathbf{x}_{k}))^T \mathbf{V}_{k,j}^{-1} (\mathbf{z}_{k,j}-f(\mathbf{y}_{j},\mathbf{x}_{k})) \right)
\end{align}
$$
By using the maximum likelihood estimation, we can first take the log-likelihood of $P(\mathbf{z}_{k,j}|\mathbf{y}_{j},\mathbf{x}_{k})$ and thus the optimization problem becomes
$$
(\mathbf{x}_k,\mathbf{y}_j)^*_{ML}=\arg \min \{(\mathbf{z}_{k,j}-f(\mathbf{y}_{j},\mathbf{x}_{k}))^T \mathbf{V}_{k,j}^{-1} (\mathbf{z}_{k,j}-f(\mathbf{y}_{j},\mathbf{x}_{k}))\}
$$
We can find that this quadratic form is equivalent to minimize the noise term (which is also called the *Mahalanobis distance*). For the multiple landmarks, we can simply optimize the sum of the objective function $\forall j$.

### Nonlinear Least Squares Optimization

For a nonlinear function $f(\cdot)$, the least square optimization problem can be formulated as
$$
\min_\mathbf{x} F(\mathbf{x})=\min_\mathbf{x} \frac{1}{2}\| f(\mathbf{x}) \|^2_2
$$
We may want to find the gradient of this nonlinear function and then assign it to 0, but this can be computationally impossible. Many famous gradient descent methods have been developed, by expanding the function using Taylor series, as
$$
F(\mathbf{x}+\Delta\mathbf{x}_k) \approx F(\mathbf{x}_k) + J(\mathbf{x}_k)^T \Delta \mathbf{x}_k + \frac{1}{2}\Delta \mathbf{x}_k^T \mathbf{H}(\mathbf{x}_K)\Delta\mathbf{x}_k
$$
By either keeping the first order term or the second order term, the iterative methods can be used. The deepest descent method use the step $\Delta x^*=-J(\mathbf{x_k})$. If the second order term is kept, we can find the gradient of $F(\mathbf{x}+\Delta\mathbf{x}_k)$ and assign it to zero, i.e.,
$$
\mathbf{H}(\mathbf{X}_K)\Delta\mathbf{x}_k = -J(\mathbf{x}_k)
$$
which is the famous Newton's method. To find the solution, we need to compute the Hessian matrix $\mathbf{H}(\mathbf{X}_K)$ which can be time-expensive. Two more practical approaches can be used to substitute this method:  the *Gauss-Newton’s* method and the (*Levernburg-Marquardt’s* method).

#### The Gauss-Newton Method

Above method uses the Taylor expansion to expand the objective function $F(\cdot)$. Another way to expand the nonlinear function $f(\cdot)$, as
$$

$$


#### The Levernberg-Marquatdt Method



# Visual Odometry: Key Points

contents

## ORB

TODO

## Epipolar Geometry



## Triangulation



## PnP



# Visual Odometry: Optical Flow

# BA & Graph Optimization

