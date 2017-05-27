四元数旋转矩阵(从body坐标系到world坐标系)：
$$
R^{w}_{b} =
\left[ \begin{matrix}
q^{2}_{w} + q^{2}_{x} - q^{2}_{y} - q^{2}_{z} & 2(q_xq_y - q_wq_z) & 2(q_xq_z + q_wq_y)   \\
2(q_xq_y + q_wq_z) & q^{2}_{w} - q^{2}_{x} + q^{2}_{y} - q^{2}_{z} & 2(q_yq_z - q_wq_x) \\
2(q_xq_z - q_wq_y) & 2(q_yq_z + q_wq_x) & q^{2}_{w} - q^{2}_{x} - q^{2}_{y} + q^{2}_{z} \\
\end{matrix} \right]
$$

数值积分方法选择欧拉方法:
$$ x_{n+1} = x_{n} + \Delta \cdot f(t_{n},x_{n}) $$
状态量选择：
$$ x= \left[ \begin{matrix} q \\ w_{b} \end{matrix} \right]$$

**step1:** \
初始化四元数:
$$
\psi \ yaw \ Z axis\\
\theta \ pitch \ Y axis\\
\varphi \ roll \ X axis
$$
$$
\theta = atan2(\frac{Acc.x}{Acc.y^2+Acc.z^2}) \\
\varphi = atan2(\frac{-Acc.y}{-Acc.z}) \\
\psi = atan2(\frac{-Mag.y \ast cos(\varphi)+Mag.z \ast sin(\varphi)}{Mag.x \ast cos(\theta)+ Mag.y \ast sin(\theta)sin(\varphi)+Mag.z \ ast sin(\theta)cos(\varphi)})
$$
初始化$Q$过程激励噪声协方差矩阵和$R$观测噪声协方差矩阵：
$$
Q = \left[ \begin{matrix} w_n & 0 \\ 0 & w_{bn} \end{matrix} \right]
$$
$$
R = \left[ \begin{matrix} a_n & 0 \\ 0 & m_n \end{matrix} \right]
$$
**step2:** \
状态观测量：
需要注意的是这里的状态观测量需要对加速度和磁力计进行归一化，这个非常重要。
$$
z = \left[ \begin{matrix} a_m \\ m_m \end{matrix} \right]
$$


**step3:** \
状态转移方程：
$$ q \gets q + \frac{1}{2} q \otimes q\{ (w_{m} - w_{b} )\Delta t\} \\
w_{b} \gets w_{b}$$

将四元数状态转移方程展开则为如下公式：
$$
\hat x_{k|k-1} = f(\hat x_{k-1|k-1},u_{k-1})
$$
$$
q \otimes q\{ (w_{m} - w_{b})T \}  = \left[ \begin{matrix} q_w \\ q_x \\ q_y \\ q_z \end{matrix} \right] \otimes
\left[ \begin{matrix} 0 \\ w_x - w_{bx} \\ w_y - w_{by} \\ w_z - w_{bz} \end{matrix} \right] \ast \frac{T}{2} \\
= \left[ \begin{matrix}
-q_x(w_x - w_{bx})  -q_y(w_y - w_{by})  -q_z(w_z - w_{bz}) \\
 q_w(w_x - w_{bx})  +q_y(w_z - w_{bz})  -q_z(w_y - w_{by}) \\
 q_w(w_y - w_{by})  -q_x(w_z - w_{bz})  +q_z(w_x + w_{bx}) \\
 q_w(w_z - w_{bz})  +q_x(w_y - w_{by})  -q_y(w_x - w_{bx}) \\
\end{matrix} \right] \ast \frac{T}{2}
$$
需要进行对状态量中的四元数中归一化。

**step4:** \
求状态转移方程的雅克比矩阵和计算协方差矩阵：
$$
F_{k-1} = \frac{\partial f}{\partial x}|_{\hat x_{k|k-1}|u_{k-1}}
$$
$$
  F_{k} = \left[ \begin{matrix} R^{T}\{(w_m - w_b) \} & -I\Delta t \\  
  0 & I \\ \end{matrix} \right]
$$
$$
\dot q^{n}_{nb} = \frac{1}{2} \Omega^{n}_{nb} q^{b}_{n} \\
A_{TC} = \frac{1}{2}\Omega^{n}_{nb} \\
\dot q^{n}_{nb} = lim_{T\to0} \frac{q^{b}_{n}(t+T) - q^{b}_{n}(t)}{T} = A_(TC)q^{b}_{n}(t) \\
q^{b}_{n}(t+T) = q^{b}_{n}(t) + A_{TC}q^{b}_{n}(t)T = (I + A_{TC}T)q^{b}_{n}(t) \\
A_k = (I + A_{TC}T)\\
\Omega^{n}_{nb} = \left[ \begin{matrix}
  0 & -\omega_{x} & -\omega_{y} & -\omega_{z} \\
  \omega_{x} & 0 & \omega_{z} & -\omega_{y} \\
  \omega_{y} & -\omega_{z} & 0 & \omega_{x} \\
  \omega_{z} & \omega_{y} & -\omega_{x} & 0 \\
 \end{matrix} \right]
$$
$$
P_{k|k-1} = F_{k-1}\cdot P_{k-1|k-1} \cdot F^{T}_{k-1}+Q_{k-1} \\
$$
**step5：** \
测量方程：
重力在世界坐标系下归一化可以得到重力参考值
$$
A_{ref} = \left[ \begin{matrix} 0 & 0 & |g| \end{matrix} \right]^T
$$
加速度计可以测量重力在机体坐标系下各轴的分量。
$$
a = R_{t}^{T}(a_t - g_t) + a_{bt} + a_n \\
w_m = w_t + w_{bt} + w_n \\
$$
将上述方程进行转置，得到真值：
$$
a_t = R_t(a_m - a_bt - a_n) + g_t \\
w_t = w_m - w_{bt} - w_n
$$
为了消除环境因素的干扰，需要进行坐标变换；因为磁倾角的存在，地球的磁场强度可以认为是由水平和垂直两部分组成的。一个加速度计可以提供一个姿态的参考，而且可以用来补偿测量地球磁场强度中倾斜错误。将传感器数值从传感器参考系转换到地球参考系。
$$ h_{mk} = q \otimes m_k \otimes q^{\ast} \\
   b_k = \left[ \begin{matrix} 0 & \sqrt[2]{h_{x}^{2} + h_{y}^{2}} & 0 & h_z  \end{matrix} \right] \\
$$
$$
h_2() = \hat m= (R^b_n)^T h_{mk}  =
\left[ \begin{matrix}
q^{2}_{w} + q^{2}_{x} - q^{2}_{y} - q^{2}_{z} & 2(q_xq_y - q_wq_z) & 2(q_xq_z + q_wq_y)   \\
2(q_xq_y + q_wq_z) & q^{2}_{w} - q^{2}_{x} + q^{2}_{y} - q^{2}_{z} & 2(q_yq_z - q_wq_x) \\
2(q_xq_z - q_wq_y) & 2(q_yq_z + q_wq_x) & q^{2}_{w} - q^{2}_{x} - q^{2}_{y} + q^{2}_{z} \\
\end{matrix} \right]^T
\ast
\left[ \begin{matrix}  b_x \\ 0 \\ b_z  \end{matrix} \right] \\
= \left[ \begin{matrix}
b_x\cdot (q^{2}_{w} + q^{2}_{x} - q^{2}_{y} - q^{2}_{z}) + 2b_z\cdot(q_xq_z - q_wq_y)   \\
2b_x\cdot(q_xq_y - q_wq_z) + 2b_z\cdot(q_yq_z + q_wq_x) \\
2b_x\cdot(q_wq_y + q_xq_z) + b_z\cdot(q^{2}_{w} - q^{2}_{x} - q^{2}_{y} + q^{2}_{z}) \\
\end{matrix} \right]
$$
对$h_2$求解雅克比矩阵：
$$
H_{k2}=
\left[ \begin{matrix}
2b_xq_w-2b_zq_y & 2b_xq_x+2b_zq_z & -2b_xq_y-2b_zq_w & -2b_xq_z+2b_zq_x   \\
-2b_xq_z+2b_zq_x & 2b_xq_y+2b_zq_w & 2b_xq_x+2b_zq_z & -2b_xq_w+2b_zq_y   \\
2b_xq_y+2b_zq_w & 2b_xq_z-2b_zq_x & 2b_xq_w-2b_zq_y & 2b_xq_x+2b_zq_z   \\
\end{matrix} \right]
$$
$$
h_1() = \hat g=(R^b_n)^T \left[ \begin{matrix}  0 \\ 0 \\ |g|  \end{matrix} \right] = |g| \left[ \begin{matrix} 2q_{x}q_{z} -2q_{w}q_{y}\\ 2q_{w}q_{x} + 2q_{y}q_{z} \\ q_w^2 - q_x^2- q_y^2 + q_z^2   \end{matrix} \right]
$$

$$
H_{k1} = \frac{\partial h_{1[i]}}{\partial q_{[j]}}=\left[ \begin{matrix}
-2q_y & 2q_z & -2q_w & 2q_x  \\
2q_x & 2q_w & 2q_z & 2q_y  \\
2q_w & -2q_x & -2q_y & 2q_z  \\
\end{matrix} \right]
$$

**step6：** \
计算卡尔曼增益：
$$K_K = P_{k|k-1}H^{T}_{k}(H_kP_{k|k-1} H^{T}_{k} + R_k)^{-1} $$
**step7:** \
更新后验状态：
$$
\hat x_{k|k} = \hat x_{k|k-1} + K_k(Z_k -h(\hat x_{k|k-1}))
$$
**step7:** \
更新后验误差协方差矩阵：
$$
P_{k|k} = (I-K_{k}H_{k})P_{k|k-1}
$$
**注意事项：** \
1.矩阵的构造应该是尝试使用基础矢量或者矩阵的数学构造的方式，而不是通过人为的来填充这样太容易出现错误，而且很不容易查找错误。
