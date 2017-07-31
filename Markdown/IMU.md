欧拉角与机体角速度的关系：
$$
\dot \Theta = W^{b} \omega \\
W^b =
\left[ \begin{matrix} 1 &  tan\theta sin\phi & tan\theta cos\phi \\
                      0 & cos\theta & -sin\phi \\
                      0 & sin\phi / cos\theta & cos\phi /cos\theta
 \end{matrix}\right]
$$
旋转矩阵与机体角速度的关系：
$$
\frac{dR_{b}^{e}}{dt} = R_{b}^{e}[{}^b \omega]_{\times}
$$
四元数与机体角速度的关系：
$$
\dot q_{e}^{b}(t) = \frac{1}{2}
\left[ \begin{matrix}
0 & -{}^b \omega ^{T} \\
{}^b \omega & -[{}^b \omega]_{\times}
\end{matrix} \right]
$$
