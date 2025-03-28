1. Determine the expression for $a_{z,k}$.

$a_{z,k}=u_{z,k}-gravity$

2. What are the matrix / vector sizes for $\mathbf{H}$, the innovation, the innovation covariance, and $\mathbf{K}$?

$H=[1 ​0​]$ is a 1x2 vector.

The innovation $y = z_{\text{sonar}} - H X_z$ is 1x1, because $z_{\text{sonar}}$​ is a scalar measurement.

The innovation covariance $S = \mathbf{H}_k \mathbf{P}_{k|k-1} \mathbf{H}_k^\top  + \mathbf{V}_k \mathbf{R}_k \mathbf{V}_k^\top$ is also 1x1, a scalar.

$\mathbf{K}$ is a 2x1 vector.

3. In order to determine $\sigma_{snr,z}^2$, one way is to make the drone stationary and collect about 100 samples from the sonar sensor and determine the variance from the samples. 
However, the drone may slowly drift upwards. 
In this situation, concisely describe your process of determining $\sigma_{snr,z}^2$ by using a best fit line and further tuning. An essay is **not** expected for this answer.

First, collecting 100 sonar measurements while the drone is stationary and using linear regression to fit a best-fit line $z_{\text{fit}}(t)=at+b$ to model the slow drift. Then, calculating the residuals $\epsilon_i = z_{\text{meas,i}}-z_{\text{fit}}(t_i)$ and determining the variance of the residuals through $\sigma_{snr,z}=\frac{1}{N}\sum_{i=1}^{N}(\epsilon_i-\overline{\epsilon})^2$. If the filter performance is unstable, slightly adjust $\sigma_{snr,z}$​ based on observed noise characteristics (e.g., increase it if corrections fluctuate excessively).