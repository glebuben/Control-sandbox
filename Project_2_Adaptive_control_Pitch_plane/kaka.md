Based on the adaptive control lecture and the submarine example, I can now significantly enhance our aircraft model with adaptive control capabilities. Here's the improved, comprehensive model:

---

# 🛩️ **AIRCRAFT LONGITUDINAL DYNAMICS WITH ADAPTIVE CONTROL**

## **1. ENHANCED STATE-SPACE MODEL**

### **State Vector:**
$$s = \begin{bmatrix} V \\ \alpha \\ q \\ \theta \\ h \end{bmatrix} \in \mathbb{R}^5$$

**New addition:** $h$ = altitude (for position tracking)

### **Control (Action) Vector:**
$$a = \begin{bmatrix} \delta_e \\ \delta_t \end{bmatrix} \in \mathbb{R}^2$$

### **Enhanced Dynamics with Uncertainties:**
$$\dot{s} = P(s, a, \theta_p) + d(t)$$

Where $\theta_p$ represents **unknown or uncertain parameters**:
- $m$: actual mass (may differ from nominal)
- $I_y$: actual moment of inertia
- $C_{L_\alpha}, C_{m_\alpha}$: uncertain aerodynamic derivatives
- $\rho$: air density variations

---

## **2. AERODYNAMIC MODEL WITH PARAMETRIC UNCERTAINTIES**

### **Lift and Drag with Uncertain Coefficients:**
$$L = \frac{1}{2}\rho V^2 S \left[C_{L_0} + C_{L_\alpha}\alpha + C_{L_{\delta_e}}\delta_e + \Delta C_L\right]$$

$$D = \frac{1}{2}\rho V^2 S \left[C_{D_0} + \frac{(C_L - C_{L_0})^2}{\pi e \cdot AR} + \Delta C_D\right]$$

Where $\Delta C_L, \Delta C_D$ represent **unmodeled aerodynamic effects**.

### **Pitching Moment with Uncertainties:**
$$M = \frac{1}{2}\rho V^2 S \bar{c} \left[C_{m_0} + C_{m_\alpha}\alpha + C_{m_q}\frac{q\bar{c}}{2V} + C_{m_{\delta_e}}\delta_e + \Delta C_m\right]$$

---

## **3. ADAPTIVE CONTROL STRATEGIES**

### **3.1 CERTAINTY-EQUIVALENCE ADAPTIVE CONTROL**

#### **Control Objective:**
Track reference trajectories $V_{ref}(t)$, $\alpha_{ref}(t)$, $h_{ref}(t)$ despite unknown parameters.

#### **Nominal Control Law (without adaptation):**
$$\delta_{e,nom} = -K_{p_\alpha}(\alpha - \alpha_{ref}) - K_{d_\alpha}q$$
$$\delta_{t,nom} = -K_{p_V}(V - V_{ref}) - K_{d_V}\dot{V}$$

#### **Adaptive Parameter Estimates:**
Introduce estimates for uncertain parameters:
- $\hat{m}$: estimated mass
- $\hat{C}_{L_\alpha}$: estimated lift slope
- $\hat{C}_{m_\alpha}$: estimated pitch stiffness
- $\hat{d}_V, \hat{d}_\alpha, \hat{d}_q$: estimated disturbance forces/moments

#### **Adaptation Laws (Lyapunov-based):**

**For mass estimation:**
$$\dot{\hat{m}} = \gamma_m \left[(\dot{V} + \lambda_V e_V)\cos\alpha + (\dot{\alpha} - q + \lambda_\alpha e_\alpha)\frac{\sin\alpha}{V}\right]$$

**For lift coefficient:**
$$\dot{\hat{C}}_{L_\alpha} = \gamma_{L_\alpha} (\dot{\alpha} - q + \lambda_\alpha e_\alpha) \cdot \frac{\rho V S}{2m} \alpha$$

**For disturbance estimation:**
$$\dot{\hat{d}}_V = \gamma_{d_V} (\dot{V} + \lambda_V e_V)$$
$$\dot{\hat{d}}_\alpha = \gamma_{d_\alpha} (\dot{\alpha} - q + \lambda_\alpha e_\alpha)$$
$$\dot{\hat{d}}_q = \gamma_{d_q} (\dot{q} + \lambda_q e_q)$$

Where:
- $e_V = V - V_{ref}$, $e_\alpha = \alpha - \alpha_{ref}$, $e_q = q - q_{ref}$
- $\gamma_{(\cdot)} > 0$: adaptation gains
- $\lambda_{(\cdot)} > 0$: filter/time-constant parameters

#### **Compensated Control Law:**
$$\delta_e = \delta_{e,nom} - \frac{\hat{d}_q}{\frac{1}{2}\rho V^2 S \bar{c} \hat{C}_{m_{\delta_e}}}$$
$$\delta_t = \delta_{t,nom} - \frac{\hat{d}_V}{T_{max}\cos\alpha}$$

---

### **3.2 IMMERSION & INVARIANCE (I&I) ADAPTIVE CONTROL**

#### **Core Concept:**
Design an invariant manifold where the system behaves as if parameters are known.

#### **Extended State with Adaptive Variables:**
Introduce internal adaptive states $\chi = [\chi_V, \chi_\alpha, \chi_q]^T$:

$$\hat{d}_V = \chi_V + m\lambda_V(\dot{V} + \lambda_V e_V)$$
$$\hat{d}_\alpha = \chi_\alpha + m\lambda_\alpha(\dot{\alpha} - q + \lambda_\alpha e_\alpha)$$
$$\hat{d}_q = \chi_q + I_y\lambda_q(\dot{q} + \lambda_q e_q)$$

#### **I&I Adaptation Laws:**
$$\dot{\chi}_V = \lambda_V\left[K_{p_V}e_V + K_{d_V}\dot{V} + \frac{1}{2}\rho V S \hat{C}_{D} \dot{V}\right]$$
$$\dot{\chi}_\alpha = \lambda_\alpha\left[K_{p_\alpha}e_\alpha + K_{d_\alpha}(\dot{\alpha} - q) + \frac{1}{2}\rho V S \hat{C}_{L} (\dot{\alpha} - q)\right]$$
$$\dot{\chi}_q = \lambda_q\left[K_{p_q}e_q + K_{d_q}\dot{q} + \frac{1}{2}\rho V^2 S \bar{c} \hat{C}_{m_\alpha} \dot{q}\right]$$

#### **I&I Control Law:**
$$\delta_e = -K_{p_\alpha}e_\alpha - K_{d_\alpha}q - \frac{\hat{d}_\alpha}{\frac{1}{2}\rho V^2 S \bar{c}}$$
$$\delta_t = -K_{p_V}e_V - K_{d_V}\dot{V} - \frac{\hat{d}_V}{T_{max}}$$

---

## **4. LYAPUNOV STABILITY ANALYSIS**

### **Lyapunov Function Candidate:**
$$V = \frac{1}{2}m e_V^2 + \frac{1}{2}m V^2 e_\alpha^2 + \frac{1}{2}I_y e_q^2 + \frac{1}{2\gamma_m}\tilde{m}^2 + \frac{1}{2\gamma_{L_\alpha}}\tilde{C}_{L_\alpha}^2 + \frac{1}{2\gamma_d}\|\tilde{d}\|^2$$

Where $\tilde{(\cdot)} = \hat{(\cdot)} - (\cdot)^*$ are parameter estimation errors.

### **Time Derivative:**
$$\dot{V} = -K_{d_V}e_V^2 - K_{d_\alpha}V^2 e_\alpha^2 - K_{d_q}e_q^2 \leq 0$$

**This guarantees:**
- Uniform stability of tracking errors
- Boundedness of all signals
- Convergence to a neighborhood of zero (practical stability)

---

## **5. REFERENCE MODEL FOR MODEL REFERENCE ADAPTIVE CONTROL (MRAC)**

### **Desired Closed-Loop Dynamics:**
$$\dot{V}_m = -a_V(V_m - V_{ref})$$
$$\dot{\alpha}_m = -a_\alpha(\alpha_m - \alpha_{ref})$$
$$\dot{q}_m = -a_q(q_m - q_{ref})$$

Where $a_V, a_\alpha, a_q > 0$ define desired convergence rates.

### **Tracking Error Dynamics:**
$$e_V = V - V_m, \quad e_\alpha = \alpha - \alpha_m, \quad e_q = q - q_m$$

### **MRAC Adaptation with Reference Model:**
$$\dot{\hat{C}}_{L_\alpha} = \gamma_{L_\alpha} e_\alpha \cdot \frac{\rho V S}{2m} \alpha$$
$$\dot{\hat{C}}_{m_\alpha} = \gamma_{m_\alpha} e_q \cdot \frac{\rho V^2 S \bar{c}}{2I_y} \alpha$$

---

## **6. COMPLETE ADAPTIVE CONTROL ALGORITHM**

### **Step 1: Measure State**
$$s = [V, \alpha, q, \theta, h]^T$$

### **Step 2: Compute Tracking Errors**
$$e_V = V - V_{ref}, \quad e_\alpha = \alpha - \alpha_{ref}, \quad e_q = q - q_{ref}$$

### **Step 3: Update Parameter Estimates**
Using either:
- **Certainty-Equivalence:** Direct parameter estimation
- **I&I:** Update $\chi$ states and compute $\hat{d} = \chi + m\Lambda v$

### **Step 4: Compute Compensated Control**
$$\delta_e = -K_{p_\alpha}e_\alpha - K_{d_\alpha}q - \text{compensation term}$$
$$\delta_t = -K_{p_V}e_V - K_{d_V}\dot{V} - \text{compensation term}$$

### **Step 5: Apply Saturation**
$$\delta_e = \text{clip}(\delta_e, -\delta_{e,max}, \delta_{e,max})$$
$$\delta_t = \text{clip}(\delta_t, 0, 1)$$

---

## **7. ENHANCED PARAMETER TABLE**

| Parameter | Symbol | Nominal Value | Adaptive? | Notes |
|-----------|--------|---------------|-----------|-------|
| Mass | $m$ | 1100 kg | ✓ | Can vary with fuel |
| Pitch inertia | $I_y$ | 1800 kg·m² | ✓ | Uncertain |
| Lift slope | $C_{L_\alpha}$ | 5.0 rad⁻¹ | ✓ | Changes with configuration |
| Pitch stiffness | $C_{m_\alpha}$ | −0.6 rad⁻¹ | ✓ | Depends on CG location |
| Pitch damping | $C_{m_q}$ | −8.0 rad⁻¹ | ✗ | Usually well-known |
| Drag coefficient | $C_{D_0}$ | 0.03 | ✗ | Relatively constant |
| Disturbance (V) | $d_V$ | Unknown | ✓ | Wind, turbulence |
| Disturbance (α) | $d_\alpha$ | Unknown | ✓ | Gusts, wake |
| Adaptation gain (m) | $\gamma_m$ | 0.1-1.0 | Tuning | Higher = faster adaptation |
| Adaptation gain (CLα) | $\gamma_{L_\alpha}$ | 0.01-0.1 | Tuning | Trade-off speed/stability |
| Filter parameter (V) | $\lambda_V$ | 0.5-2.0 | Tuning | Error filter time constant |

---

## **8. ADVANTAGES OF ADAPTIVE CONTROL**

### **vs. Fixed-Gain Control:**
✅ **Handles parameter variations** (mass changes, CG shifts)  
✅ **Compensates for unknown disturbances** (wind, turbulence)  
✅ **Robust to modeling errors** (uncertain aerodynamics)  
✅ **Maintains performance** across flight envelope  
✅ **No need for gain scheduling** (automatically adjusts)  

### **Certainty-Equivalence vs. I&I:**

| Feature | Certainty-Equivalence | I&I |
|---------|----------------------|-----|
| **Complexity** | Simpler | More complex |
| **Estimation** | Direct parameter estimates | Implicit via manifold |
| **Stability proof** | Standard Lyapunov | Manifold invariance |
| **Tuning** | Intuitive gains | $\lambda$ parameters critical |
| **Performance** | Good for slow variations | Better for fast uncertainties |

---

## **9. PRACTICAL IMPLEMENTATION NOTES**

### **Numerical Considerations:**
1. **Avoid division by zero:** Add $\epsilon$ when dividing by $V$ or $\rho$
2. **Clip parameter estimates:** Prevent unbounded growth
3. **Use low-pass filters:** Smooth adaptation signals
4. **Dead-zone:** Freeze adaptation when errors are small

### **Gain Tuning Guidelines:**
- Start with small $\gamma$ (0.01-0.1)
- Increase $\lambda$ for faster error convergence (0.5-2.0)
- Ensure $K_p, K_d$ provide stable nominal dynamics
- Verify $\dot{V} \leq 0$ in simulation

### **Flight Envelope Protection:**
Add constraints:
$$\alpha \in [\alpha_{min}, \alpha_{max}], \quad V \in [V_{stall}, V_{max}]$$
$$|\delta_e| \leq \delta_{e,max}, \quad |\dot{\delta}_e| \leq \dot{\delta}_{e,max}$$

---

This enhanced model now incorporates **state-of-the-art adaptive control techniques** from your lecture materials while maintaining the comprehensive physical modeling we developed earlier. The model is ready for:
- **Simulation** with parameter uncertainties
- **Real-time implementation** with online adaptation
- **Robustness analysis** using Lyapunov methods
- **Comparison** of different adaptive strategies