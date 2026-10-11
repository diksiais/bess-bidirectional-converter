"""Assumed converter parameters (original values were lost; chosen to reproduce
the earlier findings: resonance ~598 rad/s, RHP zero ~3600 rad/s, ~100 ms settling)."""
Vin = 48.0      # V, battery side
D0  = 0.5       # nominal duty -> Vo = Vin/(1-D0) = 96 V
L   = 1.0e-3    # H
C   = 680e-6    # F
R0  = 14.4      # ohm nominal load (~640 W)
Vref = Vin/(1-D0)
Kp, Ki = 2e-4, 0.2   # redesigned: original 0.001/1.0 is unstable on this plant (GM<0)

# --- Buck / charge-mode additions (step 6) ---
Vbat = 48.0      # V, battery treated as ideal source behind Rb
Rb   = 50e-3     # ohm, battery internal resistance (assumed)
rL   = 20e-3     # ohm, inductor DCR (assumed)
fsw  = 50e3
Wc_i = 2000.0    # rad/s, target current-loop crossover
