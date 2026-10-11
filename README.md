# Bidirectional DC-DC Converter for a BESS Interface

Modelling and control of a synchronous half-bridge converter between a 48 V battery and a 96 V DC bus. **Discharge (boost)** is voltage-controlled and has a lightly damped, non-minimum-phase plant. **Charge (buck)** is current-controlled. This is the power-electronics companion to my battery-side SOC estimation project.

All component values are **assumed** (no hardware was measured), and the models are simulations only. See [`docs/WRITEUP.md`](docs/WRITEUP.md) for the full write-up, including the design iterations.

## Results

| Direction | Controller | Key result |
|---|---|---|
| Discharge (boost), 14.4 Ω → 9.6 Ω load step | PI voltage loop, Kp = 2e-4, Ki = 0.2 (PM ≈ 91°, GM ≈ 8 dB) | 6.6 V dip, ~19–24 ms settling, zero steady-state error |
| Charge (buck), 0 → 20 A reference | PI current loop (~2000 rad/s, PM ≈ 77°) + bus-voltage feedforward | 0.8 ms rise, ~0.9 A overshoot, 0.05 A error in an 8% bus sag |
| Charge → discharge, +20 A → −10 A | Same current loop, no mode switch | 2.2 A overshoot on a 30 A step |

Open-loop boost behaviour that motivates the control design: Q ≈ 5.9, resonance near 604 rad/s, RHP zero at 3600 rad/s, and about 75% overshoot on a duty step.

![boost](docs/figures/overview.png)
![switched](docs/figures/switched_overview.png)
![buck](docs/figures/buck_overview.png)

## What each script does

| Script | Content |
|---|---|
| `src/params.py` | Converter and controller parameters |
| `src/converter.py` | Averaged boost model: open-loop step, linearisation and Bode analysis, PI design, nonlinear load-step simulation |
| `src/switched.py` | Switched (50 kHz) boost model with inductor DCR and capacitor ESR, same PI gains |
| `src/buck.py` | Charge-direction current loop on the switched model, including current reversal and a bus sag |

## Run

    pip install -r requirements.txt
    python src/converter.py
    python src/switched.py
    python src/buck.py

Each script prints its metrics and writes its figure to `docs/figures/`.

## Design findings worth knowing
- The first boost gains (Kp = 1e-3, Ki = 1.0) were unstable on this plant: the loop gain crosses 0 dB near the resonance peak. The gains were reduced until the gain at the resonance stayed below unity.
- In the current loop, cancelling the plant pole with the PI zero gave a clean reference step but a 25 ms recovery tail after a bus sag. Moving the zero up and adding bus-voltage feedforward fixed it.

## Limitations
- Assumed component values, ideal battery source behind a resistance, stiff bus in charge mode.
- The handover between voltage-regulating discharge and current-regulating charge is not modelled.
- Next: cascaded voltage/current loops for discharge, a CC-CV charge profile, mode handover logic.

## License
MIT
