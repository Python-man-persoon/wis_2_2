# Ziegler-Nichols method calculations for PID tuning

Ku = -100
Tu = 2.1

KP1 = (0.6*Ku)
KI1 = (1.2*Ku)/Tu
KD1 = (3*Ku*Tu)/40

print(f"K_P1: {KP1}, K_I1: {KI1}, K_D1: {KD1}")