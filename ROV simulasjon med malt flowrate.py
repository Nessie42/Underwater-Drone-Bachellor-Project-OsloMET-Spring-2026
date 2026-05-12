import math
import matplotlib.pyplot as plt

# Fysikk
rho = 1000.0
g = 9.81
dt = 0.1
Cd = 1.2  # Dragkoeffisient for vertikal sylinder med flat bunn + perforert PVC-ramme
A = 0.012 # Areal i fartsretning

# Skrog
V_disp = 0.006
m_skrog = 5.8
m_ballast = 0
V_ballast_maks = 0.0015
m_ballast_maks = rho * V_ballast_maks

# Styring
oensket_dybde = 10
h_ref = 0
N = 60000 # Totale tidssteg for operasjonen
Kp = 0.18
Kd = 20
Ki = 0.0003
e_deadzone = 0.1

# Startbetingelser
h = 0.0
v = 0.0
integrator = 0.0
m_ballast = 0.0
h_ref = 0.0
Q = 0.0

#Logging av punkter til graf
x = []
y = []

# maalt flowrate
Q_maalt = 0.0006 / 1000   # Konverterer fra L/s til m^3/s

for k in range(N):

    # Mission control & setpoint shaping
    t = k * dt # Tid i sekunder = Antall steg * tidssteg

    if t < 5000:
        h_ref = min(oensket_dybde - 0.1, h_ref + 0.1*dt) # Trinnvis oppdatering av referansedybde der jeg clamper til 9.9
    else:
        h_ref = max(0.0, h_ref - 0.5*dt)

    # Feil
    error = h_ref - h

    #Definere full og tom tank
    full = (m_ballast >= m_ballast_maks)
    tom  = (m_ballast <= 0.0)

    # PID kontroller
    u_raw = Kp * error - Kd * v + Ki * integrator # PID Utslag (raaverdi) 
    u = max(-1, min(1.0, u_raw)) # Hold deg mellom -1 og 1
    if abs(error) < e_deadzone or abs(u) < 0.01: # Ikke gjoer noe hvis vi er innenfor deadzone
        retning = 0
        pwm = 0.0
    else:
        retning = 1 if u > 0 else -1 # Bestemmer retning, dykk/stig. Negative u-verdier betyr stig
        pwm = abs(u) # Virkningsgrad av motor fra 0 til 1

    #Anti wind-up
    blocked = (full and retning == 1) or (tom and retning == -1) # Hvis du proever aa toemme tom tank eller fylle full tank
    stopped = (retning == 0) # Eller hvis vi er inne i deadzone
    metning = (abs(u_raw) > 1.0) or blocked or stopped # Eller hvis raaverdien av utslaget er over 1
    if not metning: # Dropp aa samle opp steady state feil
        integrator += error * dt # Hvis ikke: Samler opp positive og negative steady state feil-bidrag 

    # Ballastsjekk maksimum og minimumsgrenser
    if (full and retning == 1) or (tom and retning == -1): # Skru av pumpa hvis du proever aa toemme tom tank eller fylle full tank
        Q = 0

    # Vanntilfoersel
    else:
        Q = retning * pwm * Q_maalt

    # Ballasttank
    m_ballast += rho * Q * dt
    # Clamp av ballasttank:
    if m_ballast < 0: # Hvis vi har negativ vannvekt saa er den 0
        m_ballast = 0
    elif m_ballast > m_ballast_maks: # Hvis vi har mer enn full, saa er den full
        m_ballast = m_ballast_maks
    m = m_skrog + m_ballast # Total masse

    # Krefter
    F_drag = -0.5 * rho * Cd * A * v * abs(v)
    F = m * g - rho * g * V_disp + F_drag

    # Mekanikk
    a = F / m
    v += a * dt
    h += v * dt

    if h < 0.0 and v < 0.0:
        h = 0.0
        v = 0.0

    # Oppdatere x og y for graf
    x.append(t)
    y.append(h)

# Grafe dybde over tid 

plt.plot(x, y)

plt.title("Dybde over tid")
plt.xlabel("Tid[s]")
plt.ylabel("Dybde[m]")
plt.grid(True)

plt.show()