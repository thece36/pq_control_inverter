# pq_control_inverter

microController (μC): STM32F429ZI-Discovery
IDE: μVision V5.13.0.0

Code includes: 
- essential initializations of TIMERs, GPIOs, ADCs, DMA
- DQ Transformation and a digital 3-Phase PLL, for grid synchronization
- 9 measurements, Vabc, Iabc, Pref, Qref, VCII, by utilizing the 3 available ADCs and the DMA peripheral ( * )
- 5 PI Controllers for driving the system to the desired state
- SVPWM Technique for controlling the IGBTs
- other functions for signal compensation

Repository also includes all the essential libraries.

Full Documentation of the system @ (site-is-coming-soon)

* Vabc: grid voltages
  Iabc: currents exiting our system and entering the grid
  Pref: reference point for active power
  Qref: reference point fot reactive power
  VCII: voltage of the DC Bus - Inverters input
