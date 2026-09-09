# Ungrounded Haptics Prior Art for a Drone-Mounted Energy Absorber

## Search record

- Query family: `ungrounded haptic force feedback`, `wearable kinesthetic haptic reaction force`, `aerial haptic drone force feedback`, `encountered-type haptic ungrounded`
- Year range: 1995–2026
- The required paper-search CLI was run against Semantic Scholar, OpenAlex, arXiv, OpenReview, Crossref, and DBLP. The first run failed with DNS resolution errors in the sandbox. A network-enabled retry did not return within the bounded working interval and was interrupted. The table below is therefore a web-verified fallback corpus rather than an API-complete corpus; citation counts are included only where the retrieved source reported them.

## Verified relevant papers

| # | Title | Year | Venue | Citations reported by source | Relevance |
|---:|---|---:|---|---:|---|
| 1 | [TorqueBAR: An Ungrounded Haptic Feedback Device](https://www.cs.ubc.ca/labs/lci/papers/docs2003/icmi2003-swindells-torquebar.pdf) | 2003 | ICMI | — | Moves an internal mass to change center of mass and render inertial torque; representative internal-momentum device. |
| 2 | [Non-grounding Force Display Utilizing Nonlinearity of Human Perception](https://doi.org/10.18974/tvrsj.11.1_47) | 2006 | TVRSJ | 10 | Uses asymmetric acceleration and perceptual nonlinearity to create a directional-force illusion without a reaction base. |
| 3 | [Wearable Haptics: Taxonomy and Design Guidelines for Wearable Feedback Interfaces](https://sirslab.dii.unisi.it/papers/2013/Prattichizzo.ToH.2013.Haptics.Fin.pdf) | 2013 | IEEE Transactions on Haptics | — | Explains the grounded/body-grounded/wearable trade-off and the unavoidable counterforce on the body. |
| 4 | [A Non-grounded and Encountered-type Haptic Display Using a Drone](https://doi.org/10.1145/2983310.2985746) | 2016 | SUI | 4 | Early drone-as-moving-base encountered haptics; measures contact pressure and uses rotor airflow to stabilize a light end-effector. |
| 5 | [Wolverine: A Wearable Haptic Interface for Grasping in Virtual Reality](https://shape.stanford.edu/research/wolverine/Wolverine_IROS_2016.pdf) | 2016 | IROS | — | Brake-based locking sliders resist relative finger motion; over 100 N holding force with low actuation energy. |
| 6 | [Shifty: A Weight-Shifting Dynamic Passive Haptic Proxy](https://doi.org/10.1109/TVCG.2017.2656978) | 2017 | IEEE TVCG | — | Changes internal mass distribution to alter passive inertial cues; does not create sustained net translation force. |
| 7 | [Grabity: A Wearable Haptic Interface for Simulating Weight and Grasping in Virtual Reality](https://doi.org/10.1145/3126594.3126599) | 2017 | UIST | — | Combines a unidirectional brake with asymmetric skin deformation; separates rigidity rendering from weight illusion. |
| 8 | [HapticDrone: Initial Example for 1D Haptic Feedback](https://doi.org/10.1145/3131785.3131821) | 2017 | VRST Adjunct | — | Controls drone thrust to produce measured 1D forces; reports up to 1.53 N upward and 2.97 N downward. |
| 9 | [HapticDrone: Example of Stiffness and Weight Rendering](https://doi.org/10.1109/HAPTICS.2018.8357197) | 2018 | IEEE Haptics Symposium | 42 | Identifies force versus thrust command and closes the force loop while tracking the hand and drone. |
| 10 | [Thor's Hammer: An Ungrounded Force Feedback Device Utilizing Propeller-Induced Propulsive Force](https://doi.org/10.1145/3173574.3174099) | 2018 | CHI | 153 | Exchanges momentum with surrounding air; reports up to 4 N arbitrary-direction force rather than relying on internal reaction only. |
| 11 | [DextrES: Wearable Haptic Feedback for Grasping in VR via a Thin Form-Factor Electrostatic Brake](https://ait.ethz.ch/dextres) | 2018 | UIST | — | An under-8 g electrostatic clutch provides electronically controlled friction and up to 20 N holding force per finger. |
| 12 | [Effects of Different Hand-Grounding Locations on Haptic Performance](https://arxiv.org/abs/1906.00430) | 2019 | IEEE Robotics and Automation Letters | — | Shows that where the counterforce is closed through the body changes perception, comfort, and discrimination. |
| 13 | [Beyond The Force: Using Quadcopters to Appropriate Objects and the Environment for Haptics in Virtual Reality](https://doi.org/10.1145/3290605.3300589) | 2019 | CHI | — | Treats the drone as an encountered haptic carrier and discusses control accuracy, speed, and safety constraints. |
| 14 | [HapticPuppet: A Kinesthetic Mid-air Multidirectional Force-Feedback Drone-based Interface](https://www.dfki.de/en/web/research/projects-and-publications/publication/12778) | 2022 | UIST | — | Multiple drones apply cable forces to the body, making the surrounding air the momentum sink. |
| 15 | [HaptGlove—Untethered Pneumatic Glove for Multimode Haptic Feedback](https://pmc.ncbi.nlm.nih.gov/articles/PMC10477838/) | 2023 | Advanced Intelligent Systems | — | Pressure-controlled pneumatic clutch produces variable stiffness; each PneuClutch is 14 g and the complete glove is 283 g. |
| 16 | [DroneHaptics: Encountered-Type Haptic Interface Using Dome-Shaped Drone for 3-DoF Force Feedback](https://mudassir-awan.github.io/files/DroneHaptics.pdf) | 2023 | IEEE World Haptics | — | Maps thrust to 3-DoF contact force; reports force error below 8.6% and addresses torque from offset end-effectors. |
| 17 | [HapticWhirl: A Flywheel-Gimbal Handheld Haptic Controller](https://pmc.ncbi.nlm.nih.gov/articles/PMC10857638/) | 2023 | IEEE Transactions on Haptics | — | Uses internal angular momentum for continuous torque-like feedback, but not sustained net linear force. |
| 18 | [AirCharge: Amplifying Ungrounded Impact Force by Accumulating Air Propulsion Momentum](https://scholars.lib.ntu.edu.tw/entities/publication/193ae291-8df8-482a-bc05-40112eeda7f4) | 2023 | HCI proceedings | — | Accumulates air-propulsion momentum to increase short impact intensity; useful comparison for impulse budgeting. |
| 19 | [Magnetorheological Fluid-Based Haptic Feedback Damper](https://www.mdpi.com/2076-3417/14/9/3697) | 2024 | Applied Sciences | — | Demonstrates electrically variable fluid damping; relevant to controllable dissipation but not an ultralight flying implementation. |
| 20 | [FlyHaptics: Flying Multi-contact Haptic Interface](https://arxiv.org/abs/2505.02582) | 2025 | arXiv preprint | — | Adds multiple lightweight contact mechanisms to a flying platform and evaluates hover and force consistency. |

## Summary

### Overview

The corpus spans 2003–2025 and covers four mechanisms: internal momentum redistribution, perceptual force illusions, body-grounded brakes/clutches, and drone/propeller systems that exchange momentum with air. The last category is the closest match to a drone-mounted liquid energy absorber.

### Trends

Early work emphasized movable masses, gyroscopic effects, and asymmetric acceleration. From 2016 onward, lightweight brakes and clutches became prominent for wearable rigidity feedback, while drones and propellers enabled true external momentum exchange. Recent aerial systems increasingly identify a thrust-to-force model and close the force loop rather than treating the drone as a passive proxy.

### Key themes

1. **Internal momentum and inertia:** movable masses or flywheels create transient force/torque cues but cannot sustain net linear force indefinitely ([1], [6], [17]).
2. **Perceptual asymmetry:** rapid/slow asymmetric motion creates a directional-force illusion with near-zero average physical force ([2], [7]).
3. **Body-grounded braking:** clutches resist relative motion between body segments and can be lightweight and strong, but the reaction remains on the user ([5], [11], [12], [15]).
4. **Momentum exchange with air:** propellers and drones can generate genuine net force because air supplies the external momentum sink ([4], [8]–[10], [13], [14], [16], [20]).
5. **Variable dissipation:** pneumatic, electrostatic, and MR clutches tune impedance while dissipating or storing energy, but do not by themselves cancel system-level impulse ([11], [15], [19]).

### Keywords frequency

| Keyword | Count |
|---|---:|
| haptic / haptics | 20 |
| force feedback | 12 |
| ungrounded / non-grounded | 7 |
| drone / flying / quadcopter | 8 |
| brake / clutch / damper | 6 |

### Most cited by accepted paper

Only three retrieved pages exposed citation counts; absent counts are not treated as zero.

| Rank | Title | Year | Citations |
|---:|---|---:|---:|
| 1 | Thor's Hammer | 2018 | 153 |
| 2 | HapticDrone: Stiffness and Weight Rendering | 2018 | 42 |
| 3 | Non-grounding Force Display Utilizing Nonlinearity of Human Perception | 2006 | 10 |

### Most cited by first author

| Rank | Author | Papers in set | Total reported citations |
|---:|---|---:|---:|
| 1 | Seongkook Heo | 1 | 153 |
| 2 | Muhammad Abdullah | 2 | 42+ |
| 3 | Tomohiro Amemiya | 1 | 10 |

### Recommendations for reading

1. **TorqueBAR (2003):** foundational demonstration of what internal-mass ungrounded feedback can and cannot do.
2. **Wolverine (2016) and DextrES (2018):** best precedents for lightweight brake-based impedance with low actuation energy.
3. **HapticDrone (2018):** most direct reference for identifying and controlling contact force on a flying platform.
4. **Beyond The Force (2019):** useful for safe-to-touch drone design and encountered interaction limitations.
5. **DroneHaptics (2023):** closest recent reference for multidirectional force calibration and cage geometry.

## Design implication for the current concept

A liquid absorber mounted in series with a drone contact surface is a high-frequency passive impedance layer, not a reactionless force sink. The drone rotors are the only component in the proposed system that can exchange net momentum with the environment. A defensible architecture is therefore: series hydraulic damper for collision-energy dissipation, inline force sensing, and low-frequency drone admittance/force control to prevent accumulated impulse and end-stop loading.
