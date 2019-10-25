# FOSSASAT-2 Software Requirements
The purpose of this document is to outline all requirements that are to be fulfilled by the on-board program of FOSSASAT-2. All requirement level keywords (MUST, SHALL, etc.) are to be interpreted as per [RFC 2119](https://www.ietf.org/rfc/rfc2119.txt).

Each Software Requirement SHALL be referred to by section name, followed by requirement number. For example, SYSR1 refers to the first requirement in the System section.

## System (SYSR)
1. Satellite MUST remain inactive during launch.
2. Satellite MUST start up following jettison and a delay period.
3. Upon startup, satellite SHALL be initialized into known state.
4. Correct initialization of all components SHOULD be checked.
5. Satellite MUST keep track of the number of restarts.
6. Results of all operations with external components (e.g. sensors, external storage, drivers etc.) SHOULD be checked.
7. All communication with external components SHALL be constrained via time limit.
8. Battery charging MUST be disabled when temperature drops below specified limit.
9. Satellite MUST enter low power mode when battery voltage drops below specified limit.
10. Satellite MUST be able to correctly measure battery voltage, temperature, charging voltage and charging current.
11. Satellite MUST be able to correctly measure all other environmental properties.
12. Power and radio configuration MUST persist during restarts.
13. Satellite MUST reset external watchdog timer every second.
14. Satellite MUST be put to sleep mode when no other task is running.
15. Satellite MUST remain inactive during ground handling.
16. Satellite MUST be able to adjust its duty cycle depending on battery voltage.
17. Satellite MUST be able to charge battery before deploying if under a certain voltage.
18. Satellite MUST heat battery depending on temperature range outside limits and spare power available.
19. Satellite SHOULD implement basic radiation corruption mitigation such as memory validation and/or correction.
20. Satellite MUST put components to sleep when not in use (OV2640 FET available).

## Communication (COMR)
1. Satellite MUST be able to communicate using specified protocol, frequencies and modulations.
2. Used communication protocol SHOULD be backwards-compatible with existing FOSSA communication protocol(s).
3. Satellite MUST be able to switch between arbitrary number of specified radio settings, either via command from ground station, or automatically based on internal timer.
4. Satellite MUST be able to disable all transmissions when commanded to do so by ground station.
5. Used communication protocol SHOULD use strong encryption to protect sensitive commands.
6. Reception and execution result of private commands SHOULD be reported to the ground station.
7. Satellite MUST be able to transmit Morse code beacon.
8. Satellite MUST be able to store received messages in non-volatile memory and retransmit them later.

## Observation (OBSR)
1. Satellite MUST be able to control its attitude (Stanford).
1. Satellite MUST be able to correctly measure its attitude and communicate it.
2. Satellite MUST be able to take a photo using the on-board camera.
3. Satellite MUST be able to store at least one photo in non-volatile memory.
4. Satellite MUST be able to transmit a stored or directly taken photo to a ground station.
5. The properties of the photo MUST be changeable via private command from ground station.
6. Satellite MUST be able to store at least one taken and one permanent test photo in non-volatile memory.

## Development and Integration (DINR)
1. All debug-only code MUST be removable from compilation by a single macro.
2. It MUST be possible to wipe all non-volatile storage, both internal and external.
3. Satellite SHOULD output minimal debug information upon every reset, to allow easier integration.
