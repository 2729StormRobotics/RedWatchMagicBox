# MagicBox Red Watch

The **MagicBox** is a portable FRC test bench designed for rapid prototyping, motor burn-in, and sensor verification. This repository contains a "Universal" robot project that allows you to swap hardware and test components without redeploying code.

## Quick Start

1. **Deploy:** Standard WPILib deploy to the MagicBox RoboRIO.
2. **Dashboard:** Open Shuffleboard or SmartDashboard. Look for the `MagicBox/` tab.
3. **Configure:** * Set the **Target CAN ID** on the dashboard.
   * Select your **Motor Model** (Vortex, NEO, or NEO 550).
   * Click the **RE-INIT HARDWARE** button to apply changes.

## Controls (Xbox Controller)

* **Left Joystick (Y-Axis):** Manual motor power (Percent Output).
* **Right Trigger:** Speed Scaler.
  * *Release:* 50% max power for precision.
  * *Full Pull:* 100% max power for high-speed testing.
* **A Button (Hold):** PID Tuning Mode (requires dashboard setpoints).
* **B Button (Press):** Emergency Motor Stop.

## Key Features

### 1. Generic Motor Subsystem
Supports swapping between **NEO Vortex**, **Standard NEO**, and **NEO 550**.
* **Safety Built-in:** Automatically applies current limits based on the motor selected (e.g., 20A for the fragile NEO 550s).
* **Live Telemetry:** Monitors current draw (Amps), Temperature (C), and Encoder position in real-time.

### 2. Sensor Hub
The MagicBox acts as a universal "Logic Probe" for FRC sensors.
* **DIO Ports (0-9):** Monitors all digital headers. Great for testing Limit Switches, Beam Breaks, and Hall Effect sensors.
* **Analog Ports (0-3):** Displays raw voltage (0-5V). Use this for Potentiometers or Ultrasonic sensors.

### 3. Live PID Tuning
Tune your mechanisms on the bench before the robot is even built.
* Type `kP`, `kI`, `kD`, and `Setpoint` values into the dashboard.
* Hold the **A Button** to run the motor using those constants.

## Hardware Wiring Map

To keep testing consistent, the team should follow this wiring standard for the MagicBox:

| **Component** | **Port Type** | **Standard ID / Channel** |
| :--- | :--- | :--- |
| Test Motor | CAN (PDH) | ID 20 |
| Beam Break | DIO | Channel 0 |
| Limit Switch | DIO | Channel 1 |
| Potentiometer | Analog | Channel 0 |

## 🏗️ Development Notes

* **API Version:** 2025 WPILib / REVLib.
* **Requirement:** Ensure the **REVLib** vendor library is installed via the WPILib menu.
* **Adding Sensors:** New sensors are automatically detected by the `SensorHub` subsystem as long as they are plugged into the standard RIO headers.

---