# Avionic Box – RTG4 FPGA-Based Remote I/O System
Sensor/Actuator fusion example (SPI/I2C/PWM/double buffer)

## Overview

The **Avionic Box** is a modular remote I/O interface designed for space-grade applications, targeting a **Microchip RTG4 radiation-hardened FPGA**. The system interfaces an On-Board Computer (OBC) with multiple sensors and actuators using a command-driven architecture.

The design emphasizes deterministic control, CDC-safe data transfer, and modular scalability.

---

## System Architecture

The architecture consists of the following major subsystems:

- Command Dispatcher Engine
- Sensor Manager Engines (SPI-based)
- PWM Driver Engines
- Asynchronous Double Buffers
- Double Buffer Access Controllers
- OBC SPI Interface (AXI-based integration planned)

---

## Data Flow

### Sensor Acquisition Path

Sensor → Sensor Manager Engine → Async Double Buffer → Sensor DB Access Controller → Command Dispatcher Engine → OBC

### Actuator Control Path

OBC → Command Dispatcher Engine → Actuator DB Access Controller → Async Double Buffer → PWM Driver → Actuator

---

## Command Dispatcher Engine

The Command Dispatcher Engine is the central control unit responsible for:

- Decoding commands from the OBC
- Routing control signals to sensor and actuator engines
- Managing internal DMA engines to read from Double Buffer Access Controllers of sensor double buffers and actuator double buffers.
- Has two AXI4 master ports for the Sensor Double Buffer Access Controller and Actuator Double Buffer Access Controller.
- Initiating data acquisition and actuation sequences

The design is expected to be implemented using a deterministic FSM-based architecture.

---

## Double Buffering Strategy

Each device interface uses an asynchronous double buffer (ping-pong buffer) to:

- Enable safe clock domain crossing
- Decouple producer and consumer timing domains
- Ensure continuous data flow without stalling
- Ensure coherent snapshot of the latest readings from environment 

These buffers isolate clock domains and prevent metastability propagation, aligning with CDC best practices.

---

## Double Buffer Access Controllers

The DB access controllers are responding to the AXI traffic comes to it's AXI4 slave port and direct the memory slots of double buffers as needed to be read and written by the Command Dispatcher Engine.

### Sensor DB Access Controller
- Aggregates sensor data and configuration
- Provides structured access to the OBC

### Actuator DB Access Controller
- Manages actuator command staging and status communication
- Ensures synchronized updates to PWM engines

---

## Sensor Interfaces

Supported SPI-based devices:

- AD4111 (ADC)
- ADS7961 (SAR ADC)
- MAX31889 (Temperature Sensor)

Each device is managed by a dedicated engine and buffer pair.

---

## PWM Driver Engines

- Independent PWM channels
- Buffered control updates
- Deterministic timing behavior

---

## Bus Architecture (Planned)

The system is designed for AXI integration:

- AXI4 : Control plane (PWM Driver Engine register access through the Actuator Double Buffer Access Controller for the Command Dispatcher)
- AXI4 : Data movement (Sensor Double Buffer Access Controller Interfacing with the Command Dispatcher)

---

## Design Principles

- Modularity
- Scalability
- CDC Safety
- Determinism
- Data Integrity (ECC/SECDED)
- Separation of control and data paths

---

## Directory Structure

rtl/
- async_reset_synchronizer/
- command_dispatcher_engine/
- double_buffer_access_controllers/
- double_buffer/
- ecc/
- pwm_driver_engine/
- sensor_managers/
- spi_modules/
- top/

sim/
- async_reset_synchronizer_tb/
- avionics_box_system_tb/
- command_dispatcher_engine_tb/
- double_buffer_tbs/
- ecc_tbs/
- pwm_driver_engine_tb/
- sensor_managers_tb/
- spi_modules_tbs/

---

## Simulation

The simulation environment includes subsystem-level validation of:

- Sensor acquisition - under development
- Buffer behavior    - under development
- Command execution - under development 
- spi_module_tbs - demostration level

---

## Target Device

- FPGA: Microchip RTG4
- Application: Space-grade avionics

---

## Future Enhancements

- Full AXI integration
- UVM-based verification
- CDC/RDC formal verification
- Fault tolerance (TMR, scrubbing)
- BIT (Built In Test Implementation)

---

## Author

Sachith Rathnayake
GitHub: https://github.com/LordSach
Email: sahith.rathnayake.92@gmail.com

---

## License

To be defined

