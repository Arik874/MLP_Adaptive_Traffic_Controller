<img width="1173" height="870" alt="image" src="https://github.com/user-attachments/assets/0c985792-276b-4bcc-a2b9-f069ded0682d" />

Adaptive Traffic Control System : Embedded Hardware

Made for the Coursework requirement of MKEC - Advanced Microprocessor Design [Group Assignment]

An intelligent, bare-metal Adaptive Traffic Control System (ATCS) deployed on an STM32F446RE Cortex-M4 microcontroller. This system replaces traditional, rigid time-based traffic lights with a TinyMLP Neural Network Inference Engine that dynamically calculates optimal green-light durations in real-time, executing constant-time matrix math directly on the hardware FPU.


Architecture & Traffic Philosophy

-> The "Shibuya Philosophy" Implementation

Zero Multi-Directional Conflicts: Enforces an enhanced 4-phase sequential control (N-S Active ➔ All Red Clearance ➔ E-W Active ➔ All Red Clearance).

Physics & Sensor Constraint: IR sensors are placed 42.3 meters upstream. Assuming a 50 km/h approach speed and a 1.0s reaction time, this creates a safe stopping dilemma zone, granting the STM32 ample time to calculate AI phase extensions before sudden braking is required.

-> 3-Layer Fail-Safe Architecture

Layer 1 (The Safety Cage): Deterministic Embedded C logic strictly enforces an absolute minimum (5s) and maximum (30s) green time to prevent system gridlock, overriding the AI if extreme edge-cases occur.

Layer 2 (Adaptability): Real-time monitoring of active-low IR sensors with a 3-tick software debounce filter to estimate true queue density.

Layer 3 (Intelligence): The TinyMLP Edge AI runs non-linear multivariate optimization, avoiding the "combinatorial explosion" and CPU branch prediction penalties typical of rigid if-else decision trees.

--> Bare-Metal Hardware Design

-> Atomic Output Application (BSRR)

Standard libraries (like HAL_GPIO_WritePin) create read-modify-write hazards and unpredictable branching. This architecture writes directly to the Bit Set/Reset Register (BSRR), flipping up to 12 traffic LEDs simultaneously in a single, thread-safe, atomic clock cycle.

-> Dual Hardware Timers & Determinism

Instead of blocking HAL_Delay(), the system utilizes true hardware multitasking:

TIM2 (100ms Tick): Triggers the master state acquisition and Edge AI inference at a strictly deterministic rate.

TIM6 (2ms Tick / 500Hz): Drives the Persistence of Vision (POV) multiplexing for the 7-segment display. By multiplexing, we compress a 28-pin requirement down to just 11 GPIO pins (PC6-12, PB6-9).

-> System Watchdog (IWDG)

A dedicated hardware watchdog runs continuously on an independent clock. Programmed with a 4095 reload value, it provides continuous stability validation, ensuring an immediate system reboot if a physical or cosmic anomaly hangs the CPU.

-> Hardware Requirements & Economics

Microcontroller: STM32F446RE Nucleo-64 (ARM Cortex-M4 with FPU)

Sensors: 4x Standard IR Obstacle Detection Modules (Active-Low)

Display: 1x 4-Digit 7-Segment Display (Multiplexed)

Cost Efficiency: Utilizes ultra-low-cost IR sensors (RM 2 - RM 5) while achieving advanced traffic adaptability, eliminating the immediate need for expensive cloud-vision cameras.

-> How to Build and Flash

Clone the repository:

git clone [https://github.com/arik874/MLP_Adaptive_Traffic_Controller.git](https://github.com/arik874/MLP_Adaptive_Traffic_Controller.git)


Open the Project:
Open the root directory using STM32CubeIDE or VS Code (with the STM32 extension).

Compile:
Build the project. Ensure the Hardware FPU is enabled in your compiler flags (-mfpu=fpv4-sp-d16 -mfloat-abi=hard).

Flash:
Connect your STM32 via USB and flash the compiled binary.

-> Retraining the Neural Network

The repository includes the Python training script used to generate the deterministic Edge AI model.

Navigate to the training directory.

Install dependencies:

pip install numpy scikit-learn


Then run the generator:

python train_traffic_mlp.py


The script outputs raw C-arrays for the network's weights and biases. Copy the terminal output and paste it directly into Core/Src/traffic_mlp_weights.c, then recompile the STM32 project.

-> Future Scalability

The current bare-metal architecture serves as the foundation for broader Smart City integration:

Dual-Sensor Arrays: Upgradable for precise (±1 km/h) vehicle speed estimation.

IoT Connectivity: Ready for lightweight telemetry transmission for remote diagnostics.

Macro-Analytics: Cloud-connected intersection data to map and predict city-wide traffic trends.
