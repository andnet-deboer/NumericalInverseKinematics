# Numerical Inverse Kinematics

This project implements **numerical inverse kinematics (IK)** for a 6-DOF UR5 robot using the **Newton–Raphson method**. It builds upon the `IKinBody` function from the *Modern Robotics* library and extends it to produce iteration reports, CSV outputs, and visualizations.

---

Long Iteration             |  Short Iteration
:-------------------------:|:-------------------------:
![Screencast from 2025-10-29 21-03-21 (1)](https://github.com/user-attachments/assets/a0b81e7c-a6c5-4e14-821e-d49d7f2de5e3) |  ![Screencast from 2025-10-29 21-14-35](https://github.com/user-attachments/assets/6c707a71-1d9a-4ce5-b86a-6ebceff92042)

<img width="1199" height="503" alt="image" src="https://github.com/user-attachments/assets/ef0aa115-fe60-4bfa-b952-63a1048eacc3" />


## Overview

The main function, `IKinBodyIterates`, computes the joint configuration that achieves a desired end-effector pose `Tsd` by iteratively minimizing the body twist error.  
Each iteration prints the following information:

- **Iteration number**
- **Joint vector θᵢ**
- **End-effector transformation Tsb(θᵢ)**
- **Error twist Vᵦ**
- **Angular error ∥ωᵦ∥**
- **Linear error ∥vᵦ∥**

The function also stores each iteration’s joint values in a matrix and exports them as a `.csv` file for visualization.

---

## Features

- Implements **Newton–Raphson inverse kinematics** using the body Jacobian.  
- Reports detailed per-iteration values.  
- Supports multiple initial guesses to analyze convergence.  
- Exports results to `.csv` files (`short_iterates.csv` and `long_iterates.csv`).  
- Generates plots of:
  - End-effector position trajectory in 3D.
  - Linear error magnitude vs. iteration count.
  - Angular error magnitude vs. iteration count.

---

## Repository Structure

```
NumericalInverseKinematics/
├── src/                 # Source code implementation
├── tests/               # Unit tests
├── Makefile             # Simplified automation for setup and cleanup
├── setup_env.sh         # Creates virtual environment and installs dependencies
├── requirements.txt     # Python dependencies
├── README.md            # Project documentation (this file)
└── data/
    ├── short_iterates.csv
    └── long_iterates.csv
```

---

## Installation

Clone the repository and set up a virtual environment:

```bash
git clone https://github.com/andnet-deboer/NumericalInverseKinematics.git
cd NumericalInverseKinematics
make setup
```

Then activate your virtual environment:

```bash
source venv/bin/activate
```

---

## Usage

Run the inverse kinematics solver with your desired target configuration:

```bash
python src/ikinbody_iterates.py
```

Example output (truncated):

```
Iteration 3:
joint vector: 0.221, 0.375, 2.233, 1.414
SE(3) end-effector config:
1.000 0.000 0.000 3.275
0.000 1.000 0.000 4.162
0.000 0.000 1.000 -5.732
error twist V_b: (0.232, 0.171, 0.211, 0.345, 1.367, -0.222)
angular error ||omega_b||: 0.357
linear error ||v_b||: 1.427
```

The function will generate `.csv` files in the project directory that log each iteration's joint angles.

---

## Visualization

You can visualize the results in **CoppeliaSim** using the provided UR5 animation scene.  
Each `.csv` file can be loaded to animate the Newton–Raphson process for:
- **Short iterates (2–4 steps to convergence)**
- **Long iterates (10+ steps or divergence)**

---

## Requirements

- Python ≥ 3.8  
- `numpy`, `scipy`, `matplotlib`, `modern_robotics`  
Install all dependencies using:

```bash
pip install -r requirements.txt
```

---

## Results

The following plots are generated automatically:

1. **3D trajectory of the end-effector**
2. **Linear error vs. iteration count**
3. **Angular error vs. iteration count**

These help compare convergence behaviors for different initial guesses.

---

## References

- Lynch, K.M. & Park, F.C., *Modern Robotics: Mechanics, Planning, and Control*, 2017.  
- Northwestern University ME 449: *Robotics and Automation, Assignment 3 (2025).*

---

## License

This project is released under the **MIT License**.  
See the [LICENSE](LICENSE) file for details.
