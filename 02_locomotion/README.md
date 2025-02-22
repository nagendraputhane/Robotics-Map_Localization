## 1. Locomotion Types

### Overview
- **Legged, Aerial, Wheeled:**  
  The type of locomotion is chosen based on application requirements. Each method has its trade-offs in speed and power efficiency (e.g., horsepower per ton).

### Wheeled Locomotion
- **Differential Drive:**  
  - Consists of two independently driven wheels (each powered by its own motor).  
  - Typically includes one or more caster wheels for stability.  
  - By controlling the speed and rotation direction of each wheel, the robot can achieve both translational and rotational movements.  
  - This configuration allows the robot to reach any point in the 2D plane with any desired orientation.

- **Ackermann Drive:**  
  - Common in vehicles; can be front-wheel, rear-wheel, or four-wheel drive.  
  - Only the front wheels are steerable, meaning the vehicle cannot rotate in place.

- **Omnidirectional Drive:**  
  - Utilizes three or four independently driven wheels.  
  - With independent control of each wheel, the robot can move in any direction and rotate simultaneously.

**Note:** In any locomotion architecture, the goal is to control motor speed and direction to achieve a desired pose defined by position (X, Y) and orientation (θ).

---

## 2. Friction and Deformation Effects

### Ideal vs. Real-World Behavior
- **Ideal Case:**  
  - No deformation of the wheel or the floor.  
  - Operating in a vacuum with no energy dissipation.  
  - A single point of contact (P) means that a constant force (F) produces rotation with a constant angular velocity (ω) without deceleration.

- **Real-World Conditions:**  
  - **Deformation:** Both the wheel and the ground deform under load. This deformation can be modeled as many small springs.
  - **Force–Compression Relationship:**  
    - In an ideal elastic spring, force increases linearly with compression, and the work done (the area under the force vs. displacement curve) is fully recoverable.  
    - In reality, the relationship is nonlinear (often parabolic): initially, a small force compresses the material, but further compression requires disproportionately higher force.
  - **Energy Loss (Hysteresis and Rolling Friction):**  
    - The area between the compression and decompression curves represents energy lost due to hysteresis.  
    - This lost energy is converted into heat or causes permanent deformation.  
    - **On Soft Grounds (e.g., sand):**  
      - Both the wheel and the ground deform.  
      - The ground’s "spring" elements do not fully recover their shape, so the energy used for compression is not recovered, resulting in significant inefficiency.

---

## 3. Robot Modeling in ROS 2

### Workflow Overview
1. **Model the Robot:**  
   Define the structure, components, and their interconnections.
2. **Visualize the Model:**  
   Use tools to display the robot's structure.
3. **Simulate the Behavior:**  
   Run the model in a physics engine to test interactions.
4. **Debug and Iterate:**  
   Refine the model and control strategies based on simulation results.

### URDF (Unified Robot Description Format)
- **Purpose:**  
  Defines the robot’s structure using XML.
- **Key Elements:**
  - `<robot> ... </robot>`: Root element.
  - `<link>`: Represents a component of the robot, establishing its reference frame and properties.  
    - **Sub-elements:**  
      - `<visual>`: Defines the appearance (e.g., mesh or 3D model).  
      - `<collision>` and `<inertial>`: Specify physical properties for simulation.
  - `<joint>`: Connects two links in a tree-like structure.  
    - Each joint specifies a parent link (e.g., `base_footprint`, representing the contact area with the ground) and a child link (e.g., `base_link`).  
    - Joint types include **fixed** or **movable** (with defined rotation axes and limits).

### RViz 2
- **Purpose:**  
  A visualization tool for ROS 2.
- **Functionality:**  
  - Displays messages published on ROS topics in real time.  
  - Supports plugins to visualize maps, sensor data, video streams, and more using a publisher-subscriber model.

### Parameters in ROS 2
- ROS 2 packages often expose parameters to fine-tune software behavior.  
- Adjusting these parameters allows the system to be optimized for specific applications.

---
