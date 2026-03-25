# UAM-Route-Comparison-Tool
Engineering project focused on comparing eVTOL and eSTOL aircraft performance across user-defined urban air mobility routes.

<img width="899" height="558" alt="image" src="https://github.com/user-attachments/assets/a5d32d49-e476-496c-8def-f2ea3ef01f14" />

## What it does
- Simulates 3-DOF longitudinal flight dynamics for both vehicle types
- Models hover (momentum theory), transition (Glauert inflow), and cruise (actuator disk) propulsion phases
- Optimizes eVTOL transition trajectory using fmincon 
- Scores the performance of both aircraft types using a weighted system across multiple categories
- Outputs scored recommendation with a generated rationale

## Running Files
To run the app, download all the MATLAB Code files into the same MATLAB directory. Then run the routecomparisontool_exported.m file and the app should display.

For more information, please read the documentation.
  
