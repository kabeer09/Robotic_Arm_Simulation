# Robotic_Arm_Simulation
# 🤖 Vision-Based Robotic Sorting Simulation using PPO | PyBullet + RL

This project demonstrates a vision-guided pick-and-place task using a simulated KUKA IIWA robotic arm in PyBullet. The robot detects the color of balls (red or blue), picks them up, and places them into their respective color-coded bins using Proximal Policy Optimization (PPO) from Stable-Baselines3.

---

## 📌 Features

- ✅ Real-time robotic simulation with PyBullet
- ✅ Color-based object detection and sorting
- ✅ Pick-and-place logic using inverse kinematics
- ✅ Manual keyboard control for debugging
- ✅ Integration with PPO Reinforcement Learning (Stable-Baselines3)
- ✅ Compatible with OpenAI Gym interface

---

## 🛠️ Tech Stack

| Tool | Usage |
|------|-------|
| Python | Programming Language |
| PyBullet | Physics-based robotic simulation |
| Stable-Baselines3 | PPO reinforcement learning algorithm |
| NumPy | Numerical operations |
| Gym | Environment compatibility |


├── models/
│   └── ppo_sorting_model.zip    # Saved trained model (generated after training)
│
└── README.md
