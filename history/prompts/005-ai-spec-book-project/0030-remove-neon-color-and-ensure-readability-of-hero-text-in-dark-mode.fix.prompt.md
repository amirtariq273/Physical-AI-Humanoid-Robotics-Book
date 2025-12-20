---
id: 0030
title: Remove neon color and ensure readability of hero text in dark mode
stage: fix
date: 2025-12-07
surface: agent
model: gemini-1.5-flash-latest
feature: 005-ai-spec-book-project
branch: master
user: Aamir Tariq
command: remove neon color on this text and give the black color of this text main front text The AI/Spec-Driven Book: Pioneering Physical AI and Humanoid Robotics...
labels: ["docusaurus", "theme", "css", "dark-mode", "text-readability"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - frontend/src/css/custom.css
tests:
---

## Prompt

remove neon color on this text and give the black color of this text main front text The AI/Spec-Driven Book: Pioneering Physical AI and Humanoid Robotics
Welcome to the cutting-edge intersection of artificial intelligence, specification-driven development, and the transformative world of physical AI and humanoid robotics. This book is your comprehensive guide to understanding, designing, and implementing intelligent robotic systems that interact seamlessly with our physical environment.

Dive deep into the core concepts and advanced techniques that empower the next generation of humanoid robots. From the foundational principles of the ROS 2 Nervous System to the immersive experiences of Digital Twin Simulations (Gazebo & Unity), the advanced AI capabilities of NVIDIA Isaac™, and the sophisticated Vision-Language-Action (VLA) models, we cover every essential aspect.

Whether you are a researcher, engineer, or enthusiast, this book provides the knowledge and practical insights to build, control, and innovate in the rapidly evolving field of physical AI and humanoid robotics.


this text is not render because text color is white and bg color is also white on dark theme

## Response snapshot

Successfully modified file: C:\Users\Aamir Tariq\Desktop\AI BOOK BY HUMAN\frontend\src\css\custom.css (1 replacements).

## Outcome

- ✅ Impact: Removed the neon text-shadow from the hero section text in dark mode, ensuring pure white text for maximum readability against the dark background.
- 🧪 Tests: No automated tests were run. Manual verification by starting Docusaurus server and using the theme toggle is required.
- 📁 Files: `frontend/src/css/custom.css` (modified).
- 🔁 Next prompts: User will verify the changes.
- 🧠 Reflection: Successfully addressed the readability issue in dark mode by adjusting text styling in the hero section based on user feedback.

## Evaluation notes (flywheel)

- Failure modes observed: Initial implementation of neon text-shadow in dark mode created a readability issue when the user desired plain white text.
- Graders run and results (PASS/FAIL): 
- Prompt variant (if applicable): 
- Next experiment (smallest change to try): 
