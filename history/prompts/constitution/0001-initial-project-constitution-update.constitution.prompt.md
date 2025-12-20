---
id: 0001
title: Initial Project Constitution Update
stage: constitution
date: 2025-12-07
surface: agent
model: gemini-cli-agent
feature: none
branch: master
user: unknown
command: /sp.constitution
labels: ["constitution", "initialization", "governance"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

```text
/sp.constitution

Project: Unified Book + RAG Chatbot on Physical AI & Humanoid Robotics

Core Principles:
- Technical accuracy across robotics, AI, and simulation frameworks (ROS 2, Gazebo, Unity, NVIDIA Isaac)
- Consistency across all book modules and chatbot responses
- Code-level precision for ROS 2, Python, URDF, Gazebo worlds, Isaac Sim scripts, and VLA pipelines
- Clarity for learners with intermediate Python + AI background
- Verification through authoritative sources: ROS 2 docs, Gazebo docs, NVIDIA Isaac SDK, robotics research papers

Key Standards:
- All factual claims must reference credible technical documentation or peer-reviewed robotics sources
- Writing style: Educational, step-by-step, with diagrams and code blocks
- All code must be runnable (Python, ROS 2, FastAPI, Agent SDKs)
- RAG chatbot must answer questions **solely from book content** or user-selected text
- Infrastructure stack must align with:
  - OpenAI Agents / ChatKit SDKs
  - FastAPI backend
  - Neon Serverless Postgres
  - Qdrant Cloud (Free Tier)
- Book must be structured for Docusaurus and deployable to GitHub Pages
- Chatbot must embed inside the Docusaurus site

Constraints:
- Book size: Minimum 10–12 chapters (one per module + supporting chapters)
- Must include the following modules:
  - Module 1: ROS 2 — Nodes, Topics, Services, URDF, Python bridges
  - Module 2: Gazebo + Unity — Simulations, sensors, physics
  - Module 3: NVIDIA Isaac — VSLAM, Nav2, photorealistic simulation
  - Module 4: Vision-Language-Action — Whisper + LLM planners + Action graphs
  - Capstone: Build an Autonomous Humanoid System end-to-end
- All diagrams must be textual (Mermaid/ASCII) for GitHub Pages compatibility
- RAG pipeline must support:
  - semantic search
  - document chunking
  - citations from source book pages

Success Criteria:
- Book successfully deployed on GitHub Pages via Docusaurus
- RAG chatbot fully integrated and functioning on the live site
- Chatbot accurately answers questions based only on book content
- All modules contain:
  - correct explanations
  - runnable code examples
  - ROS 2 commands
  - Isaac Sim workflows
  - Gazebo world setups
  - VLA pipeline demos
- Project passes:
  - technical verification (robots + AI)
  - code execution validation
  - deployment validation (Docusaurus + GitHub Pages)
  - chatbot functionality test (RAG accuracy)
```

## Response snapshot

The project constitution has been successfully updated to version 0.1.0 with initial principles, standards, constraints, and success criteria. No errors were encountered.

## Outcome

- ✅ Impact: Updated project constitution with core principles, standards, constraints, and success criteria.
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated.
- 🔁 Next prompts: Proceed with feature specification based on the new constitution.
- 🧠 Reflection: Successfully updated the constitution. Implemented fallback for PHR creation due to shell script failure.

## Evaluation notes (flywheel)

- Failure modes observed: The `create-phr.sh` script failed because it is a bash script and the environment is PowerShell. Manual PHR creation was performed as a fallback.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Investigate how to execute bash scripts in PowerShell environment or ensure `create-phr.sh` is PowerShell compatible.
