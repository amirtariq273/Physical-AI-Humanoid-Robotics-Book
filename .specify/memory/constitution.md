<!-- Sync Impact Report:
Version change: (none) -> 0.1.0
Modified principles: (none) -> Technical Accuracy, Consistency, Code Precision, Clarity for Learners, Authoritative Verification
Added sections: Key Standards, Constraints, Success Criteria
Removed sections: (none)
Templates requiring updates:
- .specify/templates/plan-template.md: ✅ updated (application guidance changed, template content unchanged)
- .specify/templates/spec-template.md: ✅ updated (application guidance changed, template content unchanged)
- .specify/templates/tasks-template.md: ✅ updated (application guidance changed, template content unchanged)
Follow-up TODOs: (none)
-->
# Unified Book + RAG Chatbot on Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Technical Accuracy
Technical accuracy across robotics, AI, and simulation frameworks (ROS 2, Gazebo, Unity, NVIDIA Isaac)

### II. Consistency
Consistency across all book modules and chatbot responses

### III. Code Precision
Code-level precision for ROS 2, Python, URDF, Gazebo worlds, Isaac Sim scripts, and VLA pipelines

### IV. Clarity for Learners
Clarity for learners with intermediate Python + AI background

### V. Authoritative Verification
Verification through authoritative sources: ROS 2 docs, Gazebo docs, NVIDIA Isaac SDK, robotics research papers

## Key Standards

- All factual claims MUST reference credible technical documentation or peer-reviewed robotics sources.
- Writing style MUST be Educational, step-by-step, with diagrams and code blocks.
- All code MUST be runnable (Python, ROS 2, FastAPI, Agent SDKs).
- RAG chatbot MUST answer questions **solely from book content** or user-selected text.
- Infrastructure stack MUST align with:
  - OpenAI Agents / ChatKit SDKs
  - FastAPI backend
  - Neon Serverless Postgres
  - Qdrant Cloud (Free Tier)
- Book MUST be structured for Docusaurus and deployable to GitHub Pages.
- Chatbot MUST embed inside the Docusaurus site.

## Constraints

- Book size: Minimum 10–12 chapters (one per module + supporting chapters).
- MUST include the following modules:
  - Module 1: ROS 2 — Nodes, Topics, Services, URDF, Python bridges
  - Module 2: Gazebo + Unity — Simulations, sensors, physics
  - Module 3: NVIDIA Isaac — VSLAM, Nav2, photorealistic simulation
  - Module 4: Vision-Language-Action — Whisper + LLM planners + Action graphs
  - Capstone: Build an Autonomous Humanoid System end-to-end
- All diagrams MUST be textual (Mermaid/ASCII) for GitHub Pages compatibility.
- RAG pipeline MUST support:
  - semantic search
  - document chunking
  - citations from source book pages

## Success Criteria

- Book SUCCESSFULLY deployed on GitHub Pages via Docusaurus.
- RAG chatbot FULLY integrated and functioning on the live site.
- Chatbot ACCURATELY answers questions based ONLY on book content.
- All modules CONTAIN:
  - correct explanations
  - runnable code examples
  - ROS 2 commands
  - Isaac Sim workflows
  - Gazebo world setups
  - VLA pipeline demos
- Project PASSES:
  - technical verification (robots + AI)
  - code execution validation
  - deployment validation (Docusaurus + GitHub Pages)
  - chatbot functionality test (RAG accuracy)

## Governance

This Constitution supersedes all other practices. Amendments require a documented proposal, team approval, and a migration plan. All PRs/reviews MUST verify compliance. Complexity MUST be justified.

**Version**: 0.1.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07