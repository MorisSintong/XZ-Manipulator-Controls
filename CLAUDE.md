# CLAUDE.md — Instructions for Claude Code / Anthropic Assistants

Please see [VisionSystem/REMEDIATION_PLAN_AI.md](VisionSystem/REMEDIATION_PLAN_AI.md) for the completed `VisionSystem/` remediation baseline, regression guardrails, and completion criteria. See [AGENTS.md](AGENTS.md) for workspace toolchains and embedded STM32H723 (`SKR Octopus Pro V1.1`) workflow conventions.

Do not mark changes complete without running automated programmatic verification tests (`uv run --project VisionSystem python VisionSystem\tests\verify_vision_system.py` and `uv run --project VisionSystem pytest VisionSystem\tests`) covering continuous angle calculation, UART CRC framing, and tracking debounce.
