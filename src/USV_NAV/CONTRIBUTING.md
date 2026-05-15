# Contributing to USV_NAV

Thank you for your interest in contributing! This document explains the process for contributing to the USV_NAV repository in a simple, professional way.

## Table of Contents

- [Purpose](#purpose)
- [How to Contribute](#how-to-contribute)
- [Branching & Workflow](#branching--workflow)
- [Development Setup](#development-setup)
- [Code Style & Quality](#code-style--quality)
- [Commit Messages](#commit-messages)
- [Pull Requests](#pull-requests)
- [Reporting Issues](#reporting-issues)
- [Continuous Integration & Tests](#continuous-integration--tests)
- [License](#license)
- [Maintainers & Contact](#maintainers--contact)

## Purpose

This file documents how external contributors and project members should collaborate on the USV_NAV repository. It aims to set clear expectations about issues, code contributions, reviews, and testing.

## How to Contribute

1. **Find or open an issue.** If you want to add a feature or fix a bug, first check existing issues. If none match, open a new issue describing the problem or proposal.
2. **Create a branch.** Fork the repository (if you are not a member), then create a branch with a descriptive name (see Branching & Workflow).
3. **Work locally.** Make small, focused changes. Ensure your code builds and tests pass locally.
4. **Open a pull request.** Create a PR against `main` (or `develop` if maintained). Add a clear title and description and reference the issue number.

## Branching & Workflow

- `main` — Stable release-ready code.
- `develop` — Integration branch for the next release (optional).
- `feature/NAME` — Use for new features.
- `fix/ISSUE-NUMBER` — Use for bug fixes.

Always branch from the latest `develop` (or `main` if `develop` is not used). Keep PRs small and focused.

## Development Setup

Short instructions for getting started locally:

```bash
mkdir -p ~/usv_nav_ws/src
cd ~/usv_nav_ws/src
git clone https://github.com/USV_NAV/USV_NAV.git
```

```bash
cd ~/usv_nav_ws
colcon build --merge-install
```

```bash
source ~/usv_nav_ws/install/setup.bash
```

Add any environment or dependency notes specific to your machine in the issue or PR.

## Code Style & Quality

- Follow existing project conventions (ROS2/Humble patterns, Python style for scripts).
- Use linters and formatters where appropriate (e.g., `clang-format` for C++, `black`/`ruff` for Python).
- Keep code readable, well-documented, and accompanied by tests when feasible.

## Commit Messages

Use concise, descriptive commit messages. A common pattern is:

```
<type>(scope): short description

Longer description (if necessary).
```

Examples of types: `feat`, `fix`, `docs`, `chore`, `test`, `refactor`.

## Pull Requests

- Link the related issue (e.g., `Closes #123`).
- Describe what changed and why.
- List manual and automated tests performed.
- Keep PRs focused and small; smaller PRs are reviewed faster.
- Address review comments promptly.

## Reporting Issues

Provide the following when filing an issue:

- Short descriptive title
- Steps to reproduce
- Expected vs actual behavior
- Relevant logs or error messages
- Environment details (OS, ROS/Gazebo versions)

Use labels where possible (bug, enhancement, documentation).

## Continuous Integration & Tests

Contributions should include tests where practical. The repository may use CI to run builds and tests on PRs — ensure your changes pass CI.

## License

By contributing, you agree that your contributions will be licensed under the repository's Apache 2.0 license.

## Maintainers & Contact

For questions, open an issue or contact the maintainers listed in `README.md`.

---

Thank you for helping improve USV_NAV — we appreciate your time and contributions!