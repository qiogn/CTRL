# Contributing to STM32F103 CTRL Project

Thank you for your interest in contributing to the STM32 CTRL project! This document provides guidelines and instructions for contributing.

## 📋 Code of Conduct

By participating in this project, you agree to maintain a respectful and inclusive environment for everyone.

## 🚀 Getting Started

### Prerequisites
- Familiarity with STM32 microcontroller programming
- Basic knowledge of C programming
- Understanding of stepper motor control principles
- Git version control

### Development Environment Setup
1. **Clone the repository**
   ```bash
   git clone https://github.com/qiogn/CTRL.git
   cd CTRL
   ```

2. **Set up build environment**
   - Install ARM GCC: `sudo apt-get install gcc-arm-none-eabi`
   - Install CMake 3.22+
   - Install Python 3.7+

3. **Verify setup**
   ```bash
   cmake -S . -B build
   cmake --build build
   ```

## 🛠️ Development Workflow

### 1. Branch Strategy
- `master` - Production-ready code
- `develop` - Integration branch for features
- `feature/*` - New features
- `bugfix/*` - Bug fixes
- `docs/*` - Documentation updates

### 2. Creating a Feature
```bash
# Create and switch to feature branch
git checkout -b feature/your-feature-name

# Make your changes
# ...

# Commit with descriptive message
git commit -m "feat: add new motor control algorithm

- Implement PID controller for smoother motion
- Add configuration parameters for tuning
- Update documentation with usage examples"

# Push to remote
git push origin feature/your-feature-name
```

### 3. Commit Message Convention
We follow [Conventional Commits](https://www.conventionalcommits.org/):
- `feat:` New feature
- `fix:` Bug fix
- `docs:` Documentation changes
- `style:` Code style changes (formatting, etc.)
- `refactor:` Code refactoring
- `test:` Adding or updating tests
- `chore:` Maintenance tasks

### 4. Pull Request Process
1. Ensure your branch is up to date with `develop`
2. Run all tests locally
3. Update documentation if needed
4. Create a PR with clear description:
   - What changes were made
   - Why they were made
   - Testing performed
   - Screenshots/videos if applicable

## 💻 Coding Standards

### C Code Style
- Follow [MISRA C](https://www.misra.org.uk/) guidelines where applicable
- Use descriptive variable names
- Comment complex algorithms
- Keep functions small and focused

### Example
```c
/**
 * @brief Calculate motor step position based on vision input
 * 
 * @param x_coordinate X coordinate from vision system (0-639)
 * @param y_coordinate Y coordinate from vision system (0-359)
 * @return MotorPosition_t Calculated motor positions
 */
MotorPosition_t calculate_motor_position(uint16_t x_coordinate, uint16_t y_coordinate)
{
    // Validate input range
    if (x_coordinate >= VISION_MAX_X || y_coordinate >= VISION_MAX_Y) {
        return get_default_position();
    }
    
    // Calculate position (example algorithm)
    MotorPosition_t position;
    position.x_steps = (int16_t)((x_coordinate - VISION_CENTER_X) * STEPS_PER_PIXEL);
    position.y_steps = (int16_t)((y_coordinate - VISION_CENTER_Y) * STEPS_PER_PIXEL);
    
    return position;
}
```

### Documentation
- Document public APIs with Doxygen-style comments
- Update README.md for user-facing changes
- Add comments for complex logic
- Keep documentation in sync with code

## 🧪 Testing

### Test Requirements
- New features must include tests
- Bug fixes should include regression tests
- Maintain or improve test coverage

### Running Tests
```bash
# Build tests
cmake -S . -B build -DBUILD_TESTS=ON
cmake --build build

# Run tests
cd build && ctest --output-on-failure
```

## 🐛 Reporting Issues

### Bug Reports
When reporting bugs, include:
1. **Description** - Clear explanation of the issue
2. **Steps to Reproduce** - Step-by-step instructions
3. **Expected Behavior** - What should happen
4. **Actual Behavior** - What actually happens
5. **Environment** - Hardware, software versions
6. **Logs/Screenshots** - Relevant output

### Feature Requests
For new features, describe:
1. **Use Case** - How the feature will be used
2. **Benefits** - Advantages over current implementation
3. **Implementation Ideas** - Suggested approach

## 📖 Documentation Contributions

### Types of Documentation
- **API Documentation**: Doxygen comments in source files
- **User Guides**: Step-by-step tutorials in `/docs`
- **Architecture**: System design documents
- **Hardware**: Schematics and connection guides

### Documentation Standards
- Use clear, concise language
- Include code examples
- Add diagrams for complex concepts
- Keep documentation up to date

## 🔧 Review Process

### What Reviewers Look For
- Code quality and readability
- Adherence to coding standards
- Test coverage
- Documentation updates
- Performance considerations
- Security implications

### Review Timeline
- Small fixes: 1-2 business days
- Features: 3-5 business days
- Large changes: 1 week

## 📄 License

By contributing, you agree that your contributions will be licensed under the project's MIT License.

## 🙏 Acknowledgments

Thank you for contributing to open source! Your help makes this project better for everyone.

## 📞 Questions?

- Open an issue for technical questions
- Check existing documentation first
- Join discussions in GitHub Discussions