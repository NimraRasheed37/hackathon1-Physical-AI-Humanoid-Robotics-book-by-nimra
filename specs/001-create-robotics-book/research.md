# Research: Testing Framework for Python and ROS 2

## Decision

We will use `pytest` as the primary testing framework for the Python code examples in the book. For ROS 2 packages, we will use the built-in `ament_pytest` framework, which integrates `pytest` with the ROS 2 build system.

## Rationale

- **`pytest`**: It is a mature, feature-rich, and widely-used testing framework in the Python community. Its simple syntax, powerful fixture model, and extensive plugin ecosystem make it an excellent choice for teaching testing concepts to beginners and intermediate learners.
- **`ament_pytest`**: This is the standard testing framework for ROS 2 Python packages. It allows us to write tests in `pytest` style and run them as part of the normal ROS 2 build and test process. This provides a seamless experience for readers who are learning ROS 2 development.

## Alternatives Considered

- **`unittest`**: This is the built-in Python testing framework. While it is part of the standard library, its syntax is more verbose and less intuitive than `pytest`, especially for beginners.
- **`nose2`**: Another popular Python testing framework. However, `pytest` has a larger and more active community.
