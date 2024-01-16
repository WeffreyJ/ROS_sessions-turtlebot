# ME571 - Intro to Robotics Technology

Welcome to ME571 - Intro to Robotics Technology! This repository contains the template code, setup files, and resources needed for your labs involving the Turtlebot3 Waffle with openManipulator.

## Table of Contents
- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Getting Started](#getting-started)
- [Repository Structure](#repository-structure)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

---

## Introduction

In this course, you will get hands-on experience with the Turtlebot3 Waffle and openManipulator. This README provides guidance on setting up your development environment, accessing the template code, and getting started with the labs.

## Prerequisites

Before you begin, ensure you have the following installed:
- [Git](https://git-scm.com/)
- [Docker](https://www.docker.com/)
- [Visual Studio Code (VSCode)](https://code.visualstudio.com/)
- [Remote - Containers extension for VSCode](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

## Getting Started

1. **Clone the Repository**: Open your terminal and run the following command to clone this repository:
    ```
    git clone https://github.com/<user_name>/Drexel_Intro_labs.git
    ```

2. **Open Repository in VSCode using DevContainer**:
    - Navigate to the cloned repository directory:
        ```
        cd ME571-Intro-to-Robotics
        ```
    - Open VSCode and ensure you have the Remote - Containers extension installed.
    - In the bottom-left corner of the VSCode window, click on the green icon that says "Open a Remote Window".
    - Select "Remote-Containers: Reopen in Container" from the dropdown. This will use the `devcontainer.json` configuration file in the repository to set up the development environment inside a Docker container.

## Repository Structure

- `lab1/`: Contains template code and resources for Lab 1.
- `lab2/`: Contains template code and resources for Lab 2.
- `lab3/`: Contains template code and resources for Lab 3.
- `lab4/`: Contains template code and resources for Lab 4.
- `dockerfile`: Dockerfile for setting up the development environment.
- `devcontainer.json`: Configuration file for the development container in VSCode.
- `README.md`: This README file.

## Usage

1. **Working with VSCode and DevContainer**: Once the container is running, you can start working on your labs directly within VSCode, with the development environment set up inside the container.

## Contributing

We welcome contributions! If you have suggestions or find issues in the template code or documentation, please create a pull request or open an issue in the repository.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

---

Happy coding! If you have any questions or need further assistance, feel free to reach out.
