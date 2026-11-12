# Software Setup Guide:
Please use the following as a rough guide to set up your development environment for this project. We primarily develop in Python.

## Create a GitHub account:
if you don't already have one [here](https://github.com/signup)

## Installing Git:
For Mac: No action needed. Open terminal and type `git --version` and make sure it doesn't say `command not found`\
For Windows: Download [Git Bash](https://git-scm.com/downloads) **OR** (recommened if you plan on doing lots of development in various languages or just want to have some fun with linux) [install WSL](https://learn.microsoft.com/en-us/windows/wsl/install) to install Ubuntu on Windows\
For Linux: `sudo apt install git` or other package manager of choice

## Set up SSH key:
Please follow this [comprehensive guides](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent) from Github. Be sure to select your operating system of choice. Then add it to your GitHub account with [this guide](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/adding-a-new-ssh-key-to-your-github-account?tool=webui).
> tldr: 
>```
>ssh-keygen -t ed25519 -C "your_email@example.com"
><hit enter to accept default options>
><don't add a password unless you really want to>
>cat ~/.ssh/id_ed25519.pub
><copy entire line including your email at the end>
>```
>Go to your Github settings -> SSH and GPG keys -> New SSH key -> paste in public key you just copied

## Install Python:
Open terminal and type `python3 --version` and make sure the version is = 3.10. If you need to update your version, follow the instructions on the [python website](https://wiki.python.org/moin/BeginnersGuide/Download) 

## Clone this repo:
On the repo’s main page, git the green “code” button.
Copy the “ssh” link under the “local” tab
Now, open a terminal. If you’re on Windows, your default is powershell, but we recommend using Windows Terminal. If you’re on Mac or Linux, your default terminal is fine.
From there run:
>```
>git clone [link-to-repo] [path-to-desired-repo-location] 
>```
Or cd into your desired folder and run:
>```
>git clone [link-to-repo]
>```

## Install dependencies:
Open terminal and navigate to the repo you just cloned (`cd ~/neu-underwater-robotics`).If you don't want to use a virtual environment, run `pip install -r requirements.txt` to install all needed dependencies. 

If you would like to use a venv, run `./setup.sh` to create a virtual environment for this project. Remember to run `source ./activate.sh` to activate this virtual environment when working with code in this repository, and `deactivate` when you are done. If you don't have any experience using a venv, you can skip that step. 

## Editor:
Any editor that works with Python is fine, but we recommend using [VSCode](https://code.visualstudio.com/download) as your editor especially if you are a beginner. It has great support for Python and Git. If you use VSCode, we recommend installing the following extensions:
- Python
- GitLens
- Black Formatter
- Pylint
- Remote - SSH
- GitHub Copilot (Sign up for free as a student!)
## Viewing ROS 2 logging levels

ROS log levels, from most to least verbose, are `DEBUG`, `INFO`, `WARN`, `ERROR`,
and `FATAL`. A chosen level includes messages at that severity and above.
Pass logging options after `--ros-args` when starting a node:

```sh
# Show debug messages as well as the usual info/warning/error messages.
ros2 run houston_pkg houston_node --ros-args --log-level debug

# Show warnings and errors for most loggers, but debug output for Houston.
ros2 run houston_pkg houston_node --ros-args --log-level warn --log-level houston:=debug

# Keep a run's ROS log files in a chosen directory.
ROS_LOG_DIR="$PWD/rov-logs" ros2 run houston_pkg houston_node --ros-args --log-level info
```

Logger-specific overrides use the node's runtime logger name, which can differ
from the executable name or change when the node is remapped. Existing nodes use
`get_logger().debug()`, `.info()`, `.warn()`/`.warning()`, and `.error()`.
These options filter ROS logging; ordinary Python `print()` output is unaffected.
ROS logs normally go to the terminal, log files, and `/rosout`, unless those
outputs are disabled. Restart a running node with the desired options to use
these examples; append any parameters after the same `--ros-args` separator.

Reference: [ROS 2 Humble logging configuration](https://github.com/ros2/ros2_documentation/blob/humble/source/Concepts/Intermediate/About-Logging.rst).
