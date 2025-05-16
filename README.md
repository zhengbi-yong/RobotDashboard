# Robot Control Dashboard

A Dash application for controlling and monitoring a robot with dual arms and a head.

## Setup

1.  **Clone the repository (or create the directory structure).**
2.  **Install Python dependencies:**
    ```bash
    pip install -r requirements.txt
    ```
3.  **Configure ROS:**
    Ensure your ROS master and `rosbridge_server` are running.
    Update `your_robot_dashboard/config.py` with the correct `ROS_BRIDGE_HOST` if necessary.

## Running the Application

Navigate to the `your_robot_dashboard` directory's PARENT directory in your terminal.
Then run the application as a module:

```bash
python -m RobotDashboard.app