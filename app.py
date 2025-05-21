# your_robot_dashboard/app.py

import dash
import dash_bootstrap_components as dbc
import os

# Import configurations and components
from . import config # if running as a module `python -m your_robot_dashboard.app`
# from config import TRAJECTORY_DIR # if running directly `python app.py` in the root
                                 # and your_robot_dashboard is in PYTHONPATH

from .components.layout import create_layout
from .callbacks.ui_callbacks import register_callbacks
from .ros_comms.handler import cleanup_ros_threads # Corrected import for cleanup

# --- Dash App Initialization ---
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.LUX], suppress_callback_exceptions=True, update_title=None)
server = app.server # For Gunicorn/Waitress deployment

# Set the layout
app.layout = create_layout()

# Register all callbacks
register_callbacks(app)


# --- Main Execution Block ---
if __name__ == "__main__":
    # Ensure trajectory directory exists (config.py already does this, but can be a safeguard)
    if not os.path.exists(config.TRAJECTORY_DIR):
        try:
            os.makedirs(config.TRAJECTORY_DIR)
            print(f"Created trajectory directory: {config.TRAJECTORY_DIR}")
        except OSError as e:
            print(f"Error creating trajectory directory {config.TRAJECTORY_DIR}: {e}.")

    try:
        print("Starting Dash application server on http://0.0.0.0:8050/")
        # For development: debug=True. For production, use a proper WSGI server.
        app.run(debug=True, host='0.0.0.0', port=8050)
    except KeyboardInterrupt:
        print("\nDash server stopping due to KeyboardInterrupt.")
    except Exception as e:
        print(f"An unexpected error caused the Dash server to stop: {e}")
    finally:
        print("Performing cleanup before application shutdown...")
        cleanup_ros_threads() # Use the imported cleanup function
        print("Application shutdown complete.")