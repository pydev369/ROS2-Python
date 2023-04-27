End-to-end robot movement project, dashboard, and Docker containerization:

## Step 1: Setting up the Robot Movement Project

1. Clone the project repository from GitHub.

2. Install ROS2 on your system following the official installation guide.

3. Create a ROS2 workspace in the project directory:

```
mkdir -p ~/robot_ws/src
cd ~/robot_ws/src
```

4. Copy the robot movement script created earlier to the "src" directory.

5. Build the workspace using the following command:

```
cd ~/robot_ws
colcon build
```

6. Source the workspace:

```
source ~/robot_ws/install/setup.bash
```

7. Run the robot movement script using the following command:

```
ros2 run <package_name> <script_name>
```

## Step 2: Setting up the Flask App and UI

1. Install Flask on your system using the following command:

```
pip install flask
```

2. Create a new Python file in the project directory and name it "dashboard.py".

3. Import the necessary modules in the "dashboard.py" file:

```python
from flask import Flask, render_template
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
```

4. Create the Flask app and set up a route to the dashboard page:

```python
app = Flask(__name__)

@app.route('/')
def dashboard():
    return render_template('dashboard.html')
```

5. Create the ROS2 node class that subscribes to the robot's current position and velocity:

```python
class RobotNode(Node):

    def __init__(self):
        super().__init__('my_robot')

        self.current_position_ = (0, 0)
        self.current_velocity_ = 0.0

        self.subscription_ = self.create_subscription(Twist, 'cmd_vel', self.callback, 10)
        self.subscription_

    def callback(self, msg):
        self.current_velocity_ = msg.linear.x

        x = self.current_position_[0] + msg.linear.x
        y = self.current_position_[1] + msg.linear.y

        self.current_position_ = (x, y)
```

6. Add the Flask app and ROS2 node logic to the "dashboard.py" file:

```python
def main(args=None):
    rclpy.init(args=args)
    robot_node = RobotNode()

    try:
        rclpy.spin(robot_node)
    except KeyboardInterrupt:
        pass

    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

7. Create the dashboard HTML file in the "templates" directory called "dashboard.html":

```html
<!DOCTYPE html>
<html>
<head>
    <title>Robot Dashboard</title>
</head>
<body>
    <h1>Robot Dashboard</h1>

    <div>
        <p>Current position: {{ current_position }}</p>
        <p>Current velocity: {{ current_velocity }}</p>
    </div>
</body>
</html>
```

8. Update the Flask route to include the current position and velocity data:

```python
@app.route('/')
def dashboard():
    return render_template('dashboard.html', current_position=robot_node.current_position_, current_velocity=robot_node.current_velocity_)
```

9. Run the Flask app using the following command:

```
FLASK_APP=dashboard.py flask run
```

10. Open a web browser and go to http://localhost:5000/ to view the robot dashboard.

## Step 3: Creating a requirements.txt file

1. Create a new file in the project directory called "requirements.txt".

2
