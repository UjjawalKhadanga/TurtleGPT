# TurtleGPT

## Introduction
TurtleGPT is a project aimed at revolutionizing human-robot interactions by seamlessly integrating ChatGPT into the field of robotics. The project features a flexible high-level function library ensuring ChatGPT's adaptability to a diverse range of tasks, form factors, and simulation environments. Highlighting its capabilities through the Turtlesim simulator, ChatGPT in TurtleGPT demonstrates remarkable flexibility and agility. Key technologies driving this innovation include OpenAI's GPT-3.5 Turbo for advanced language skills, the TurtleSim simulator, ROS (Robot Operating System) for seamless communication, Tkinter for a user-friendly speech-to-text interface, and the Vosk library for efficient voice-based communication. This amalgamation of cutting-edge technologies redefines the landscape of human-robot collaboration within the robotics field.

## Steps to Reproduce

### Setup

1. Clone the repository:
   ```
   git clone https://github.com/UjjawalKhadanga/TurtleGPT.git
   ```
   into your ROS workspace.

2. Install required dependencies:
   ```
   pip3 install -r requirements.txt
   ```

3. Install the pyaudio dependency:
   ```
   sudo apt install python3-pyaudio
   ```

4. Build the ROS workspace:
   ```
   cd <your_ros_workspace>
   catkin_make
   ```

5. Add your OpenAI key to `./launch/example.launch` in the 'OPENAI_API_KEY' environment variable. Get an OpenAI key from [platform.openai.com](https://platform.openai.com/account/api-keys).

6. You're all set!

### Running the Program

1. Launch the program using the following command:
   ```
   roslaunch turtle_gpt example.launch
   ```

2. Choose between CLI or Text-to-Speech input.

3. Provide the desired input, and the corresponding action will be performed in the simulation.

Feel free to contribute and explore the exciting possibilities of TurtleGPT in enhancing human-robot collaboration within the robotics domain.
