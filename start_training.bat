@echo off
REM Activate the virtual environment
call C:\Users\kevin\Documents\_Coding\Projects\Rabbit_Project_v4\.venv\Scripts\activate.bat

REM Run the Python script
python C:\Users\kevin\Documents\_Coding\Projects\Rabbit_Project_v4\Train_RL_Agents.py

REM Deactivate the virtual environment
deactivate