@echo off
REM Activate the virtual environment
call C:\Users\kevin\Documents\_Coding\Projects\Rabbit_Project_v4\.venv\Scripts\activate.bat

REM Run the tensorbard command
call tensorboard --logdir=C:\Users\kevin\Documents\_Coding\Projects\Rabbit_Project_v4\Models\SACDisney_Imitation_v10


REM Deactivate the virtual environment
deactivate