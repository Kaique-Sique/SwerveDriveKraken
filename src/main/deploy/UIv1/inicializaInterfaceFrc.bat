@echo off
cd "C:\Users\Ricardo\Documents\GitHub\SwerveDriveKraken\src\main\depoy\UIv1"
start http://localhost:8888
pynetworktables2js --robot roborio-7563-frc.local

pause