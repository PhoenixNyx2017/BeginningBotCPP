This contains code for an FRC Swerve base in C++

Last updated for 2023 season
Created: 12/30/2023

This swerve base makes use of the following hardware

- 8 Falcon 500s
- 4 CANcoders
- Navx v2.0
- 1-2 PS4 controllers
- Roborio v1/v2
- Power Distribution Panel

This is boiler plate code tested on FRC 122 Swerve drivetrain.

# Intellisense

Install clangd vscode extension. Run the following gradle command at the root

```
./gradlew generateCompileCommands
```

If the clangd extension doesn't detect the compile commands file, copy compile_commands.json out of the build folder into the project root.

# Code formatting

Code formatting is done with the wpiformat automatic formatter, run the following at the root

```
python3 -m pip install wpiformat
python3 -m wpiformat
```
