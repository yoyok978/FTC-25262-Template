# 05 - Hardware and reusable subsystems

[עברית](he/05-hardware.md)

[Guide index](../../README.md#learning-path) | Examples: [02 Motor](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example02Motor.java), [03 Servo](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example03Servo.java) | Next: [driving](06-driving.md)

## Map one physical device once

Configure the device on the Robot Controller, give it a meaningful name, and use that exact string in code. A name such as `intake` describes a device's role; a port number alone does not. `hardwareMap.get(DcMotorEx.class, "exampleMotor")` requires a compatible configured motor. A servo needs `Servo.class`, while a continuous-rotation servo uses a different SDK API and is not covered by ServoSubsystem.

Create one subsystem instance per mechanism in an OpMode, then share that instance among its commands. Two subsystem instances wrapping the same motor fool command ownership: the scheduler sees different owners for the same hardware. Group mechanisms by real coordination needs, not by how many source files you want.

## MotorSubsystem, line by responsibility

[MotorSubsystem.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/MotorSubsystem.java) has two constructors. The HardwareMap constructor resolves a named device. The other constructor accepts an already supplied `DcMotorEx`. `SubsystemBase` registers the object with the scheduler automatically.

Construction sets direction, `RUN_WITHOUT_ENCODER`, zero-power BRAKE, initial zero power, and a continuous default stop command. These are explicit choices for an open-loop intake/conveyor example. BRAKE resists coasting; it is not a position-holding controller. An encoder reading may still be available while the motor is in open-loop mode.

`setPower()` accepts normalized power, rejects non-finite input and clamps to [-1, 1]. `velocity()` reports SDK encoder ticks per second. `run(DoubleSupplier)` is a command factory: it reads the supplied power each execute cycle, requires this subsystem, and stops when canceled. `stopCommand()` is finite; `defaultStopCommand()` is continuous.

To turn it into your intake:

1. Copy MotorSubsystem to `IntakeSubsystem.java` and rename the class and constructors.
2. Choose the configured name and physical direction in your OpMode constructor call.
3. Add mechanism operations such as `intake()`/`eject()` factories using measured power limits.
4. Bind A to intake and B to eject in your own TeleOp. Keep requirements and cleanup.
5. Test initialization at zero, direction, release, cancellation, then STOP.

Do not copy the old shooter's motor RPM, PIDF or velocity setpoints into a different mechanism. They belonged to that robot and are archived.

## Control modes and units

| Mode | What your request means | Additional work |
|---|---|---|
| SDK `RUN_WITHOUT_ENCODER` | Open-loop normalized power | Determine suitable power and directions |
| SDK `RUN_USING_ENCODER` with `setVelocity()` | Closed-loop encoder ticks/second (or specified angular units) | Verify motor configuration, encoder resolution and PIDF |
| SDK `RUN_TO_POSITION` | Controller moves toward a target encoder position | Set target before entering mode; add homing, tolerance, timeout and cleanup |
| A library's software velocity controller | Library-specific feedback/feedforward | Tune that controller; SDK PIDF may not affect it |

Choose one controller owner intentionally. To convert ticks/second to shaft revolutions/second divide by actual ticks/revolution, accounting for gearing and the encoder location. A motor catalog's nominal RPM is not a measured loaded speed. Never mix inches, centimeters and encoder ticks in one undocumented target.

The generic example does not configure current alerts, limit switches, homing or fault latching. Add those for your real mechanism. An SDK current alert requires a threshold chosen from your hardware measurements; checking `isOverCurrent()` without configuring a useful threshold is insufficient. Pass a combined stop condition into MotorCommands or create a dedicated command, and decide whether holding a button should be allowed to retry after a fault clears.

## ServoSubsystem

[ServoSubsystem.java](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsystems/ServoSubsystem.java) maps a positional servo without issuing a position in its constructor. `setPosition()` validates [0, 1]. `moveTo(DoubleSupplier)` reads the position when the instant command initializes and requires the subsystem. It finishes immediately after sending the request.

Those normalized positions are neither degrees nor feedback. Example03 uses 0.45/0.55 only as small example requests; actual linkage limits must be measured. A servo can move quickly to either request. STOP does not automatically return it to a safe position, and cancellation does not undo a completed servo command. Define any shutdown policy for the actual mechanism explicitly; there is no universal safe endpoint.

Exercise: make a `ClawSubsystem` exposing `openCommand()` and `closeCommand()` with named, measured positions. Add the claw to a copied motor TeleOp, then explain why the claw and intake can run in parallel. Verify the linkage with the servo horn/mechanism configured appropriately before extending travel.
