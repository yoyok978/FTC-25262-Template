# 03 - Java and FTC foundations

[עברית](he/03-foundations.md)

[Guide index](../../README.md#learning-path) | Example: [01 Lifecycle](../../TeamCode/src/main/java/org/firstinspires/ftc/teamcode/examples/Example01Lifecycle.java) | Next: [commands](04-commands.md)

## Read a Java file

`package` places a class in a namespace matching its directory. `import` lets code use a type's short name. A public class's name must match its filename. When copying an example into `opmodes`, change the package to `org.firstinspires.ftc.teamcode.opmodes` and rename the class and file together.

A class groups state and behavior. Fields hold state between calls; local variables exist only within their block. A constructor initializes an object, for example `new MotorSubsystem(hardwareMap, "exampleMotor", Direction.FORWARD)`. `extends` inherits behavior: your OpMode extends RobotCommandOpMode, which uses SolversLib's CommandOpMode lifecycle. `@Override` asks the compiler to check that you are replacing an inherited method correctly.

`@TeleOp(name = "My TeleOp", group = "Team")` registers a driver-controlled OpMode. `@Autonomous` registers an autonomous one. `@Disabled` prevents an example from appearing as a selectable normal OpMode; merely compiling it does not enable it. `@Configurable` exposes supported fields through Panels, not through the Java compiler.

`private` hides implementation details. `public` exposes an interface. `static` means class-wide state, not one value per object. A `static final` PoseFactory is a shared immutable factory; a public static configurable power field may persist across OpMode instances in the same process. Do not assume every INIT resets static values.

## Lifecycle in this project

| Method | When it runs | Typical work |
|---|---|---|
| `initialize()` | Once after INIT is selected | Reset scheduler, map hardware, construct subsystems, bind buttons |
| `initialize_loop()` | Repeatedly while waiting for START | Show configuration/status; avoid initiating movement |
| `preRun()` | Once after START, before active loops | Optional start-specific work |
| `run()` | Repeatedly while active | `super.run()`, then telemetry |
| `end()` | At the end of the active lifecycle | Cancel commands and release mechanisms |

RobotCommandOpMode inherits a LinearOpMode-based runner, so override these methods rather than writing an additional `while (opModeIsActive())` inside `run()`. Always call `super.run()` when overriding it; that is what runs the command scheduler. Always preserve `super.end()` if adding your own cleanup. The base also cancels commands before scheduler reset. Hardware construction can fail before a routine starts; use meaningful hardware names and inspect INIT errors rather than ignoring exceptions.

Example01 has no hardwareMap calls. Its loop counter is local robot software state, not elapsed seconds: the loop rate varies. Use a timer for durations, and `WaitCommand` for delays inside command sequences.

## Lambdas, method references and suppliers

A lambda is behavior passed as a value:

```text
() -> gamepad1.left_bumper ? 0.2 : 0.4
```

This is a `DoubleSupplier`: when called, it reads the current bumper and returns a number. `BooleanSupplier` returns a boolean. `motor::stop` is a method reference equivalent to `() -> motor.stop()`. An interface describes the operations an object supports; a supplier is a tiny interface with one getter-like method.

The difference matters when creating commands. `double power = REQUESTED;` saves a number now. `() -> REQUESTED` reads later, but only when the receiving command calls it. MotorSubsystem reads its supplier every execution; ServoSubsystem reads its position supplier once when its instant command initializes. A supplier alone does not automatically run every loop.

## Hardware, inputs and telemetry

`hardwareMap.get(DcMotorEx.class, "exampleMotor")` asks for a configured device by type and exact name. It does not create imaginary hardware. Missing names/types throw during initialization. Gamepad state is updated by the SDK; raw stick up is negative Y and right is positive X. GamepadEx wraps inputs for button bindings. The starter negates axes to request forward, left and counterclockwise motion.

`telemetry.addData("label", value)` queues a value; `telemetry.update()` sends it. JoinedTelemetry sends compatible data to both Driver Station and Panels. Display useful units: `ticks/s`, `in`, `rad`, or normalized power. A servo's reported position is the requested controller value, not a measured physical angle.

Exercise: copy Example01, display both stick axes, and add a counter that changes on an A press using GamepadEx. Predict whether a raw `if (gamepad1.a)` increments once or every loop. Use [command bindings](04-commands.md) for edge-triggered behavior.
