# 01 - Setup and first run

[עברית](he/01-setup.md)

[Guide index](../../README.md#learning-path) | Next: [project structure](02-structure.md)

## Get a team copy

Use your team's repository created from this template. From the IDE welcome screen, choose **Get from VCS** or **Clone Repository**, paste your team's repository URL, and open the cloned project. For an existing copy, use **File > Open** and select the root directory containing `settings.gradle.kts`. Do not submit robot-specific work to the upstream FTC SDK.

Install an Android-capable IDE such as Android Studio Narwhal 3 Feature Drop or later, Git, the Android SDK, and a Java 25 JDK. IntelliJ needs Android project support; if that is unavailable, use Android Studio. Open **Settings > Build, Execution, Deployment > Build Tools > Gradle** and set **Gradle JDK/JVM** to Java 25. Use the JDK selector to add or download it if needed. Setting only the Java editor's language level does not select the Gradle runtime. Menu labels can vary slightly between IDE versions; use **Find Action** to locate a named action when needed.

The template uses Gradle 9.1.0, Dairy's SDK 12 plugin, and Android Gradle plugin 8.13.2. Production code is compiled using the Java 8 toolchain for FTC compatibility. Foojay can resolve missing toolchains when downloads are available. Java 8 source in your editor does not mean you should run Gradle on Java 8.

## Android SDK and local settings

Use the IDE SDK Manager to install the platform and build tools requested during Gradle sync. Dairy currently configures compile SDK 30; installed dependencies may cause additional SDK/tool downloads. Keep FIRST/Dairy's target/minimum SDK conventions unless performing a separate compatibility review.

Select your machine's Android SDK location in **SDK Manager** and let the IDE create its local SDK settings in `local.properties`. Do not copy somebody else's absolute path or select a JDK directory as the Android SDK. This file is ignored by Git.

Let the initial Gradle sync finish. After changing dependencies or build configuration, click **Sync Now** when prompted, or **Sync Project with Gradle Files** in Android Studio. In IntelliJ, use **Reload All Gradle Projects** in the Gradle tool window. Ordinary Java edits do not normally need another sync.

Your everyday workflow is **edit code → select the controller → Run ▶**. Run builds and installs the app automatically; a separate Build action is usually unnecessary. Sync imports build configuration and dependencies; it does not deploy the app. See [Android Studio's build and run guide](https://developer.android.com/build/building-studio).

Do not create another `TeamCode/build.gradle` or add module includes to repair this single-application layout. Read errors in the Build/Sync tool window before changing versions; [chapter 09](09-troubleshooting.md) covers common problems.

## Configure the robot

The enabled Starter TeleOp needs four independently named mecanum motors, Pinpoint and compatible pods. Use the names in [the README hardware table](../../README.md#required-hardware), or edit Constants to your names. Select the corresponding configuration on the Robot Controller. Configuration names refer to devices, not Java class names.

Set signed pod offsets, pod type, encoder directions and motor directions before field-centric driving. The template's offsets are zero placeholders. A successful INIT only proves device lookup and initialization succeeded; it does not prove the directions or units are correct.

## Install and run

1. Connect the Robot Controller through USB or an already configured authorized wireless connection. Accept any USB debugging authorization prompt on the controller. Select the actual controller in the IDE's device selector. If it is missing or offline, use the device connection troubleshooting controls and check the cable and authorization.
2. With the robot stopped, select the project's **Android App** run configuration in the toolbar, then click **Run ▶**. This project's configuration may be named **FTC-25262-Template** or **TeamCode**; the name alone does not establish its type. Use **Run > Edit Configurations** to check that it targets the root Android application. Wait for the IDE to finish building and installing on the selected controller. You do not need to click Build first. If the last selected configuration was a Sloth task, switch back to the Android app configuration.
3. Match the Driver Station to the FTC SDK generation used here (12.0). Restore connection and select the intended hardware configuration.
4. Choose **Starter TeleOp**, press INIT, read telemetry, then START. Test each direction at low power with the robot supported appropriately. STOP ends scheduled commands and zeroes drive outputs.

IDE **Run ▶** deploys the Robot Controller app. Driver Station **INIT** and **START** initialize and run the selected OpMode on that controller; these are separate actions. A Java Application run configuration does not deploy the FTC app.

For learning without peripherals, copy lesson 01, change its package/file/class/annotation names and remove `@Disabled`. Click the Android app's **Run ▶** again, then select that lesson on Driver Station; it never constructs PedroDrive or calls hardwareMap. A hub/Robot Controller app is still required to run an OpMode.

## Check before continuing

You should be able to identify the Gradle JDK, Android SDK location, selected controller and active hardware configuration. Explain when Sync is needed, how Run builds and installs, and why Driver Station INIT/START are still required. Record the tool versions in team notes so the next programmer can reproduce the setup.
