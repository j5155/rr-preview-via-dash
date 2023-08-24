# RR Preview Via Dashboard
This is a project to preview [Roadrunner](https://github.com/acmerobotics/road-runner) 1.x trajectories using [FTC Dashboard](https://github.com/acmerobotics/ftc-dashboard).
It works by using the integrated Dashboard preview and visualisation features of RR and "gaslighting" a standard autonomous into running without a robot.

## Installation
The simplest way of using this is to clone this repository (or download it as a zip) and open it in IntelliJ IDEA (or Android Studio).
Then, edit the [ExampleOpMode](https://github.com/j5155/rr-preview-via-dash/blob/main/src/main/java/com/github/j5155/ExampleOpMode.java) file, following the comments.
After that, with the ExampleOpMode open in your IDE, hit the green play button with Current File selected.

Finally, you can view your trajectory by installing and running the [Dashboard app](https://github.com/jdhs-ftc/ftc-dashboard/releases). 
Select the version for your OS from the top release on https://github.com/jdhs-ftc/ftc-dashboard/releases, and after setting it up you should be able to run it and see your trajectory.

To edit and iterate on your trajectory, simply stop the ExampleOpMode in your IDE and rerun it with your new changes.
You can leave the Dashboard app running and it will automatically update with your new trajectory.

## Use with Existing Android Studio project
If you already have an existing Android Studio project, you can add this to it by following these steps:
(Credit to NoahBres's [MeepMeep](https://github.com/NoahBres/MeepMeep) for these instructions)


1.  In Android Studio, click on the "FtcRobotController" Module, then right click on the FtcRobotController folder and click `New > Module`
    <img src="/images/readme/installationStep1.png" width="751" height="287"/>
2.  On the left part of this window, select "Java or Kotlin Library"
    <img src="/images/readme/installationStep2.png" width="544" height="382"/>

3.  From here, remove the `:ftcrobotcontroller:lib` in the "Library Name" section, and rename it to `RRPreview`. You may use whatever name you wish but the rest of the instructions will assume you have chosen the name `RRPreview`. Ensure that you also change the "class name" section to match.

4.  Hit "Finish" at the bottom right of the Module Create window.

5.  Open up the `build.gradle` file for the RRPreview module (or whatever you chose to name it prior). In this file, change all instances `JavaVersion.VERSION_1_7` to `JavaVersion.VERSION_17`
    <img src="/images/readme/installationStep5.png" width="566" height="274"/>
NOTE: This image shows MeepMeepTesting (original source of these instructions), you want RRPreview

6.  At the bottom of the file add the following gradle snippet:

```
repositories {
    maven { url = "https://jitpack.io" }
    maven { url = "https://maven.brott.dev/" }
}

dependencies {
    implementation "com.github.j5155:rr-preview-via-dash:0.1.4"
    implementation "com.acmerobotics.roadrunner:core:1.0.0-beta2"
    implementation "com.acmerobotics.roadrunner:actions:1.0.0-beta2"
}
```

7.  When android studio prompts you to make a gradle sync, click "Sync Now".
    <img src="/images/readme/installationStep7.png" width="644" height="20"/>

8.  Create a class for your RRPreview java module if it does not yet exist. Paste the following sample in it. Make sure to follow the comments to update it to match your robot.

```java
package com.example.rrpreview;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.github.j5155.PreviewMecanumDrive;
import com.github.j5155.TestDashboardInstance;

public class ExampleOpMode {
    // TODO: Copy these from your existing MecanumDrive!

    // drive model parameters
    public static double IN_PER_TICK = 0.000549539;
    public static double LATERAL_IN_PER_TICK = 1;
    public static double TRACK_WIDTH_TICKS = 48852.1340223;
    public static double LATERAL_MULTIPLIER = IN_PER_TICK / LATERAL_IN_PER_TICK;

    // path profile parameters
    public static double MAX_WHEEL_VEL = 50;
    public static double MIN_PROFILE_ACCEL = -30;
    public static double MAX_PROFILE_ACCEL = 50;

    // turn profile parameters
    public static double MAX_ANG_VEL = Math.PI; // shared with path
    public static double MAX_ANG_ACCEL = Math.PI;

    public static void main(String[] args) {
        TestDashboardInstance dash = TestDashboardInstance.getInstance();
        dash.start();

        Canvas c = new Canvas();
        PreviewMecanumDrive drive = new PreviewMecanumDrive(IN_PER_TICK, LATERAL_IN_PER_TICK, TRACK_WIDTH_TICKS, LATERAL_MULTIPLIER,
                MAX_WHEEL_VEL, MIN_PROFILE_ACCEL, MAX_PROFILE_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL);

        Pose2d startPose = new Pose2d(0,0,0); // TODO: Update this to reflect your autonomous start position
        TrajectoryActionBuilder trajBuild =
                drive.actionBuilder(startPose) // TODO: Copy your autonomous here
                        .splineTo(new Vector2d(0, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .strafeTo(new Vector2d(48,48));
        // Make sure to remove the .build(), it is not needed here

        Action traj = trajBuild.build();
        traj.preview(c);
        PreviewMecanumDrive.drawRobot(c, startPose);

        while(true) {
            TelemetryPacket p = new TelemetryPacket();
            if (!traj.run(p)) traj = trajBuild.build();
            p.fieldOverlay().getOperations().addAll(c.getOperations());
            dash.core.sendTelemetryPacket(p);
        }


    }
}
```

9. Create a run configuration for Android Studio.
    1. First, click on the drop down menu on the top bar of Android Studio, where it says "TeamCode" with a little Android logo next to it.
    2. Click `Edit Configurations`
    3. Click on the "+" symbol in the top left of the window, and when it prompts you, select "Application".
    4. Change the name to your liking (ex. rr-preview)
    5. Where it says "module not specified", click to open the dropdown, then select your JRE.
    6. Where it says "cp <no module>" click it to open the dropdown, and then select FtcRobotController.RRPreview.main
    7. Where it says "Main Class", click the little "file" icon to the right of the text and then select the name of the main class for your RRPreview module.
    8. From here, in the bottom right of the window, press "Apply" then "Ok".
    9. It will now automatically switch to that Run/Debug Configuration profile.
10. If at any point you would like to build code onto your Control Hub or Phone, then click the Run/Debug configuration profile at the top to open the dropdown menu and select TeamCode. Perform the same steps to switch back to RRPreview.
11. Finally, you can view your trajectory by installing and running the [Dashboard app](https://github.com/jdhs-ftc/ftc-dashboard/releases).
    Select the version for your OS from the top release on https://github.com/jdhs-ftc/ftc-dashboard/releases, and after setting it up you should be able to run it and see your trajectory.

To edit and iterate on your trajectory, simply stop the ExampleOpMode in your IDE and rerun it with your new changes.
You can leave the Dashboard app running and it will automatically update with your new trajectory.
