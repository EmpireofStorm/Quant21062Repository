package org.firstinspires.ftc.teamcode.drive.opmode;
/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.PathContinuityViolationException;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Config
@Autonomous(group = "drive")
public class BlueRightCornerMiddleTeamProp extends LinearOpMode {
    public static double coefficient = 1.375;
    public double x = 0;
    public double y = 0;
    DcMotor lift1 = hardwareMap.dcMotor.get("lift1");
    DcMotor lift2 = hardwareMap.dcMotor.get("lift2");
    @Override
    public void runOpMode() throws InterruptedException, PathContinuityViolationException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(12*coefficient, 58*coefficient));

        Trajectory startToSpikeMarkMiddle = drive.trajectoryBuilder(drive.getPoseEstimate())
                .forward(24*coefficient)
                .build();
        Trajectory spikeMarkMiddleToBack = drive.trajectoryBuilder(startToSpikeMarkMiddle.end())
                .back(5*coefficient)
                .build();
        Trajectory spikeMarkMiddleBackToBackboard = drive.trajectoryBuilder(spikeMarkMiddleToBack.end())
                .lineToLinearHeading(new Pose2d(47*coefficient, 35*coefficient, Math.toRadians(90)))
                .build();
        Trajectory backboardToNextToPark = drive.trajectoryBuilder(spikeMarkMiddleBackToBackboard.end())
                .strafeLeft(20*coefficient)
                .build();
        Trajectory nextToParkToPark = drive.trajectoryBuilder(backboardToNextToPark.end())
                .forward(15*coefficient)
                .build();
        waitForStart();

        if (isStopRequested()) return;

        drive.followTrajectory(startToSpikeMarkMiddle);
        drive.followTrajectory(spikeMarkMiddleToBack);
        drive.followTrajectory(spikeMarkMiddleBackToBackboard);
        lift1.setPower(1);
        lift2.setPower(-1);
        drive.followTrajectory(backboardToNextToPark);
        drive.followTrajectory(nextToParkToPark);

        while (!isStopRequested() && opModeIsActive()) ;

    }
}
