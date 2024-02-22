package org.firstinspires.ftc.teamcode.drive.Autonomo;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


@Config
@Autonomous(group = "drive")
public class TestBranco extends LinearOpMode {
    //Pixel no Centro

    private DcMotor coletor = null;
    private DcMotor linear = null;
    Servo coletors;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        coletor = hardwareMap.get(DcMotor.class, "coletor");
        linear = hardwareMap.get(DcMotor.class, "mlinear");
        coletors = hardwareMap.get(Servo.class, "coletorss");

        coletor.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d startPose = new Pose2d(0, 0, 90);

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .forward(33)
                .strafeLeft(6)
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .turn(Math.toRadians(-90))
                .forward(83)
                .build();

        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .back(10)
                .strafeLeft(3)
                .build();

        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .back(125)
                .build();

        TrajectorySequence traj5 = drive.trajectorySequenceBuilder(traj4.end())
                .waitSeconds(1)
                .forward(7)
                .strafeRight(30)
                .back(17)
                .build();

        waitForStart();

        if (!isStopRequested())
            idle();
            drive.followTrajectorySequence(traj1);
            coletor.setPower(-.5);
            sleep(1500);
            coletor.setPower(0);
            drive.followTrajectorySequence(traj2);
            coletor.setPower(.73);
            sleep(1500);
            coletor.setPower(0);
            drive.followTrajectorySequence(traj3);
            drive.followTrajectorySequence(traj4);
            coletor.setPower(1);
            sleep(500);
            coletor.setPower(0);
            linear.setTargetPosition(2300);
            linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linear.setPower(1);
            sleep(1150);
            coletors.setPosition(-1);
            drive.followTrajectorySequence(traj5);

    }
}
