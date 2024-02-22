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
public class VermDireitaD extends LinearOpMode {
    //Pixel no Direita

    private DcMotor coletor = null;
    private DcMotor linear = null;
    Servo entregador;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        coletor = hardwareMap.get(DcMotor.class, "coletor");
        linear = hardwareMap.get(DcMotor.class, "linear");
        entregador = hardwareMap.get(Servo.class, "entregador");

        coletor.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Pose2d startPose = new Pose2d(0, 0, 90);

        drive.setPoseEstimate(startPose);

        TrajectorySequence traj1 = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(5) //Ir na Diagonal Direita
                .forward(39) //Ir para Frente
                .turn(Math.toRadians(90)) //Girar para a Esquerda
                .strafeRight(7) //Ir na Diagonal Direita
                .build();
        TrajectorySequence traj2 = drive.trajectorySequenceBuilder(traj1.end())
                .back(55) //Ir para trás até o Backdrop
                .build();
        TrajectorySequence traj3 = drive.trajectorySequenceBuilder(traj2.end())
                .waitSeconds(1) //Subir Linear e soltar pixel
                .build();
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(traj3.end())
                .forward(15) //Ir para Frente
                .strafeLeft(40) //Ir na Diagonal Esquerda
                .back(20) //Estacionar
                .build();

        waitForStart();

        if (!isStopRequested())
            idle();
        drive.followTrajectorySequence(traj1);
        //Soltar pixel roxo
        coletor.setPower(-.5);
        sleep(1500);
        coletor.setPower(0);
        drive.followTrajectorySequence(traj2);
        //Subir o Linear e o efetuador
        linear.setTargetPosition(1470);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(1);
        sleep(1150);
        entregador.setPosition(-1);
        sleep(2000);
        drive.followTrajectorySequence(traj3);
        //Descer o Linear e o efetuador
        entregador.setPosition(1);
        sleep(1000);
        linear.setTargetPosition(0);
        linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linear.setPower(1);
        drive.followTrajectorySequence(traj4);
        //Estacionar


    }
}
