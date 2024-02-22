package org.firstinspires.ftc.teamcode.drive.TeleOp;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOpOfc24", group="Linear Opmode")
public class
TeleOp24 extends LinearOpMode {

    private DcMotor motoref = null;
    private DcMotor motoret = null;
    private DcMotor motordf = null;
    private DcMotor motordt = null;
    private DcMotor motordir = null; //motor 1 para Subir/Descer o Linear // VISAO DE FRENTE PARA O ROBO
    private DcMotor motoresq = null; //motor 2 para Subir/Descer o Linear // VISAO DE FRENTE PARA O ROBO
    private DcMotor linear = null; //motor para subir o linear
    //
    private DcMotor coletor = null; // motor coletor de pixels
    Servo entregador;
    int position = 0;
    int erro = 0;
    double power;
    int e1 = 1300;
    int e2 = 2300;
    double kpl = 0.9;

    @Override
    public void runOpMode() {
        motoref = hardwareMap.get(DcMotor.class, "ef");//PORTA 0 EXPANSION HUB
        motoret = hardwareMap.get(DcMotor.class, "et");//PORTA 3 EXPANSION HUB
        motoresq = hardwareMap.get(DcMotor.class, "ntm");//PORTA 3 EXPANSION HUB
        motordir = hardwareMap.get(DcMotor.class, "mtmm");//PORTA 1 EXPANSION HUB
        motordf = hardwareMap.get(DcMotor.class, "df");//PORTA 3 CONTROL HUB
        motordt = hardwareMap.get(DcMotor.class, "dt");//PORTA 0 CONTROL HUB
        linear = hardwareMap.get(DcMotor.class, "linear");//PORTA 0  CONTROL HUB
        coletor = hardwareMap.get(DcMotor.class, "coletor");//PORTA 1 CONTROL HUB
        entregador = hardwareMap.get(Servo.class, "entregador");
        motoref.setDirection(DcMotor.Direction.REVERSE);
        motoret.setDirection(DcMotor.Direction.REVERSE);
        motordf.setDirection(DcMotor.Direction.FORWARD);
        motordt.setDirection(DcMotor.Direction.FORWARD);

        motordir.setDirection(DcMotor.Direction.FORWARD);
        motoresq.setDirection(DcMotor.Direction.REVERSE);
        coletor.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setDirection(DcMotorSimple.Direction.FORWARD);
        linear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motordir.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motoresq.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Init", "Pressionado");
        telemetry.update();

        while (linear.getCurrentPosition() != 0 && motoresq.getCurrentPosition() != 0 && motordir.getCurrentPosition() != 0) {
            idle();
        }

        waitForStart();

        telemetry.addData("Start", "Pressionado");
        telemetry.update();

        while (opModeIsActive()) {
            idle();
//-------------------------------MOBILIDADE------------------------------------------------


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double eixo = gamepad1.right_stick_x * 0.8;

            double theta = Math.atan2(y,x);
            double vel = Math.hypot(x, y);

            double seno = Math.sin((theta - Math.PI/4));
            double cosseno = Math.cos((theta - Math.PI/4));
            double max = Math.abs(Math.abs(seno));

            double motoretP = vel* (seno/max) + eixo;
            double motordtP = vel* (cosseno/max) - eixo;
            double motorefP = vel* (cosseno/max) + eixo;
            double motordfP = vel* (seno/max) - eixo;

            if ((vel + Math.abs(eixo)) > 1.0) {
                motorefP /= vel+eixo;
                motordfP /= vel +eixo;
                motoretP /= vel +eixo;
                motordtP /= vel +eixo;
            }

            motoref.setPower(motorefP);
            motordf.setPower(motordfP);
            motoret.setPower(motoretP);
            motordt.setPower(motordtP);
//------------------------------------------------------------------------------------------------------------------
            motordir.setPower(gamepad2.right_stick_y);
            motoresq.setPower(gamepad2.right_stick_y);

            coletor.setPower(-gamepad2.right_trigger);
            coletor.setPower(gamepad2.left_trigger);
//------------------------------------------------------------------------------------------------------------------
            telemetry.addData("Estagio", "1", motoresq.getCurrentPosition());
            telemetry.update();


            if (gamepad2.x) {//Primeiro Estágio do Linear
                position = linear.getCurrentPosition();
                erro = position - 1300;
                linear.setTargetPosition(e1);
                linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                sleep(1000);
                entregador.setPosition(-1);
                telemetry.addData("Estagio", "1");
            } else if (gamepad2.y) {//Segundo Estágio do Linear
                position = linear.getCurrentPosition();
                erro = position - 2300;
                linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linear.setTargetPosition(e2);
                sleep(1150);
                entregador.setPosition(-1);
                telemetry.addData("Estagio", "2");
            } else if (gamepad2.a) {//Descer Linear até chegar ao sensor de toque
                entregador.setPosition(1);
                sleep(1500);
                linear.setTargetPosition(0);
                linear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linear.setPower(1);
                telemetry.addData("Estagio", "0");
            } else if (gamepad2.left_bumper){
                entregador.setPosition(1);
            } else if (gamepad2.right_bumper){
                entregador.setPosition(1);
            } else if (gamepad2.dpad_up){
                motordf.setTargetPosition(1);
            }

//--------------------------------------------PROPORCIONAL LINEAR PIXEL----------------------------------------------------------------------
            power = erro * kpl;
            if (linear.isBusy()) {
                linear.setPower(power);
            }
        }

    }
}