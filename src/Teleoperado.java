import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="teleopGeral")
public class Teleoperado extends LinearOpMode {
    private ElapsedTime timer = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor corehexintake = null;
    private DcMotor corehexgarra = null;
    private DcMotor slideintake = null;
    private DcMotor slideouttake = null;
    private Servo servo = null;
    private DcMotor motor;
    private DcMotor motor2;
    private DcMotor motor3;
    private CRServo motorgarra1;
    private CRServo motorgarra2;
    private Servo servoGarra = null;
    
    private int targetPosition = 0;
    private final int INCREMENTO = 1000;
    
    boolean garraAtiva = false;
    private long tempoAcao = 0;
    private int duracao = 0;
    private double power1 = 0;
    private double power2 = 0;

    
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;


  @Override
public void runOpMode() {
    leftFrontDrive = hardwareMap.dcMotor.get("FL");
    leftBackDrive = hardwareMap.dcMotor.get("BL");
    rightFrontDrive = hardwareMap.dcMotor.get("FRONT RIGHT");
    rightBackDrive = hardwareMap.dcMotor.get("BR");
    leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
    leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
    rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
    rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    
        motor = hardwareMap.get(DcMotor.class, "motor");
        motor3 = hardwareMap.get(DcMotor.class, "motor3");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        motorgarra1 = hardwareMap.get(CRServo.class, "MotorGarra1");
        motorgarra2 = hardwareMap.get(CRServo.class, "MotorGarra2");
        servoGarra = hardwareMap.get(Servo.class, "ServoGarra");

        motor2.setDirection(DcMotor.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
   

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        
        
    waitForStart();
    if (isStopRequested()) return;
    timer.reset();

    while (opModeIsActive()) {
        // CONTROLE 1: Movimentação (analógico esquerdo)
        double axial = -gamepad1.left_stick_y;  // Frente e Trás (Eixo Y do analógico esquerdo)
        double lateral = -gamepad1.left_stick_x; // Lateral (Eixo X do analógico esquerdo)

        // CONTROLE 2: Rotação no eixo Z (giro no próprio eixo) - analógico direito
        double yaw = -gamepad1.right_stick_x; // Giro no eixo Z (Eixo X do analógico direito)

        // Cálculo das potências dos motores
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;
    
        
        double max = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
        if (max > 0.3) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        
        if (gamepad1.a && !lastA) {
            targetPosition += INCREMENTO;
        }

        // Detecta clique do botão B (descer)
        if (gamepad1.b && !lastB) {
            targetPosition -= INCREMENTO;
        }

        // Detecta clique do botão Y (manter posição atual)
        if (gamepad1.y && !lastY) {
            targetPosition = motor.getCurrentPosition();
        }
        
       if(gamepad2.x){
           timer.reset();
           garraAtiva = true;
           duracao = 500;
           power1 = 0.5;
           power2 = 0.9;
       }

 else if (gamepad2.a) {
        timer.reset();
        garraAtiva = true;
        duracao = 300;
        power1 = -1.0;
        power2 = 1.0;
    }
    else if (gamepad2.y) {
        timer.reset();
        garraAtiva = true;
        duracao = 300;
        power1 = 1.0;
        power2 = -1.0;
    }
    else if (gamepad2.b) {
        timer.reset();
        garraAtiva = true;
        duracao = 450;
        power1 = -0.9;
        power2 = -1.0;
    }
        
        
        
       if (gamepad1.right_trigger > 0.1) {
    motor3.setPower(0.5);
}       
if (gamepad1.left_trigger > 0.1) {
    motor3.setPower(-0.5);
}       
if (gamepad1.right_bumper){
    motor3.setPower(0.0);
}

  if (gamepad2.left_trigger > 0.0) {
    servoGarra.setPosition(0);
}
      
    //Garra em formato de pinça se fecha
 if (gamepad2.right_trigger > 0.0)
     servoGarra.setPosition(0.3);
    

        // Atualiza os estados dos botões
        lastA = gamepad1.a;
        lastB = gamepad1.b;
        lastY = gamepad1.y;
        
        
        if (garraAtiva) {
        if (timer.milliseconds() < duracao) {
            motorgarra1.setPower(power1);
            motorgarra2.setPower(power2);
        } else {
            motorgarra1.setPower(0);
            motorgarra2.setPower(0);
            garraAtiva = false; // encerra a ação
        }
    }

        // Define nova posição e roda para ela
        motor.setTargetPosition(targetPosition);
        motor2.setTargetPosition(targetPosition);

        if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (motor2.getMode() != DcMotor.RunMode.RUN_TO_POSITION)
            motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor.setPower(1);
        motor2.setPower(0.5);

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        
         telemetry.addData("Target Pos", targetPosition);
        telemetry.addData("Motor1 Pos", motor.getCurrentPosition());
        telemetry.addData("Motor2 Pos", motor2.getCurrentPosition());
        telemetry.update();
    }
}

}
