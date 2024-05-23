import rclpy
from rclpy.node import Node
from threading import Thread
import serial 
import math
from std_msgs.msg import Float64MultiArray

class MotorController(Node):
    def __init__(self):

        super().__init__('motor_controller')
        self.subscription_input_velocities = self.create_subscription(Float64MultiArray, "/motor_desired_velocities", self.input_velocities_callback, 0)      # CHANGE
        self.arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
        if(not self.arduino.is_open):
            self.arduino.open()
            
        # create publishers
        self.tendon_lengths_publisher       = self.create_publisher(Float64MultiArray, 'tendon_lengths', 0)
        self.motor_velocities_publisher     = self.create_publisher(Float64MultiArray, 'motor_velocities', 0)
        self.motor_encoder_counts_publisher = self.create_publisher(Float64MultiArray, 'motor_encoder_counts', 0)

        # define initial state
        self.initial_tendon_length = 150.0
        self.encoder_ticks_per_rev = 12
        self.tendon_mm_per_rev = 7 * math.pi    
        
        # create motors
        motor1 = Motor(1, self.initial_tendon_length, self.encoder_ticks_per_rev, self.tendon_mm_per_rev)
        motor2 = Motor(2, self.initial_tendon_length, self.encoder_ticks_per_rev, self.tendon_mm_per_rev)
        motor3 = Motor(3, self.initial_tendon_length, self.encoder_ticks_per_rev, self.tendon_mm_per_rev)

        self.motor_arr = [motor1, motor2, motor3]

    def input_velocities_callback(self, commands):
        """
        Gets commands from data and writes to arduino.
    
        Args:
            commands (int[]): The int arr of commands to be sent to Arduino
        """
        command_str = self.convert_cmd_to_str(commands.data)
        self.write(command_str) #TODO undo

    def write(self, command_str): 
        """
        Writes a command to the arduino.
    
        Args:
            commandsStr (String): The string of commands to be sent to Arduino, of form 
                "[motor1 rpm] [motor2 rpm] [motor3 rpm]"
        """
        
        self.arduino.write(bytes(command_str, 'utf-8'))
    
    def convert_cmd_to_str(self, commands):
        """
        Convert commands to String for Arduino.
    
        Args:
            commands (int[]): The commands to be sent to arduino.
        Return: 
            String: The command String to be sent to Arduino, of form 
                "[motor1 rpm] [motor2 rpm] [motor3 rpm]"
        """

        command_str =  str(int(commands[0])) + " " + str(int(commands[1])) + " " + str(int(commands[2])) + "\r\n"
        
        return command_str # TODO 
    
    def parse_measured_outputs(self, measured_output_str):
        """
        Convert measured_outputs_str into var array for motors i.e. vel, current, etc..
    
        Args:
            measured_output_str (String): The string of received outputs from Arduino.
                Of form "[motor1_encoder_counts] [motor1_velocities] [motor1_currents] , 
                        [motor2_encoder_counts] [motor2_velocities] [motor2_currents] , 
                        [motor3_encoder_counts] [motor3_velocities] [motor3_currents] ,"
                Ex. "0 0 0 1 0 0 0 219 ,0 0 0 1 0 0 0 115 ,0 0 0 1 0 0 0 129 ,""
        Return: 
            int[][]: the array with all data parsed.
        """
        # split for individual motors
        split_for_motors_str_arr = str(measured_output_str)[2:-7].split(" ,") # 2 & -7 removes excess chars i.e. /r/n and extra spaces

        if len(split_for_motors_str_arr) != len(self.motor_arr):
            raise UnparsableReceivedOutputsStrException()
        
        returned_motor_val_arr = [0] * len(self.motor_arr)

        # get motor vals for each motor
        for i in range(len(self.motor_arr)):
            split_for_motor_vals_str_arr = str(split_for_motors_str_arr[i]).split()
            
            for stry in split_for_motor_vals_str_arr:
                a = None

            if len(split_for_motor_vals_str_arr) != 8:
                raise UnparsableReceivedOutputsStrException()

            # convert str arr into int arr
            split_for_motor_vals_int_arr = [0] * 8
            for j in range(len(split_for_motor_vals_str_arr)):
                split_for_motor_vals_int_arr[j] = int(str(split_for_motor_vals_str_arr[j]))

            # create encoder, velocity, and current arr
            encoder_count_arr = [split_for_motor_vals_int_arr[0], 
                                  split_for_motor_vals_int_arr[1],
                                  split_for_motor_vals_int_arr[2],
                                  split_for_motor_vals_int_arr[3]]
            velocity_arr = [split_for_motor_vals_int_arr[0],
                            split_for_motor_vals_int_arr[1]]
            current_arr = [split_for_motor_vals_int_arr[0],
                           split_for_motor_vals_int_arr[1]]
            
            returned_motor_val_arr[i] = [encoder_count_arr, velocity_arr, current_arr]
        return returned_motor_val_arr # TODO

class UnparsableReceivedOutputsStrException(Exception):
    """
        Represents exceptions when the received outputs string can't be parsed.
    """
    pass

class Motor:
    """
        Represents motors.
 
        Attributes:
            id (int): The real part of the complex number.
            tendon_length (int): The imaginary part of the complex number.
            encoder_count_arr (int): The encoder value array in encoder ticks.
            last_encoder_count_arr (int): The last encoder value array in encoder ticks.
            velocity_arr (int): The velocity array in rpm.
            current_arr (int): The array of currents in amps.
    """
    def __init__(self, id, initial_tendon_length, encoder_ticks_per_rev, tendon_mm_per_rev):
        """
        Initializes a Motor object.
 
        Parameters:
            id (int): The id of the motor.
            initial_tendon_length (double): The initial length (mm) of the related tendon of the motor.
            encoder_ticks_per_rev (int): The encoder ticks per motor revolution.
            tendon_mm_per_rev (double): The length (mm) of tendon wrapped around the motor spool after one rev.
        """

        # set name
        self.id = id

        # set tendon length vals
        self.tendon_length = initial_tendon_length
        self.tendon_mm_per_rev = tendon_mm_per_rev

        # set encoder vals (null at start)
        self.encoder_count_arr = None
        self.last_encoder_count_arr = None
        self.encoder_ticks_per_rev = encoder_ticks_per_rev

        # set velocities (null at start)
        self.velocity_arr = None

        # set currents (null at start)
        self.current_arr = None

    def update_motor_vars(self, encoder_count_arr, velocity_arr, current_arr):
        """
        Updates instance variables for received vals.
    
        Args:
            encoder_count_arr (int[]): The encoder value array in encoder ticks.
            velocity_arr (int[]): The velocity array in rpm.
            current_arr (int[]): The current array in amps.
        """

        # update measured vals
        self.last_encoder_count_arr = self.encoder_count_arr
        self.encoder_count_arr = encoder_count_arr
        self.velocity_arr = velocity_arr
        self.current_arr = current_arr

        self.update_tendon_length() # TODO update tendon length

    def update_tendon_length(self):
        """
        Updates tendon length of motor.
        """
        if self.last_encoder_count_arr is not None:       

            encoder_count_delta = math.pow(2,16)*(self.encoder_count_arr[0] - self.last_encoder_count_arr[0]) + math.pow(2,8)*(self.encoder_count_arr[1] - self.last_encoder_count_arr[1]) + self.encoder_count_arr[2] - self.last_encoder_count_arr[2]
            length_delta = encoder_count_delta / self.encoder_ticks_per_rev * self.tendon_mm_per_rev
            
            self.tendon_length = self.tendon_length + length_delta

def main(args=None):
    rclpy.init(args=args)

    motor_controller = MotorController()

    spin_thread = Thread(target=rclpy.spin, args=(motor_controller,))
    spin_thread.start()

    rate = motor_controller.create_rate(20)

    try:
        while rclpy.ok():
            # gets data from arduino--of form 
            #"[motor1_encoder_counts] [motor1_velocities] [motor1_currents] , [motor2_encoder_counts] [motor2_velocities] [motor2_currents] , [motor3_encoder_counts] [motor3_velocities] [motor3_currents] ,"
            # ex. "0 0 0 1 0 0 0 219 ,0 0 0 1 0 0 0 115 ,0 0 0 1 0 0 0 129 ,""
            # motor_controller.arduino.reset_input_buffer()
            
            # motor_controller.write(motor_controller.convert_cmd_to_str([0,0,10]))
            
            measured_outputs_str = motor_controller.arduino.readline()
            print(measured_outputs_str)

            motor_measured_vals = None
            # parses data from arduino
            try:
                motor_measured_vals = motor_controller.parse_measured_outputs(measured_outputs_str)
            except UnparsableReceivedOutputsStrException:
                print("Unparsable received output!")
                continue

            # updates current values of velocity, current, encoder_count, etc.
            for i in range(len(motor_controller.motor_arr)):
                motor_controller.motor_arr[i].update_motor_vars(motor_measured_vals[i][0], motor_measured_vals[i][1], motor_measured_vals[i][2]) #TODO need to break out actual vals

            # create and set tendon length msg
            tendon_length_msg = Float64MultiArray()
            tendon_length_msg.data = [float(motor_controller.motor_arr[0].tendon_length), float(motor_controller.motor_arr[1].tendon_length), float(motor_controller.motor_arr[2].tendon_length)]
            
            # create motor vel msg
            motor_velocities_msg = Float64MultiArray()
            motor_velocities_msg.data = []
          
            # create motor encoder counts msg
            motor_encoder_counts_msg = Float64MultiArray()
            motor_encoder_counts_msg.data = []
            
            # set motor vel and encoder counts msg
            for motor in motor_controller.motor_arr:
                motor_velocities_msg.data.extend(motor.velocity_arr)
                motor_encoder_counts_msg.data.extend(motor.encoder_count_arr)
           
            # publishes tendon lengths, motor vel and encoder counts
            motor_controller.tendon_lengths_publisher.publish(tendon_length_msg)
            motor_controller.motor_velocities_publisher.publish(motor_velocities_msg) #TODO CHECK THAT COMMENTING THESE IN WORKS! 5/21/24
            motor_controller.motor_encoder_counts_publisher.publish(motor_encoder_counts_msg)

            rate.sleep()
    except KeyboardInterrupt:
        pass

    # stop motors after shutdown
    zero_msg = Float64MultiArray()
    zero_msg.data = [0.0,0.0,0.0]
    motor_controller.input_velocities_callback(zero_msg)
    
    print("rclpy shut down!")
    rclpy.shutdown()
    spin_thread.join()


if __name__ == '__main__':
    main()