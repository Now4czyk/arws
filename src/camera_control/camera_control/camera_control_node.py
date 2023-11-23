#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import std_msgs

class GripperNode (Node):
    def __init__ (self):
        super().__init__("py_testt")
        self.counter_ = 0
        self.get_logger ().info("Hello  Gripper Node")
        self.publisher_ = self.create_publisher(std_msgs.msg.String, "urscript_interface", 1)
        ms = std_msgs.msg.String()
        ms.data = self.generate_string()

        self.publisher_.publish(ms)
        print("Published message")
    
    def generate_string(self):
        ms = self.rg6_cmd()
        return ms

    def rg6_cmd(self, range_open=4, force=40):
        cmd_str = "def rg2ProgOpen():\n";
        cmd_str += "\ttextmsg(\"inside RG6 function called\")\n";

        cmd_str += "\ttarget_width={0}\n".format(range_open);
        cmd_str += "\ttarget_force={0}\n".format(force);
        cmd_str += "\tpayload=1.0\n";
        cmd_str += "\tset_payload1=False\n";
        cmd_str += "\tdepth_compensation=False\n";
        cmd_str += "\tslave=False\n";

        cmd_str += "\ttimeout = 0\n";
        cmd_str += "\twhile get_digital_in(9) == False:\n";
        cmd_str += "\t\ttextmsg(\"inside while\")\n";
        cmd_str += "\t\tif timeout > 400:\n";
        cmd_str += "\t\t\tbreak\n";
        cmd_str += "\t\tend\n";
        cmd_str += "\t\ttimeout = timeout+1\n";
        cmd_str += "\t\tsync()\n";
        cmd_str += "\tend\n";
        cmd_str += "\ttextmsg(\"outside while\")\n";

        cmd_str += "\tdef bit(input):\n";
        cmd_str += "\t\tmsb=65536\n";
        cmd_str += "\t\tlocal i=0\n";
        cmd_str += "\t\tlocal output=0\n";
        cmd_str += "\t\twhile i<17:\n";
        cmd_str += "\t\t\tset_digital_out(8,True)\n";
        cmd_str += "\t\t\tif input>=msb:\n";
        cmd_str += "\t\t\t\tinput=input-msb\n";
        cmd_str += "\t\t\t\tset_digital_out(9,False)\n";
        cmd_str += "\t\t\telse:\n";
        cmd_str += "\t\t\t\tset_digital_out(9,True)\n";
        cmd_str += "\t\t\tend\n";
        cmd_str += "\t\t\tif get_digital_in(8):\n";
        cmd_str += "\t\t\t\tout=1\n";
        cmd_str += "\t\t\tend\n";
        cmd_str += "\t\t\tsync()\n";
        cmd_str += "\t\t\tset_digital_out(8,False)\n";
        cmd_str += "\t\t\tsync()\n";
        cmd_str += "\t\t\tinput=input*2\n";
        cmd_str += "\t\t\toutput=output*2\n";
        cmd_str += "\t\t\ti=i+1\n";
        cmd_str += "\t\tend\n";
        cmd_str += "\t\treturn output\n";
        cmd_str += "\tend\n";
        cmd_str += "\ttextmsg(\"outside bit definition\")\n";
        #TU GRZEBIE
        cmd_str += "\ttarget_width=target_width+9.2\n";
        cmd_str += "\tif target_force>120:\n";
        cmd_str += "\t\ttarget_force=120\n";
        cmd_str += "\tend\n";
        cmd_str += "\tif target_force<25:\n";
        cmd_str += "\t\ttarget_force=25\n";
        cmd_str += "\tend\n";
        cmd_str += "\tif target_width>160:\n";
        cmd_str += "\t\ttarget_width=160\n";
        cmd_str += "\tend\n";
        cmd_str += "\tif target_width<0:\n";
        cmd_str += "\t\ttarget_width=0\n";
        cmd_str += "\tend\n";
        cmd_str += "\trg_data=floor(target_width)*4\n";
        cmd_str += "\trg_data=rg_data+floor(target_force/5)*4*161\n";
        cmd_str += "\tif slave:\n";
        cmd_str += "\t\trg_data=rg_data+16384\n";
        cmd_str += "\tend\n";

        cmd_str += "\ttextmsg(\"about to call bit\")\n";
        cmd_str += "\tbit(rg_data)\n";
        cmd_str += "\ttextmsg(\"called bit\")\n";

        cmd_str += "\tif depth_compensation:\n";
        cmd_str += "\t\tfinger_length = 80.0/1000\n";
        cmd_str += "\t\tfinger_heigth_disp = 6.3/1000\n";
        cmd_str += "\t\tcenter_displacement = 10.5/1000\n";

        cmd_str += "\t\tglobal start_pose = get_forward_kin()\n";
        cmd_str += "\t\tset_analog_inputrange(2, 1)\n";
        cmd_str += "\t\tzscale = (get_analog_in(2)-0.026)/2.976\n";
        cmd_str += "\t\tzangle = zscale*1.57079633-0.0942477796\n";
        cmd_str += "\t\tzwidth = 8.4+160*sin(zangle)\n";

        cmd_str += "\t\tstart_depth = cos(zangle)*finger_length\n";

        cmd_str += "\t\tsync()\n";
        cmd_str += "\t\tsync()\n";
        cmd_str += "\t\ttimeout = 0\n";

        cmd_str += "\t\twhile get_digital_in(9) == True:\n";
        cmd_str += "\t\t\ttimeout=timeout+1\n";
        cmd_str += "\t\t\tsync()\n";
        cmd_str += "\t\t\tif timeout > 20:\n";
        cmd_str += "\t\t\t\tbreak\n";
        cmd_str += "\t\t\tend\n";
        cmd_str += "\t\tend\n";
        cmd_str += "\t\ttimeout = 0\n";
        cmd_str += "\t\twhile get_digital_in(9) == False:\n";
        cmd_str += "\t\t\tzscale = (get_analog_in(2)-0.026)/2.976\n";
        cmd_str += "\t\t\tzangle = zscale*1.57079633-0.0942477796\n";
        cmd_str += "\t\t\tzwidth = 8.4+160*sin(zangle)\n";
        cmd_str += "\t\t\tmeasure_depth = cos(zangle)*finger_length\n";
        cmd_str += "\t\t\tcompensation_depth = (measure_depth - start_depth)\n";
        cmd_str += "\t\t\ttarget_pose = pose_trans(start_pose,p[0,0,-compensation_depth,0,0,0])\n";
        cmd_str += "\t\t\tif timeout > 400:\n";
        cmd_str += "\t\t\t\tbreak\n";
        cmd_str += "\t\t\tend\n";
        cmd_str += "\t\t\ttimeout=timeout+1\n";
        cmd_str += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.01,2000)\n";
        cmd_str += "\t\tend\n";
        cmd_str += "\t\tnspeed = norm(get_actual_tcp_speed())\n";
        cmd_str += "\t\twhile nspeed > 0.001:\n";
        cmd_str += "\t\t\tservoj(get_inverse_kin(target_pose),0,0,0.008,0.01,2000)\n";
        cmd_str += "\t\t\tnspeed = norm(get_actual_tcp_speed())\n";
        cmd_str += "\t\tend\n";
        cmd_str += "\tend\n";
        cmd_str += "\tif depth_compensation==False:\n";
        cmd_str += "\t\ttimeout = 0\n";
        cmd_str += "\t\twhile get_digital_in(9) == True:\n";
        cmd_str += "\t\t\ttimeout = timeout+1\n";
        cmd_str += "\t\t\tsync()\n";
        cmd_str += "\t\t\tif timeout > 20:\n";
        cmd_str += "\t\t\t\tbreak\n";
        cmd_str += "\t\t\tend\n";
        cmd_str += "\t\tend\n";
        cmd_str += "\t\ttimeout = 0\n";
        cmd_str += "\t\twhile get_digital_in(9) == False:\n";
        cmd_str += "\t\t\ttimeout = timeout+1\n";
        cmd_str += "\t\t\tsync()\n";
        cmd_str += "\t\t\tif timeout > 400:\n";
        cmd_str += "\t\t\t\tbreak\n";
        cmd_str += "\t\t\tend\n";
        cmd_str += "\t\tend\n";
        cmd_str += "\tend\n";
        cmd_str += "\tif set_payload1:\n";
        cmd_str += "\t\tif slave:\n";
        cmd_str += "\t\t\tif get_analog_in(3) < 2:\n";
        cmd_str += "\t\t\t\tzslam=0\n";
        cmd_str += "\t\t\telse:\n";
        cmd_str += "\t\t\t\tzslam=payload\n";
        cmd_str += "\t\t\tend\n";
        cmd_str += "\t\telse:\n";
        cmd_str += "\t\t\tif get_digital_in(8) == False:\n";
        cmd_str += "\t\t\t\tzmasm=0\n";
        cmd_str += "\t\t\telse:\n";
        cmd_str += "\t\t\t\tzmasm=payload\n";
        cmd_str += "\t\t\tend\n";
        cmd_str += "\t\tend\n";
        cmd_str += "\t\tzsysm=0.0\n";
        cmd_str += "\t\tzload=zmasm+zslam+zsysm\n";
        cmd_str += "\t\tset_payload(zload)\n";
        cmd_str += "\tend\n";
        cmd_str += "end\n\n";

        return cmd_str

def main (args=None):
    rclpy.init(args=args)
    node = GripperNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()