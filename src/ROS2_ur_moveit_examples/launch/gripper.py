import socket
import numpy as np

class Gripper:
    def __init__(self, ip, port_read, port_write):
        """ Opens connection between a root and a host robot """
        assert isinstance(ip, str)
        assert isinstance(port_read, int) or isinstance(port_read, str)
        assert isinstance(port_write, int) or isinstance(port_write, str)
        if type(port_read) is str:
            port_read = int(port_read)

        if type(port_write) is str:
            port_write = int(port_write)

        self.ip = ip
        self.port_write = port_write
        self.port_read = port_read
        self.coordinates_mapping = None

        try:
            self.socket_read = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket_write = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket_read.connect((self.ip, self.port_read))
            self.socket_write.connect((self.ip, self.port_write))
            self.socket_write.settimeout(5)
            self.socket_read.settimeout(5)

            print("[Socket -- IP: {0}. Write port: {1}, read port: {2}]\n".format(ip, self.port_write, self.port_read))
        except(socket.error, exc):
            print("[Socket cannot be created. Exception occured:\n{0}]\n".format(exc))

        self.chunk_size = 8
        self.num_chunks = 6
        self.start_chunk_cartesian = 444
        self.start_chunk_joint = 252
        self.stop_chunk_cartesian = self.start_chunk_cartesian + self.num_chunks * self.chunk_size
        self.stop_chunk_joint = self.start_chunk_joint + self.num_chunks * self.chunk_size
        self.ur_package_size = 1060
        self.vec = None

    def __del__(self):
        """ Closes sockets created during initialization. """
        self.socket_read.close()
        self.socket_write.close()
        print("\n[Sockets succesfully closed.]\n".format(self.ip))
    
    def rg6_cmd(self, range_open, force=50):
        cmd_str = "def rg6ProgOpen():\n";
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

    def grip(self, range_open):
        """
        Sends command to open the RG6 gripper.
        :param range_open: number in [cm]
        :return: void
        """
        assert isinstance(range_open, float) or isinstance(range_open, int)
        command = self.rg6_cmd(range_open)
        self.socket_write.send(command)

if __name__ == '__main__':
    gripper = Gripper("192.168.1.101", 50002, 50003)










