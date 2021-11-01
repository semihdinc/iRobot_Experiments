function bio=RobotControl1_640_480bio
bio = [];
bio(1).blkName='status checker';
bio(1).sigName='enable';
bio(1).portIdx=0;
bio(1).dim=[1,1];
bio(1).sigWidth=1;
bio(1).sigAddress='&RobotControl1_640_480_B.enable';
bio(1).ndims=2;
bio(1).size=[];

bio(getlenBIO) = bio(1);

bio(2).blkName='Clock';
bio(2).sigName='';
bio(2).portIdx=0;
bio(2).dim=[1,1];
bio(2).sigWidth=1;
bio(2).sigAddress='&RobotControl1_640_480_B.Clock';
bio(2).ndims=2;
bio(2).size=[];


bio(3).blkName='packet size';
bio(3).sigName='';
bio(3).portIdx=0;
bio(3).dim=[1,1];
bio(3).sigWidth=1;
bio(3).sigAddress='&RobotControl1_640_480_B.packetsize';
bio(3).ndims=2;
bio(3).size=[];


bio(4).blkName='packet size4';
bio(4).sigName='';
bio(4).portIdx=0;
bio(4).dim=[1,1];
bio(4).sigWidth=1;
bio(4).sigAddress='&RobotControl1_640_480_B.packetsize4';
bio(4).ndims=2;
bio(4).size=[];


bio(5).blkName='RS232 Binary Receive1/p1';
bio(5).sigName='';
bio(5).portIdx=0;
bio(5).dim=[8,1];
bio(5).sigWidth=8;
bio(5).sigAddress='&RobotControl1_640_480_B.RS232BinaryReceive1_o2[0]';
bio(5).ndims=2;
bio(5).size=[];


bio(6).blkName='Open-loop control/Camera Servos Function/p1';
bio(6).sigName='PAN';
bio(6).portIdx=0;
bio(6).dim=[1,1];
bio(6).sigWidth=1;
bio(6).sigAddress='&RobotControl1_640_480_B.PAN';
bio(6).ndims=2;
bio(6).size=[];


bio(7).blkName='Open-loop control/Camera Servos Function/p2';
bio(7).sigName='TILT';
bio(7).portIdx=1;
bio(7).dim=[1,1];
bio(7).sigWidth=1;
bio(7).sigAddress='&RobotControl1_640_480_B.TILT';
bio(7).ndims=2;
bio(7).size=[];


bio(8).blkName='Open-loop control/PWM Calculation/p1';
bio(8).sigName='PWM_L';
bio(8).portIdx=0;
bio(8).dim=[1,1];
bio(8).sigWidth=1;
bio(8).sigAddress='&RobotControl1_640_480_B.PWM_L';
bio(8).ndims=2;
bio(8).size=[];


bio(9).blkName='Open-loop control/PWM Calculation/p2';
bio(9).sigName='PWM_R';
bio(9).portIdx=1;
bio(9).dim=[1,1];
bio(9).sigWidth=1;
bio(9).sigAddress='&RobotControl1_640_480_B.PWM_R';
bio(9).ndims=2;
bio(9).size=[];


bio(10).blkName='Status Extraction/Command State';
bio(10).sigName='CCS';
bio(10).portIdx=0;
bio(10).dim=[1,1];
bio(10).sigWidth=1;
bio(10).sigAddress='&RobotControl1_640_480_B.CCS';
bio(10).ndims=2;
bio(10).size=[];


bio(11).blkName='Status Extraction/Unpack/p1';
bio(11).sigName='';
bio(11).portIdx=0;
bio(11).dim=[1,1];
bio(11).sigWidth=1;
bio(11).sigAddress='&RobotControl1_640_480_B.Unpack_o1';
bio(11).ndims=2;
bio(11).size=[];


bio(12).blkName='Status Extraction/Unpack/p2';
bio(12).sigName='';
bio(12).portIdx=1;
bio(12).dim=[1,1];
bio(12).sigWidth=1;
bio(12).sigAddress='&RobotControl1_640_480_B.Unpack_o2';
bio(12).ndims=2;
bio(12).size=[];


bio(13).blkName='Status Extraction/Unpack/p3';
bio(13).sigName='';
bio(13).portIdx=2;
bio(13).dim=[1,1];
bio(13).sigWidth=1;
bio(13).sigAddress='&RobotControl1_640_480_B.Unpack_o3';
bio(13).ndims=2;
bio(13).size=[];


bio(14).blkName='Status Extraction/Unpack/p4';
bio(14).sigName='';
bio(14).portIdx=3;
bio(14).dim=[1,1];
bio(14).sigWidth=1;
bio(14).sigAddress='&RobotControl1_640_480_B.Unpack_o4';
bio(14).ndims=2;
bio(14).size=[];


bio(15).blkName='Status Extraction/Unpack/p5';
bio(15).sigName='';
bio(15).portIdx=4;
bio(15).dim=[1,1];
bio(15).sigWidth=1;
bio(15).sigAddress='&RobotControl1_640_480_B.Unpack_o5';
bio(15).ndims=2;
bio(15).size=[];


bio(16).blkName='Status Extraction/Unpack/p6';
bio(16).sigName='';
bio(16).portIdx=5;
bio(16).dim=[1,1];
bio(16).sigWidth=1;
bio(16).sigAddress='&RobotControl1_640_480_B.Unpack_o6';
bio(16).ndims=2;
bio(16).size=[];


bio(17).blkName='Status Extraction/Unpack/p7';
bio(17).sigName='';
bio(17).portIdx=6;
bio(17).dim=[1,1];
bio(17).sigWidth=1;
bio(17).sigAddress='&RobotControl1_640_480_B.Unpack_o7';
bio(17).ndims=2;
bio(17).size=[];


bio(18).blkName='Status Extraction/Unpack/p8';
bio(18).sigName='';
bio(18).portIdx=7;
bio(18).dim=[1,1];
bio(18).sigWidth=1;
bio(18).sigAddress='&RobotControl1_640_480_B.Unpack_o8';
bio(18).ndims=2;
bio(18).size=[];


bio(19).blkName='Timer control/Subtract';
bio(19).sigName='';
bio(19).portIdx=0;
bio(19).dim=[1,1];
bio(19).sigWidth=1;
bio(19).sigAddress='&RobotControl1_640_480_B.Subtract';
bio(19).ndims=2;
bio(19).size=[];


bio(20).blkName='Timer control/Subtract1';
bio(20).sigName='';
bio(20).portIdx=0;
bio(20).dim=[1,1];
bio(20).sigWidth=1;
bio(20).sigAddress='&RobotControl1_640_480_B.Subtract1';
bio(20).ndims=2;
bio(20).size=[];


bio(21).blkName='Timer control/Switch';
bio(21).sigName='';
bio(21).portIdx=0;
bio(21).dim=[1,1];
bio(21).sigWidth=1;
bio(21).sigAddress='&RobotControl1_640_480_B.Switch';
bio(21).ndims=2;
bio(21).size=[];


bio(22).blkName='Timer control/Unary Minus';
bio(22).sigName='';
bio(22).portIdx=0;
bio(22).dim=[1,1];
bio(22).sigWidth=1;
bio(22).sigAddress='&RobotControl1_640_480_B.UnaryMinus';
bio(22).ndims=2;
bio(22).size=[];


bio(23).blkName='Open-loop control/Vision System/Calculate Target Points/p1';
bio(23).sigName='L_p';
bio(23).portIdx=0;
bio(23).dim=[2,4];
bio(23).sigWidth=8;
bio(23).sigAddress='&RobotControl1_640_480_B.L_p[0]';
bio(23).ndims=2;
bio(23).size=[];


bio(24).blkName='Open-loop control/Vision System/Calculate Target Points/p2';
bio(24).sigName='R_p';
bio(24).portIdx=1;
bio(24).dim=[2,4];
bio(24).sigWidth=8;
bio(24).sigAddress='&RobotControl1_640_480_B.R_p[0]';
bio(24).ndims=2;
bio(24).size=[];


bio(25).blkName='Open-loop control/Vision System/Desired Pose V2/p1';
bio(25).sigName='qr';
bio(25).portIdx=0;
bio(25).dim=[6,1];
bio(25).sigWidth=6;
bio(25).sigAddress='&RobotControl1_640_480_B.qr[0]';
bio(25).ndims=2;
bio(25).size=[];


bio(26).blkName='Open-loop control/Vision System/Desired Pose V2/p2';
bio(26).sigName='ur';
bio(26).portIdx=1;
bio(26).dim=[2,1];
bio(26).sigWidth=2;
bio(26).sigAddress='&RobotControl1_640_480_B.ur[0]';
bio(26).ndims=2;
bio(26).size=[];


bio(27).blkName='Open-loop control/Vision System/Non-Holonomic Robot Controller/p1';
bio(27).sigName='v';
bio(27).portIdx=0;
bio(27).dim=[1,1];
bio(27).sigWidth=1;
bio(27).sigAddress='&RobotControl1_640_480_B.v';
bio(27).ndims=2;
bio(27).size=[];


bio(28).blkName='Open-loop control/Vision System/Non-Holonomic Robot Controller/p2';
bio(28).sigName='omega';
bio(28).portIdx=1;
bio(28).dim=[1,1];
bio(28).sigWidth=1;
bio(28).sigAddress='&RobotControl1_640_480_B.omega';
bio(28).ndims=2;
bio(28).size=[];


bio(29).blkName='Open-loop control/Vision System/Pose Estimation';
bio(29).sigName='q';
bio(29).portIdx=0;
bio(29).dim=[6,1];
bio(29).sigWidth=6;
bio(29).sigAddress='&RobotControl1_640_480_B.q[0]';
bio(29).ndims=2;
bio(29).size=[];


bio(30).blkName='Open-loop control/Vision System/Left Camera';
bio(30).sigName='';
bio(30).portIdx=0;
bio(30).dim=[1,1];
bio(30).sigWidth=921600;
bio(30).sigAddress='&RobotControl1_640_480_B.LeftCamera[0]';
bio(30).ndims=3;
bio(30).size=[480, 640, 3];


bio(31).blkName='Open-loop control/Vision System/Right Camera';
bio(31).sigName='';
bio(31).portIdx=0;
bio(31).dim=[1,1];
bio(31).sigWidth=921600;
bio(31).sigAddress='&RobotControl1_640_480_B.RightCamera[0]';
bio(31).ndims=3;
bio(31).size=[480, 640, 3];


bio(32).blkName='Open-loop control/plant/pulse transformer camera/p1';
bio(32).sigName='PAN_O';
bio(32).portIdx=0;
bio(32).dim=[1,1];
bio(32).sigWidth=1;
bio(32).sigAddress='&RobotControl1_640_480_B.PAN_O';
bio(32).ndims=2;
bio(32).size=[];


bio(33).blkName='Open-loop control/plant/pulse transformer camera/p2';
bio(33).sigName='TILT_O';
bio(33).portIdx=1;
bio(33).dim=[1,1];
bio(33).sigWidth=1;
bio(33).sigAddress='&RobotControl1_640_480_B.TILT_O';
bio(33).ndims=2;
bio(33).size=[];


bio(34).blkName='Open-loop control/plant/pulse transformer1/p1';
bio(34).sigName='Left_PWM';
bio(34).portIdx=0;
bio(34).dim=[1,1];
bio(34).sigWidth=1;
bio(34).sigAddress='&RobotControl1_640_480_B.Left_PWM';
bio(34).ndims=2;
bio(34).size=[];


bio(35).blkName='Open-loop control/plant/pulse transformer1/p2';
bio(35).sigName='Right_PWM';
bio(35).portIdx=1;
bio(35).dim=[1,1];
bio(35).sigWidth=1;
bio(35).sigAddress='&RobotControl1_640_480_B.Right_PWM';
bio(35).ndims=2;
bio(35).size=[];


bio(36).blkName='Timer control/Read/Data Store Read';
bio(36).sigName='';
bio(36).portIdx=0;
bio(36).dim=[1,1];
bio(36).sigWidth=1;
bio(36).sigAddress='&RobotControl1_640_480_B.DataStoreRead';
bio(36).ndims=2;
bio(36).size=[];


bio(37).blkName='Open-loop control/Vision System/All Scopes/Mean';
bio(37).sigName='';
bio(37).portIdx=0;
bio(37).dim=[1,1];
bio(37).sigWidth=1;
bio(37).sigAddress='&RobotControl1_640_480_B.Mean';
bio(37).ndims=2;
bio(37).size=[];


bio(38).blkName='Open-loop control/Vision System/All Scopes/Tapped Delay';
bio(38).sigName='';
bio(38).portIdx=0;
bio(38).dim=[2,1];
bio(38).sigWidth=2;
bio(38).sigAddress='&RobotControl1_640_480_B.TappedDelay[0]';
bio(38).ndims=2;
bio(38).size=[];


bio(39).blkName='Open-loop control/Vision System/All Scopes/theta_tilde';
bio(39).sigName='etheta';
bio(39).portIdx=0;
bio(39).dim=[1,1];
bio(39).sigWidth=1;
bio(39).sigAddress='&RobotControl1_640_480_B.etheta';
bio(39).ndims=2;
bio(39).size=[];


bio(40).blkName='Open-loop control/Vision System/All Scopes/xtilde';
bio(40).sigName='ex';
bio(40).portIdx=0;
bio(40).dim=[1,1];
bio(40).sigWidth=1;
bio(40).sigAddress='&RobotControl1_640_480_B.ex';
bio(40).ndims=2;
bio(40).size=[];


bio(41).blkName='Open-loop control/Vision System/All Scopes/ytilde';
bio(41).sigName='ey';
bio(41).portIdx=0;
bio(41).dim=[1,1];
bio(41).sigWidth=1;
bio(41).sigAddress='&RobotControl1_640_480_B.ey';
bio(41).ndims=2;
bio(41).size=[];


bio(42).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/CheckSum/p1';
bio(42).sigName='csum0';
bio(42).portIdx=0;
bio(42).dim=[1,1];
bio(42).sigWidth=1;
bio(42).sigAddress='&RobotControl1_640_480_B.csum0';
bio(42).ndims=2;
bio(42).size=[];


bio(43).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/CheckSum/p2';
bio(43).sigName='csum1';
bio(43).portIdx=1;
bio(43).dim=[1,1];
bio(43).sigWidth=1;
bio(43).sigAddress='&RobotControl1_640_480_B.csum1';
bio(43).ndims=2;
bio(43).size=[];


bio(44).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Embedded MATLAB Function';
bio(44).sigName='CharArray';
bio(44).portIdx=0;
bio(44).dim=[22,1];
bio(44).sigWidth=22;
bio(44).sigAddress='&RobotControl1_640_480_B.CharArray[0]';
bio(44).ndims=2;
bio(44).size=[];


bio(45).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p1';
bio(45).sigName='';
bio(45).portIdx=0;
bio(45).dim=[1,1];
bio(45).sigWidth=1;
bio(45).sigAddress='&RobotControl1_640_480_B.ByteReversal_o1';
bio(45).ndims=2;
bio(45).size=[];


bio(46).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p2';
bio(46).sigName='';
bio(46).portIdx=1;
bio(46).dim=[1,1];
bio(46).sigWidth=1;
bio(46).sigAddress='&RobotControl1_640_480_B.ByteReversal_o2';
bio(46).ndims=2;
bio(46).size=[];


bio(47).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p3';
bio(47).sigName='';
bio(47).portIdx=2;
bio(47).dim=[1,1];
bio(47).sigWidth=1;
bio(47).sigAddress='&RobotControl1_640_480_B.ByteReversal_o3';
bio(47).ndims=2;
bio(47).size=[];


bio(48).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p4';
bio(48).sigName='';
bio(48).portIdx=3;
bio(48).dim=[1,1];
bio(48).sigWidth=1;
bio(48).sigAddress='&RobotControl1_640_480_B.ByteReversal_o4';
bio(48).ndims=2;
bio(48).size=[];


bio(49).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p5';
bio(49).sigName='';
bio(49).portIdx=4;
bio(49).dim=[1,1];
bio(49).sigWidth=1;
bio(49).sigAddress='&RobotControl1_640_480_B.ByteReversal_o5';
bio(49).ndims=2;
bio(49).size=[];


bio(50).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p6';
bio(50).sigName='';
bio(50).portIdx=5;
bio(50).dim=[1,1];
bio(50).sigWidth=1;
bio(50).sigAddress='&RobotControl1_640_480_B.ByteReversal_o6';
bio(50).ndims=2;
bio(50).size=[];


bio(51).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p7';
bio(51).sigName='';
bio(51).portIdx=6;
bio(51).dim=[1,1];
bio(51).sigWidth=1;
bio(51).sigAddress='&RobotControl1_640_480_B.ByteReversal_o7';
bio(51).ndims=2;
bio(51).size=[];


bio(52).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p8';
bio(52).sigName='';
bio(52).portIdx=7;
bio(52).dim=[1,1];
bio(52).sigWidth=1;
bio(52).sigAddress='&RobotControl1_640_480_B.ByteReversal_o8';
bio(52).ndims=2;
bio(52).size=[];


bio(53).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Pack';
bio(53).sigName='';
bio(53).portIdx=0;
bio(53).dim=[22,1];
bio(53).sigWidth=22;
bio(53).sigAddress='&RobotControl1_640_480_B.Pack[0]';
bio(53).ndims=2;
bio(53).size=[];


function len = getlenBIO
len = 53;

