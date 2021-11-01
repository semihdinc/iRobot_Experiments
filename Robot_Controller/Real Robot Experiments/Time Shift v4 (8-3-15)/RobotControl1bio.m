function bio=RobotControl1bio
bio = [];
bio(1).blkName='status checker';
bio(1).sigName='enable';
bio(1).portIdx=0;
bio(1).dim=[1,1];
bio(1).sigWidth=1;
bio(1).sigAddress='&RobotControl1_B.enable';
bio(1).ndims=2;
bio(1).size=[];

bio(getlenBIO) = bio(1);

bio(2).blkName='Clock';
bio(2).sigName='';
bio(2).portIdx=0;
bio(2).dim=[1,1];
bio(2).sigWidth=1;
bio(2).sigAddress='&RobotControl1_B.Clock';
bio(2).ndims=2;
bio(2).size=[];


bio(3).blkName='packet size';
bio(3).sigName='';
bio(3).portIdx=0;
bio(3).dim=[1,1];
bio(3).sigWidth=1;
bio(3).sigAddress='&RobotControl1_B.packetsize';
bio(3).ndims=2;
bio(3).size=[];


bio(4).blkName='packet size4';
bio(4).sigName='';
bio(4).portIdx=0;
bio(4).dim=[1,1];
bio(4).sigWidth=1;
bio(4).sigAddress='&RobotControl1_B.packetsize4';
bio(4).ndims=2;
bio(4).size=[];


bio(5).blkName='RS232 Binary Receive1/p1';
bio(5).sigName='';
bio(5).portIdx=0;
bio(5).dim=[8,1];
bio(5).sigWidth=8;
bio(5).sigAddress='&RobotControl1_B.RS232BinaryReceive1_o2[0]';
bio(5).ndims=2;
bio(5).size=[];


bio(6).blkName='Add';
bio(6).sigName='';
bio(6).portIdx=0;
bio(6).dim=[1,1];
bio(6).sigWidth=1;
bio(6).sigAddress='&RobotControl1_B.Add';
bio(6).ndims=2;
bio(6).size=[];


bio(7).blkName='Unit Delay';
bio(7).sigName='';
bio(7).portIdx=0;
bio(7).dim=[1,1];
bio(7).sigWidth=1;
bio(7).sigAddress='&RobotControl1_B.UnitDelay';
bio(7).ndims=2;
bio(7).size=[];


bio(8).blkName='Unit Delay1';
bio(8).sigName='';
bio(8).portIdx=0;
bio(8).dim=[1,1];
bio(8).sigWidth=1;
bio(8).sigAddress='&RobotControl1_B.UnitDelay1';
bio(8).ndims=2;
bio(8).size=[];


bio(9).blkName='Open-loop control/ MATLAB Function1/p1';
bio(9).sigName='PWM_L';
bio(9).portIdx=0;
bio(9).dim=[1,1];
bio(9).sigWidth=1;
bio(9).sigAddress='&RobotControl1_B.PWM_L';
bio(9).ndims=2;
bio(9).size=[];


bio(10).blkName='Open-loop control/ MATLAB Function1/p2';
bio(10).sigName='PWM_R';
bio(10).portIdx=1;
bio(10).dim=[1,1];
bio(10).sigWidth=1;
bio(10).sigAddress='&RobotControl1_B.PWM_R';
bio(10).ndims=2;
bio(10).size=[];


bio(11).blkName='Open-loop control/Camera Servos Function/p1';
bio(11).sigName='PAN';
bio(11).portIdx=0;
bio(11).dim=[1,1];
bio(11).sigWidth=1;
bio(11).sigAddress='&RobotControl1_B.PAN';
bio(11).ndims=2;
bio(11).size=[];


bio(12).blkName='Open-loop control/Camera Servos Function/p2';
bio(12).sigName='TILT';
bio(12).portIdx=1;
bio(12).dim=[1,1];
bio(12).sigWidth=1;
bio(12).sigAddress='&RobotControl1_B.TILT';
bio(12).ndims=2;
bio(12).size=[];


bio(13).blkName='Open-loop control/MATLAB Function 2/p1';
bio(13).sigName='v';
bio(13).portIdx=0;
bio(13).dim=[1,1];
bio(13).sigWidth=1;
bio(13).sigAddress='&RobotControl1_B.v';
bio(13).ndims=2;
bio(13).size=[];


bio(14).blkName='Open-loop control/MATLAB Function 2/p2';
bio(14).sigName='omega';
bio(14).portIdx=1;
bio(14).dim=[1,1];
bio(14).sigWidth=1;
bio(14).sigAddress='&RobotControl1_B.omega';
bio(14).ndims=2;
bio(14).size=[];


bio(15).blkName='Read/Data Store Read';
bio(15).sigName='';
bio(15).portIdx=0;
bio(15).dim=[1,1];
bio(15).sigWidth=1;
bio(15).sigAddress='&RobotControl1_B.DataStoreRead';
bio(15).ndims=2;
bio(15).size=[];


bio(16).blkName='Status Extraction/Command State';
bio(16).sigName='CCS';
bio(16).portIdx=0;
bio(16).dim=[1,1];
bio(16).sigWidth=1;
bio(16).sigAddress='&RobotControl1_B.CCS';
bio(16).ndims=2;
bio(16).size=[];


bio(17).blkName='Status Extraction/Unpack/p1';
bio(17).sigName='';
bio(17).portIdx=0;
bio(17).dim=[1,1];
bio(17).sigWidth=1;
bio(17).sigAddress='&RobotControl1_B.Unpack_o1';
bio(17).ndims=2;
bio(17).size=[];


bio(18).blkName='Status Extraction/Unpack/p2';
bio(18).sigName='';
bio(18).portIdx=1;
bio(18).dim=[1,1];
bio(18).sigWidth=1;
bio(18).sigAddress='&RobotControl1_B.Unpack_o2';
bio(18).ndims=2;
bio(18).size=[];


bio(19).blkName='Status Extraction/Unpack/p3';
bio(19).sigName='';
bio(19).portIdx=2;
bio(19).dim=[1,1];
bio(19).sigWidth=1;
bio(19).sigAddress='&RobotControl1_B.Unpack_o3';
bio(19).ndims=2;
bio(19).size=[];


bio(20).blkName='Status Extraction/Unpack/p4';
bio(20).sigName='';
bio(20).portIdx=3;
bio(20).dim=[1,1];
bio(20).sigWidth=1;
bio(20).sigAddress='&RobotControl1_B.Unpack_o4';
bio(20).ndims=2;
bio(20).size=[];


bio(21).blkName='Status Extraction/Unpack/p5';
bio(21).sigName='';
bio(21).portIdx=4;
bio(21).dim=[1,1];
bio(21).sigWidth=1;
bio(21).sigAddress='&RobotControl1_B.Unpack_o5';
bio(21).ndims=2;
bio(21).size=[];


bio(22).blkName='Status Extraction/Unpack/p6';
bio(22).sigName='';
bio(22).portIdx=5;
bio(22).dim=[1,1];
bio(22).sigWidth=1;
bio(22).sigAddress='&RobotControl1_B.Unpack_o6';
bio(22).ndims=2;
bio(22).size=[];


bio(23).blkName='Status Extraction/Unpack/p7';
bio(23).sigName='';
bio(23).portIdx=6;
bio(23).dim=[1,1];
bio(23).sigWidth=1;
bio(23).sigAddress='&RobotControl1_B.Unpack_o7';
bio(23).ndims=2;
bio(23).size=[];


bio(24).blkName='Status Extraction/Unpack/p8';
bio(24).sigName='';
bio(24).portIdx=7;
bio(24).dim=[1,1];
bio(24).sigWidth=1;
bio(24).sigAddress='&RobotControl1_B.Unpack_o8';
bio(24).ndims=2;
bio(24).size=[];


bio(25).blkName='Open-loop control/plant/pulse transformer camera/p1';
bio(25).sigName='PAN_O';
bio(25).portIdx=0;
bio(25).dim=[1,1];
bio(25).sigWidth=1;
bio(25).sigAddress='&RobotControl1_B.PAN_O';
bio(25).ndims=2;
bio(25).size=[];


bio(26).blkName='Open-loop control/plant/pulse transformer camera/p2';
bio(26).sigName='TILT_O';
bio(26).portIdx=1;
bio(26).dim=[1,1];
bio(26).sigWidth=1;
bio(26).sigAddress='&RobotControl1_B.TILT_O';
bio(26).ndims=2;
bio(26).size=[];


bio(27).blkName='Open-loop control/plant/pulse transformer1/p1';
bio(27).sigName='Left_PWM';
bio(27).portIdx=0;
bio(27).dim=[1,1];
bio(27).sigWidth=1;
bio(27).sigAddress='&RobotControl1_B.Left_PWM';
bio(27).ndims=2;
bio(27).size=[];


bio(28).blkName='Open-loop control/plant/pulse transformer1/p2';
bio(28).sigName='Right_PWM';
bio(28).portIdx=1;
bio(28).dim=[1,1];
bio(28).sigWidth=1;
bio(28).sigAddress='&RobotControl1_B.Right_PWM';
bio(28).ndims=2;
bio(28).size=[];


bio(29).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/CheckSum/p1';
bio(29).sigName='csum0';
bio(29).portIdx=0;
bio(29).dim=[1,1];
bio(29).sigWidth=1;
bio(29).sigAddress='&RobotControl1_B.csum0';
bio(29).ndims=2;
bio(29).size=[];


bio(30).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/CheckSum/p2';
bio(30).sigName='csum1';
bio(30).portIdx=1;
bio(30).dim=[1,1];
bio(30).sigWidth=1;
bio(30).sigAddress='&RobotControl1_B.csum1';
bio(30).ndims=2;
bio(30).size=[];


bio(31).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Embedded MATLAB Function';
bio(31).sigName='CharArray';
bio(31).portIdx=0;
bio(31).dim=[22,1];
bio(31).sigWidth=22;
bio(31).sigAddress='&RobotControl1_B.CharArray[0]';
bio(31).ndims=2;
bio(31).size=[];


bio(32).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p1';
bio(32).sigName='';
bio(32).portIdx=0;
bio(32).dim=[1,1];
bio(32).sigWidth=1;
bio(32).sigAddress='&RobotControl1_B.ByteReversal_o1';
bio(32).ndims=2;
bio(32).size=[];


bio(33).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p2';
bio(33).sigName='';
bio(33).portIdx=1;
bio(33).dim=[1,1];
bio(33).sigWidth=1;
bio(33).sigAddress='&RobotControl1_B.ByteReversal_o2';
bio(33).ndims=2;
bio(33).size=[];


bio(34).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p3';
bio(34).sigName='';
bio(34).portIdx=2;
bio(34).dim=[1,1];
bio(34).sigWidth=1;
bio(34).sigAddress='&RobotControl1_B.ByteReversal_o3';
bio(34).ndims=2;
bio(34).size=[];


bio(35).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p4';
bio(35).sigName='';
bio(35).portIdx=3;
bio(35).dim=[1,1];
bio(35).sigWidth=1;
bio(35).sigAddress='&RobotControl1_B.ByteReversal_o4';
bio(35).ndims=2;
bio(35).size=[];


bio(36).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p5';
bio(36).sigName='';
bio(36).portIdx=4;
bio(36).dim=[1,1];
bio(36).sigWidth=1;
bio(36).sigAddress='&RobotControl1_B.ByteReversal_o5';
bio(36).ndims=2;
bio(36).size=[];


bio(37).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p6';
bio(37).sigName='';
bio(37).portIdx=5;
bio(37).dim=[1,1];
bio(37).sigWidth=1;
bio(37).sigAddress='&RobotControl1_B.ByteReversal_o6';
bio(37).ndims=2;
bio(37).size=[];


bio(38).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p7';
bio(38).sigName='';
bio(38).portIdx=6;
bio(38).dim=[1,1];
bio(38).sigWidth=1;
bio(38).sigAddress='&RobotControl1_B.ByteReversal_o7';
bio(38).ndims=2;
bio(38).size=[];


bio(39).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Byte Reversal/p8';
bio(39).sigName='';
bio(39).portIdx=7;
bio(39).dim=[1,1];
bio(39).sigWidth=1;
bio(39).sigAddress='&RobotControl1_B.ByteReversal_o8';
bio(39).ndims=2;
bio(39).size=[];


bio(40).blkName='Open-loop control/plant/Send Serial Packet to Servo Controller And recieve status message from SSC/Pack';
bio(40).sigName='';
bio(40).portIdx=0;
bio(40).dim=[22,1];
bio(40).sigWidth=22;
bio(40).sigAddress='&RobotControl1_B.Pack[0]';
bio(40).ndims=2;
bio(40).size=[];


function len = getlenBIO
len = 40;

