function bio=mirageTrackingRobotbio
bio = [];
bio(1).blkName='Calculate Target Points/p1';
bio(1).sigName='L_p';
bio(1).portIdx=0;
bio(1).dim=[2,4];
bio(1).sigWidth=8;
bio(1).sigAddress='&mirageTrackingRobot_B.L_p[0]';
bio(1).ndims=2;
bio(1).size=[];

bio(getlenBIO) = bio(1);

bio(2).blkName='Calculate Target Points/p2';
bio(2).sigName='R_p';
bio(2).portIdx=1;
bio(2).dim=[2,4];
bio(2).sigWidth=8;
bio(2).sigAddress='&mirageTrackingRobot_B.R_p[0]';
bio(2).ndims=2;
bio(2).size=[];


bio(3).blkName='Estimate Pose Error';
bio(3).sigName='qtilde';
bio(3).portIdx=0;
bio(3).dim=[6,1];
bio(3).sigWidth=6;
bio(3).sigAddress='&mirageTrackingRobot_B.qtilde[0]';
bio(3).ndims=2;
bio(3).size=[];


bio(4).blkName='From Left Camera';
bio(4).sigName='';
bio(4).portIdx=0;
bio(4).dim=[1,1];
bio(4).sigWidth=3686400;
bio(4).sigAddress='&mirageTrackingRobot_B.FromLeftCamera[0]';
bio(4).ndims=3;
bio(4).size=[960, 1280, 3];


bio(5).blkName='From Right Camera';
bio(5).sigName='';
bio(5).portIdx=0;
bio(5).dim=[1,1];
bio(5).sigWidth=3686400;
bio(5).sigAddress='&mirageTrackingRobot_B.FromRightCamera[0]';
bio(5).ndims=3;
bio(5).size=[960, 1280, 3];


function len = getlenBIO
len = 5;

