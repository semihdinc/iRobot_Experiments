function bio=untitledbio
bio = [];
bio(1).blkName='Sort for plotting/p1';
bio(1).sigName='longitude';
bio(1).portIdx=0;
bio(1).dim=[1,1];
bio(1).sigWidth=1;
bio(1).sigAddress='&untitled_B.longitude';
bio(1).ndims=2;
bio(1).size=[];

bio(getlenBIO) = bio(1);

bio(2).blkName='Sort for plotting/p2';
bio(2).sigName='latitude';
bio(2).portIdx=1;
bio(2).dim=[1,1];
bio(2).sigWidth=1;
bio(2).sigAddress='&untitled_B.latitude';
bio(2).ndims=2;
bio(2).size=[];


bio(3).blkName='Sort for plotting/p3';
bio(3).sigName='altitude';
bio(3).portIdx=2;
bio(3).dim=[1,1];
bio(3).sigWidth=1;
bio(3).sigAddress='&untitled_B.altitude';
bio(3).ndims=2;
bio(3).size=[];


bio(4).blkName='Sort for plotting/p4';
bio(4).sigName='X';
bio(4).portIdx=3;
bio(4).dim=[1,1];
bio(4).sigWidth=1;
bio(4).sigAddress='&untitled_B.X';
bio(4).ndims=2;
bio(4).size=[];


bio(5).blkName='Sort for plotting/p5';
bio(5).sigName='Y';
bio(5).portIdx=4;
bio(5).dim=[1,1];
bio(5).sigWidth=1;
bio(5).sigAddress='&untitled_B.Y';
bio(5).ndims=2;
bio(5).size=[];


bio(6).blkName='Sort for plotting/p6';
bio(6).sigName='Z';
bio(6).portIdx=5;
bio(6).dim=[1,1];
bio(6).sigWidth=1;
bio(6).sigAddress='&untitled_B.Z';
bio(6).ndims=2;
bio(6).size=[];


bio(7).blkName='Sort for plotting/p7';
bio(7).sigName='Xe';
bio(7).portIdx=6;
bio(7).dim=[1,1];
bio(7).sigWidth=1;
bio(7).sigAddress='&untitled_B.Xe';
bio(7).ndims=2;
bio(7).size=[];


bio(8).blkName='Sort for plotting/p8';
bio(8).sigName='Ye';
bio(8).portIdx=7;
bio(8).dim=[1,1];
bio(8).sigWidth=1;
bio(8).sigAddress='&untitled_B.Ye';
bio(8).ndims=2;
bio(8).size=[];


bio(9).blkName='Sort for plotting/p9';
bio(9).sigName='Ze';
bio(9).portIdx=8;
bio(9).dim=[1,1];
bio(9).sigWidth=1;
bio(9).sigAddress='&untitled_B.Ze';
bio(9).ndims=2;
bio(9).size=[];


function len = getlenBIO
len = 9;

