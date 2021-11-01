function bio=testbio
bio = [];
bio(1).blkName='packet size';
bio(1).sigName='';
bio(1).portIdx=0;
bio(1).dim=[1,1];
bio(1).sigWidth=1;
bio(1).sigAddress='&test_B.packetsize';
bio(1).ndims=2;
bio(1).size=[];

bio(getlenBIO) = bio(1);

bio(2).blkName='RS232 Binary Receive/p1';
bio(2).sigName='';
bio(2).portIdx=0;
bio(2).dim=[8,1];
bio(2).sigWidth=8;
bio(2).sigAddress='&test_B.RS232BinaryReceive_o2[0]';
bio(2).ndims=2;
bio(2).size=[];


bio(3).blkName='Status Extraction/Command State';
bio(3).sigName='CCS';
bio(3).portIdx=0;
bio(3).dim=[1,1];
bio(3).sigWidth=1;
bio(3).sigAddress='&test_B.CCS';
bio(3).ndims=2;
bio(3).size=[];


bio(4).blkName='Status Extraction/Unpack/p1';
bio(4).sigName='';
bio(4).portIdx=0;
bio(4).dim=[1,1];
bio(4).sigWidth=1;
bio(4).sigAddress='&test_B.Unpack_o1';
bio(4).ndims=2;
bio(4).size=[];


bio(5).blkName='Status Extraction/Unpack/p2';
bio(5).sigName='';
bio(5).portIdx=1;
bio(5).dim=[1,1];
bio(5).sigWidth=1;
bio(5).sigAddress='&test_B.Unpack_o2';
bio(5).ndims=2;
bio(5).size=[];


bio(6).blkName='Status Extraction/Unpack/p3';
bio(6).sigName='';
bio(6).portIdx=2;
bio(6).dim=[1,1];
bio(6).sigWidth=1;
bio(6).sigAddress='&test_B.Unpack_o3';
bio(6).ndims=2;
bio(6).size=[];


bio(7).blkName='Status Extraction/Unpack/p4';
bio(7).sigName='';
bio(7).portIdx=3;
bio(7).dim=[1,1];
bio(7).sigWidth=1;
bio(7).sigAddress='&test_B.Unpack_o4';
bio(7).ndims=2;
bio(7).size=[];


bio(8).blkName='Status Extraction/Unpack/p5';
bio(8).sigName='';
bio(8).portIdx=4;
bio(8).dim=[1,1];
bio(8).sigWidth=1;
bio(8).sigAddress='&test_B.Unpack_o5';
bio(8).ndims=2;
bio(8).size=[];


bio(9).blkName='Status Extraction/Unpack/p6';
bio(9).sigName='';
bio(9).portIdx=5;
bio(9).dim=[1,1];
bio(9).sigWidth=1;
bio(9).sigAddress='&test_B.Unpack_o6';
bio(9).ndims=2;
bio(9).size=[];


bio(10).blkName='Status Extraction/Unpack/p7';
bio(10).sigName='';
bio(10).portIdx=6;
bio(10).dim=[1,1];
bio(10).sigWidth=1;
bio(10).sigAddress='&test_B.Unpack_o7';
bio(10).ndims=2;
bio(10).size=[];


bio(11).blkName='Status Extraction/Unpack/p8';
bio(11).sigName='';
bio(11).portIdx=7;
bio(11).dim=[1,1];
bio(11).sigWidth=1;
bio(11).sigAddress='&test_B.Unpack_o8';
bio(11).ndims=2;
bio(11).size=[];


function len = getlenBIO
len = 11;

