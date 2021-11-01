function bio=testModelbio
bio = [];
bio(1).blkName='From File';
bio(1).sigName='';
bio(1).portIdx=0;
bio(1).dim=[7,1];
bio(1).sigWidth=7;
bio(1).sigAddress='&testModel_B.FromFile[0]';
bio(1).ndims=2;
bio(1).size=[];

bio(getlenBIO) = bio(1);

function len = getlenBIO
len = 1;

