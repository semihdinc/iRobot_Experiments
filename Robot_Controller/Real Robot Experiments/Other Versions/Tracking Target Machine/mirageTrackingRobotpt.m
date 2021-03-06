function pt=mirageTrackingRobotpt
pt = [];
pt(1).blockname = 'Desired Robot Pose';
pt(1).paramname = 'Value';
pt(1).class     = 'vector';
pt(1).nrows     = 1;
pt(1).ncols     = 6;
pt(1).subsource = 'SS_DOUBLE';
pt(1).ndims     = '2';
pt(1).size      = '[]';
pt(getlenPT) = pt(1);

pt(2).blockname = 'Object Model';
pt(2).paramname = 'Value';
pt(2).class     = 'col-mat';
pt(2).nrows     = 4;
pt(2).ncols     = 4;
pt(2).subsource = 'SS_DOUBLE';
pt(2).ndims     = '2';
pt(2).size      = '[]';

pt(3).blockname = 'From Left Camera';
pt(3).paramname = 'P1';
pt(3).class     = 'scalar';
pt(3).nrows     = 1;
pt(3).ncols     = 1;
pt(3).subsource = 'SS_DOUBLE';
pt(3).ndims     = '2';
pt(3).size      = '[]';

pt(4).blockname = 'From Left Camera';
pt(4).paramname = 'P2';
pt(4).class     = 'scalar';
pt(4).nrows     = 1;
pt(4).ncols     = 1;
pt(4).subsource = 'SS_DOUBLE';
pt(4).ndims     = '2';
pt(4).size      = '[]';

pt(5).blockname = 'From Left Camera';
pt(5).paramname = 'P3';
pt(5).class     = 'scalar';
pt(5).nrows     = 1;
pt(5).ncols     = 1;
pt(5).subsource = 'SS_DOUBLE';
pt(5).ndims     = '2';
pt(5).size      = '[]';

pt(6).blockname = 'From Left Camera';
pt(6).paramname = 'P4';
pt(6).class     = 'scalar';
pt(6).nrows     = 1;
pt(6).ncols     = 1;
pt(6).subsource = 'SS_DOUBLE';
pt(6).ndims     = '2';
pt(6).size      = '[]';

pt(7).blockname = 'From Left Camera';
pt(7).paramname = 'P5';
pt(7).class     = 'scalar';
pt(7).nrows     = 1;
pt(7).ncols     = 1;
pt(7).subsource = 'SS_DOUBLE';
pt(7).ndims     = '2';
pt(7).size      = '[]';

pt(8).blockname = 'From Left Camera';
pt(8).paramname = 'P6';
pt(8).class     = 'scalar';
pt(8).nrows     = 1;
pt(8).ncols     = 1;
pt(8).subsource = 'SS_DOUBLE';
pt(8).ndims     = '2';
pt(8).size      = '[]';

pt(9).blockname = 'From Left Camera';
pt(9).paramname = 'P7';
pt(9).class     = 'scalar';
pt(9).nrows     = 1;
pt(9).ncols     = 1;
pt(9).subsource = 'SS_DOUBLE';
pt(9).ndims     = '2';
pt(9).size      = '[]';

pt(10).blockname = 'From Left Camera';
pt(10).paramname = 'P8';
pt(10).class     = 'scalar';
pt(10).nrows     = 1;
pt(10).ncols     = 1;
pt(10).subsource = 'SS_DOUBLE';
pt(10).ndims     = '2';
pt(10).size      = '[]';

pt(11).blockname = 'From Left Camera';
pt(11).paramname = 'P9';
pt(11).class     = 'vector';
pt(11).nrows     = 1;
pt(11).ncols     = 2;
pt(11).subsource = 'SS_DOUBLE';
pt(11).ndims     = '2';
pt(11).size      = '[]';

pt(12).blockname = 'From Right Camera';
pt(12).paramname = 'P1';
pt(12).class     = 'scalar';
pt(12).nrows     = 1;
pt(12).ncols     = 1;
pt(12).subsource = 'SS_DOUBLE';
pt(12).ndims     = '2';
pt(12).size      = '[]';

pt(13).blockname = 'From Right Camera';
pt(13).paramname = 'P2';
pt(13).class     = 'scalar';
pt(13).nrows     = 1;
pt(13).ncols     = 1;
pt(13).subsource = 'SS_DOUBLE';
pt(13).ndims     = '2';
pt(13).size      = '[]';

pt(14).blockname = 'From Right Camera';
pt(14).paramname = 'P3';
pt(14).class     = 'scalar';
pt(14).nrows     = 1;
pt(14).ncols     = 1;
pt(14).subsource = 'SS_DOUBLE';
pt(14).ndims     = '2';
pt(14).size      = '[]';

pt(15).blockname = 'From Right Camera';
pt(15).paramname = 'P4';
pt(15).class     = 'scalar';
pt(15).nrows     = 1;
pt(15).ncols     = 1;
pt(15).subsource = 'SS_DOUBLE';
pt(15).ndims     = '2';
pt(15).size      = '[]';

pt(16).blockname = 'From Right Camera';
pt(16).paramname = 'P5';
pt(16).class     = 'scalar';
pt(16).nrows     = 1;
pt(16).ncols     = 1;
pt(16).subsource = 'SS_DOUBLE';
pt(16).ndims     = '2';
pt(16).size      = '[]';

pt(17).blockname = 'From Right Camera';
pt(17).paramname = 'P6';
pt(17).class     = 'scalar';
pt(17).nrows     = 1;
pt(17).ncols     = 1;
pt(17).subsource = 'SS_DOUBLE';
pt(17).ndims     = '2';
pt(17).size      = '[]';

pt(18).blockname = 'From Right Camera';
pt(18).paramname = 'P7';
pt(18).class     = 'scalar';
pt(18).nrows     = 1;
pt(18).ncols     = 1;
pt(18).subsource = 'SS_DOUBLE';
pt(18).ndims     = '2';
pt(18).size      = '[]';

pt(19).blockname = 'From Right Camera';
pt(19).paramname = 'P8';
pt(19).class     = 'scalar';
pt(19).nrows     = 1;
pt(19).ncols     = 1;
pt(19).subsource = 'SS_DOUBLE';
pt(19).ndims     = '2';
pt(19).size      = '[]';

pt(20).blockname = 'From Right Camera';
pt(20).paramname = 'P9';
pt(20).class     = 'vector';
pt(20).nrows     = 1;
pt(20).ncols     = 2;
pt(20).subsource = 'SS_DOUBLE';
pt(20).ndims     = '2';
pt(20).size      = '[]';

function len = getlenPT
len = 20;

