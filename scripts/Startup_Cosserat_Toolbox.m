%Add all required paths to matlab PATH variable.
thisDir = fileparts(mfilename('fullpath'));
cd(thisDir)
addpath('../functions')
addpath('../classes')
addpath('../scripts')
addpath('../plots')
addpath('../codegen')
cd ..