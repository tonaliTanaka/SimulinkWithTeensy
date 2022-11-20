%Set names
modelName = 'model'; 
dirRoot = 'ProjectTemplate/';
dirBuild = append(dirRoot, append(modelName, '_ert_rtw/'));
dirPlt = append(dirRoot, 'src/');
dirArdLib = '/Users/xxxxxxx/Documents/MATLAB/SupportPackages/R2022a/toolbox/target/supportpackages/arduinobase/';

%Set embedded coder
cs = getActiveConfigSet(modelName);
switchTarget(cs,'ert.tlc',[]);

%Generate C-code
slbuild(modelName);

%Copy generated files to 'src' directory of PlatformIO
copyfile(append(dirBuild, '*.c'), dirPlt, 'f');
copyfile(append(dirBuild, '*.h'), dirPlt, 'f');

%Convert model file from C-file to C++-file
movefile(append(dirPlt, append(modelName, '.c')), append(dirPlt, append(modelName,'.cpp')), 'f');

%Copy files of 'MW_arduino_digital_io' to 'src' directory of PlatformIO
copyfile(append(dirArdLib, 'include/MW_arduino_digitalio.h'), dirPlt, 'f');
copyfile(append(dirArdLib, 'src/MW_arduino_digitalio.cpp'), dirPlt, 'f');

%Build generated C++-code to Teensy.
system('pio run')