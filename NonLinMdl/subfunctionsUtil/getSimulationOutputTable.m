function [currentOutTableTest, tictoc_LPVMPC,GenPwrRef] = getSimulationOutputTable(matFileOutTableTest1,loadData,OutTable,simMdlname1,varnames)
% OutTableTest1 retrieves simulation data, either from previous runs or by
% running the simulation. All inputs but varnames are required.
%
% Inputs
% - matFileOutTableTest1: mat file name
% - loadData: data loaded if available
% - OutTable: FAST simulation table
% - simMdlname1: Simulink mdl name
% - varnames (optional): variables names of simulation output
%
% Outputs
% - currentOutTableTest: Simulation results as a table
% - tictoc_LPVMPC: duration of qLPVMPC calculation
% - GenPwrRef: Power reference, calculated from windspeed

% set default variablenames
if nargin < 5 || isempty(varnames)
    varnames = {'Wind', 'RotSpeed', 'GenPwr', 'GenTq', 'BlPitch1', ...
    'NcIMUTAxs', 'NcIMUTAys'}; %%'Tower FA acc', 'Tower SS acc'};
end

% Get the name of the table
[~, strTable] = fileparts(matFileOutTableTest1);

% Get the simulation output
tictoc_LPVMPC = [];
GenPwrRef = [];
if exist(matFileOutTableTest1,'file') == 2 && loadData
    tmp = load(matFileOutTableTest1);
    currentOutTableTest = tmp.(strTable);
    if isfield(tmp,'tictoc_LPVMPC')
        tictoc_LPVMPC = tmp.tictoc_LPVMPC;
        GenPwrRef = tmp.GenPwrRef;
    end
else
    WS = 'base';
    % assignin(WS,'DT',DT)
    assignin(WS,'OutTable',OutTable)
    open_system(simMdlname1);
    out1 = sim(simMdlname1,'SrcWorkspace',WS);
    currentOutTableTest = array2table(out1.OutDataTest,'VariableNames',varnames);
    eval([strTable,' = currentOutTableTest;']);
    if contains(strTable,'MPC')
        tictoc_LPVMPC = out1.tictoc_LPVMPC;
        GenPwrRef = out1.GenPwrRef;
        save(matFileOutTableTest1,strTable,'tictoc_LPVMPC','GenPwrRef');
    else
        save(matFileOutTableTest1,strTable);
    end
end
