function colorInOutports(aSimulinkMdl,aBackgroundColorOut,aBackgroundColorIn)
% color in and outports of a model

if ~nargin
    aSimulinkMdl = bdroot;
    if isempty(aSimulinkMdl)
        warning('No Simulink model open. Return without action')
        return;
    end
end

if nargin < 2 || isempty(aBackgroundColorOut)
    aBackgroundColorOut = 'yellow'; % 'lightblue'; %'magenta';
end

if nargin < 3 || isempty(aBackgroundColorIn)
    aBackgroundColorIn = 'gray'; %'green';% 'cyan';
end

% BlockType, 'Inport'; BackgroundColor, 'magenta'
aBlockType = 'Inport';
colorInports(aSimulinkMdl, aBlockType, aBackgroundColorIn)

% BlockType, 'Outport'; BackgroundColor, 'cyan'
aBlockType = 'Outport';

colorInports(aSimulinkMdl, aBlockType, aBackgroundColorOut)

function colorInports(aSimulinkMdl, aBlockType, aBackgroundColor)

gbInports = find_system(aSimulinkMdl,...           % search vdp for all Gain blocks,
    'LookUnderMasks','all','BlockType', aBlockType);
for idx = 1: length(gbInports)
    set_param(gbInports{idx},'BackgroundColor',aBackgroundColor)
end
