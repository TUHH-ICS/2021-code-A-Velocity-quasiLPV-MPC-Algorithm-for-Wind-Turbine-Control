%function generateReport
clc; clear; close all; bdclose all;
loadSimResults = 0;

%% Get Rotorsimulation model name
% Get the name of the model. Model is saved with a different name in R2018b.
simulinkModel = 'test_SimulinkMdl1_Baseline.slx'; %'Lebensdauer_mit_Aktuator_und_Lagerschaden.slx'; %'Lebensdauer_mit_Aktuator.slx'; 
versionRelease = version('-release');
is2018b = str2num(versionRelease(1:4)) >= 2018; %#ok<ST2NM> (2007b compatibility)

% Set outputfolder names
outDirFig = ['outFig',versionRelease];
if ~isdir(outDirFig) %#ok<ISDIR> isdir used for R2007b 
    mkdir(outDirFig)
end

outDirMat = ['outMat',versionRelease];
if ~isdir(outDirMat) %#ok<ISDIR> isdir used for R2007b 
    mkdir(outDirMat)
end
simoutFile = fullfile(outDirMat,'simout.mat');

%% Get scope names
% Get handle on scopes and set their data logging on

% Load Simulink model
simulinkModelPrint = simulinkModel(1:end-4);
open_system(simulinkModelPrint);

% Get handle on all scopes and print out
aBlockTypeScope = 'Scope';
listScopes = find_system(simulinkModelPrint,...
    'LookUnderMasks','all','BlockType', aBlockTypeScope);
fprintf('%s\n',listScopes{:});


%% Run Rotorsimulation model or load simulation data
% Run Rotorsimulation model or load simulation data

DT = 0.008;
load(outDataSimulationMat ,'OutTable');

% Run Rotorsimulation model if data is not available
if ~(exist(simoutFile,'file')== 2) || ~loadSimResults   
    % Set logging on for all scopes
    for idxScopes = 1: length(listScopes)
        aScope = listScopes{idxScopes};
        set_param(aScope,'SaveToWorkspace','on', 'DataFormat','StructureWithTime')
    end
    sim(simulinkModel);
    save(simoutFile,'ScopeData*')
    
    % Clean up
    clear ScopeData* 
    for idxScopes = 1: length(listScopes)
        aScope = listScopes{idxScopes};
        set_param(aScope,'SaveToWorkspace','off')
    end
else
    % Run to fill blade data display
    sim(simulinkModel,0);
end
simData = load(simoutFile);


%% Display the Simulink model as image
% Print Simulink model to png file. Color inports and outports to make them
%easily detectable.

% Get name of model, color in/ouports, create outDirfolder
colorInOutports(simulinkModelPrint ,'cyan','yellow');

% create full path to png file and display in MATLAB
simulinModelFile = fullfile(outDirFig,[simulinkModelPrint,'.jpg']);
print(['-s',simulinkModelPrint],'-djpeg',simulinModelFile);
img = imread(simulinModelFile);

if is2018b
    figure(1);
    imshow(img);
end

%% Display scopes main level
% Display scopes of main level in one plot.

% Gets the scopes on base layer 
simDataCell = struct2cell(simData);
simDataName = cellfun(@(x) x.blockName, simDataCell, 'UniformOutput',false);
cellScopesSim = regexp(simDataName, '/', 'split');
isMainScope = cellfun(@length,cellScopesSim) == 2; 

simDataCellMain = simDataCell(isMainScope);
simDataStructSub = simDataCell(~isMainScope);
simDataCellSubName = cellScopesSim(~isMainScope);

lenBlockName = cell2mat(cellfun(@(x) max(strfind(x,'/')),...
    simDataName(~isMainScope), 'UniformOutput',false));

% create the plots as one figure with subplots as well as 
hf = figure(is2018b + 1); % there is a figure in 2018b, no figure in 2008b
for idxMain = 1: length(simDataCellMain)
    
    aScopeStruct = simDataCellMain{idxMain}; %  list is ordered alphatically
    
    subplot(length(simDataCellMain),1,idxMain)
    plot(aScopeStruct.time,aScopeStruct.signals.values);
    axis tight; grid on;
    ht = title(aScopeStruct.blockName); %ht.Interpreter = 'none';  
    set(ht,'Interpreter','none');
end
xlabel('Time (s)');

if isnumeric(hf)
    hfNumber = hf;
    saveas(hfNumber,fullfile(outDirFig,'MainScope'),'png');
else
    hfNumber = hf.Number;
    saveas(hfNumber,fullfile(outDirFig,'MainScope'),'png');
    saveas(hfNumber,fullfile(outDirFig,'MainScope'),'fig');
end


% there is a figure in 2018b, no figure in 2008b
for idxMain = 1: length(simDataCellMain)
    
    aScopeStruct = simDataCellMain{idxMain}; %  list is ordered alphatically
    
    hf = figure(hfNumber +1); %subplot(length(simDataCellMain),1,idxMain)
    plot(aScopeStruct.time,aScopeStruct.signals.values);
    axis tight; grid on;
    ht = title(aScopeStruct.blockName); %ht.Interpreter = 'none';  
    set(ht,'Interpreter','none');
    if isnumeric(hf)
        saveas(hf,fullfile(outDirFig,sprintf('Scope%02d',hf)),'png');
    else
        hfNumber = hf.Number;
        saveas(hfNumber,fullfile(outDirFig,sprintf('Scope%02d',hfNumber)),'png');
        saveas(hfNumber,fullfile(outDirFig,sprintf('Scope%02d',hfNumber)),'fig');
    end
    
   hfNumber = hfNumber +1;
end
xlabel('Time (s)');

%% Get subsystem names
% Get subsystem names with find_system. Print out with fprintf.

% Find subsystems with find_system
aBlockType = 'SubSystem';
listSubsysAll = find_system(simulinkModelPrint,...
    'LookUnderMasks','all','BlockType', aBlockType);

% Remove unnecessary subsystems from list
%
% * Rotorsimulation/Manual Switch'
% * Rotorsimulation/Manual Switch1'
% * Rotorsimulation/Ramp'
% * Rotorsimulation/Windstufensimulation/Manual Switch
listSubsys = setdiff(listSubsysAll,...
    {[simulinkModelPrint, '/Manual Switch'],...
    [simulinkModelPrint, '/Manual Switch1'],...
    [simulinkModelPrint, '/Ramp'],...
    [simulinkModelPrint, '/Windstufensimulation/Manual Switch']});

fprintf('%s\n',listSubsys{:});


%% Display subsystems as images
% Displaye subsystems as images with imshow (not available in R2008b)
% If there are only a  be few subsystems displaying them by looping
% over the list listSubsys is a valid, easily readable solution. 

for idxSubsys = 1: length(listSubsys)
    aSimulinkModelPrint = listSubsys{idxSubsys};
    fprintf('%s\n',aSimulinkModelPrint);
    if ~is2018b
        open_system(aSimulinkModelPrint); 
    end
    aPic = fullfile(outDirFig,sprintf('%s_Subsys%02d.jgg',simulinkModelPrint,idxSubsys));
    print(['-s', aSimulinkModelPrint],'-djpeg',aPic)
    if ~is2018b
        close_system(aSimulinkModelPrint);
    end
    
    if is2018b
        img = imread(aPic);
        stringTitle = strrep(aSimulinkModelPrint,[simulinkModelPrint,'/'],'');
        hf = figure(hf.Number+1);
        imshow(img);
        ht = title(stringTitle); %ht.Interpreter ='none';
        set(ht,'Interpreter','none');
    end
     
    aSubSysCell = regexp(aSimulinkModelPrint, '/', 'split');
    aSubSysCellCompare = aSubSysCell(2:end);
    
    for idxSub = 1: length(simDataStructSub)
        if isequal(simDataCellSubName{idxSub}(2:end-1),aSubSysCellCompare)
            aScopeStruct = simDataStructSub{idxSub}; %  list is ordered alphatically
            if is2018b
                hf = figure(hf.Number+1);
            else
                hf = figure(hf+1);
            end
            for idxSubplots = 1:length(aScopeStruct.signals)
                subplot(length(aScopeStruct.signals),1,idxSubplots)
                plot(aScopeStruct.time,aScopeStruct.signals(idxSubplots).values);
                axis tight; grid on;
                ht = title(aScopeStruct.blockName); %ht.Interpreter = 'none';
                set(ht,'Interpreter','none');
                if isnumeric(hf)
                    saveas(hf,fullfile(outDirFig,sprintf('Scope%02d',hf)),'png');
                end
            xlabel('Time (s)');
            end
        end
    end
    drawnow;
end
bdclose all;

%% Unused Code

% % Set logging on for all scopes
% There is no ScopeConfiguration property in R2008b
% for idxScopes = 1: length(listScopes)
%     aScope = listScopes{idxScopes};
%     set_param(aScope,'SaveToWorkspace','on', 'DataFormat','StructureWithTime')
% %     scopeConfig = get_param(aScope,'ScopeConfiguration');
% %     scopeConfig.DataLogging = true;
% %     scopeConfig.DataLoggingSaveFormat = 'Dataset';
% end


% % Save scope output data to stucture simData 
% currentWorkspace = who;
% simData = struct;
% keyword = 'ScopeData';
% lenKeyword = length(keyword);
% for idx = 1: length(currentWorkspace)
%     aVar = currentWorkspace{idx};
%     if strcmp(aVar(1:min(length(aVar),lenKeyword)),keyword)
%         eval(['simData.(''',aVar,''') = ',aVar]);
%     end
% end
