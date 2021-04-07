% script to update data dictionary
clear;
[wecs, M, Ce, K, Q, L, rho, tau, kappa, lambda, beta,Cq,Ct] = initModel5MWNREL;
K5 = [K,-K(:,4)];

DDName = 'DD_Mdl1.sldd';
DDDataNames = {'beta';'Ce';'Cq';'Ct';'K5'; 'lambda'; 'M';'Q'; 'rho';'wecs'};

myDictionaryObj = Simulink.data.dictionary.open(DDName);
dDataSectObj = getSection(myDictionaryObj,'Design Data');


for idx = 1: length(DDDataNames)
   tempObj = getEntry(dDataSectObj,DDDataNames{idx});
   eval(['setValue(tempObj, ',DDDataNames{idx},')']);
end

saveChanges(myDictionaryObj)

% listEntry(myDictionaryObj)
%   Design Data   beta              DD_Mdl1.sldd   2021-04-02 13:18   ditt_aj          double   
%   Design Data   Ce                DD_Mdl1.sldd   2021-04-02 13:18   ditt_aj          double   
%   Design Data   Cq                DD_Mdl1.sldd   2021-04-02 13:18   ditt_aj          double   
%   Design Data   Ct                DD_Mdl1.sldd   2021-04-02 13:18   ditt_aj          double   
%   Design Data   K5                DD_Mdl1.sldd   2021-04-02 13:18   ditt_aj          double   
%   Design Data   lambda            DD_Mdl1.sldd   2021-04-02 13:18   ditt_aj          double   
%   Design Data   M                 DD_Mdl1.sldd   2021-04-02 13:18   ditt_aj          double   
%   Design Data   Q                 DD_Mdl1.sldd   2021-04-02 13:18   ditt_aj          double   
%   Design Data   rho               DD_Mdl1.sldd   2021-04-02 13:18   ditt_aj          double   
%   Design Data   wecs              DD_Mdl1.sldd   2021-04-02 13:18   ditt_aj          struct   


