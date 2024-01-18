function [prompt_number,storage_variable] = insert_prompt(prompt_title,prompt_text)

% INSERT ON-SCREEN PROMPT. 
% prompt_text: text to be displayed on the prompt;
% prompt_title: title of the prompt;
% prompt_number: number of the prompt;
% storage_variable: variable to which the end value will be stored

prompt_number = {prompt_text}; % create a prompt on the screen
dlgtitle = prompt_title; % name the prompt
dims = [1 35]; % establish the size of the prompt
storage_variable = str2double(inputdlg(prompt_number,dlgtitle,dims)); % store the user inputted data


