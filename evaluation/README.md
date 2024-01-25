# EVALUATION
In this folder you find all files to evaluate the users. [CenterData](/evaluation/CenterData/) contains the tracked position for all experiments. 

## plot.ipynb
Here you can evaluate a single experiment for a user and compare it to the ground truth and Roberta's tracker. Plots are created and the error & frequency of the experiment are calculated. 
To save the plots and metrics, make sure to set the 'Save' parameter to True. Then, the plots will be saved at the corresponding user in the [CenterData](/evaluation/CenterData/) folder. 

The error and frequency are saved in the file corresponding to the experiment in [error](/evaluation/error/) and [frequency](/evaluation/frequency/).

## evaluation.ipynb
In this file you can evaluate all experiments for a single user. It just finds the error and the frequency. No plots.

The error and frequency are saved in the file corresponding to the experiment in [error](/evaluation/error/) and [frequency](/evaluation/frequency/).