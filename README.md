# Part of a project with C++ as a sample
This repository is only a sample for part of my C++ code from a larger project called Local-HDP for robotic vision

# Simulated Teacher
In the Local-HDP project, a simulated teacher is responsible for introducing different 3D object-views to the model and ask the model to recognize their categories and provide correcting feedbacks when needed. 

This has two forms: 
* 1) offline: The offline evaluations use 10 fold cross validation for evaluating the performance of the model (not included in this repository)
* 2) open-ended: In particular, we have developed a simulated teacher which can interact with the model by either *teaching* a new category to it or *asking* the model to categorize the unforeseen object view. In case of wrong categorization of an object by the model, *correcting feedback* is sent to the model by the simulated teacher. In order to teach a new category, the simulated teacher presents three randomly selected object views of the corresponding category to the model. After teaching a new category, all of the previously learned categories are tested using a set of randomly selected unforeseen object views. Subsequently, the accuracy of category prediction is computed. In open-ended evaluation, the model observes the 3D objects one by one and the history of the latest *3n* predictions of the model is considered for calculating the accuracy, where *n* is the number of the learned categories. If the corresponding accuracy is higher than a certain threshold *T=0.66* (which means that the number of true-positives is at least twice the number of wrong predictions), the simulated teacher will teach a new category to the model. If the learning accuracy does not exceed the threshold *T* after a certain number of iterations (*100* for our experiments), the teacher infers that the agent is not able to learn more categories and the experiment stops. 

More details on the methodology can be found in the [arXiv version of the paper](https://arxiv.org/abs/2009.01152): 


